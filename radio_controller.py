import serial
import time
import threading


class RadioController:
    """
    Controls one Arduino Nano + nRF24 radio.

    Pi -> USB Serial -> Nano -> nRF24 -> RX

    The Nano has:
        - permanent RADIO_ID
        - dynamic RX target
        - unique RF control address per RX
    """

    def __init__(
        self,
        port,
        radio_id=None,
        baudrate=115200
    ):
        self.port = port
        self.radio_id = radio_id
        self.baudrate = baudrate

        self.serial = None
        self.connected = False

        self.sequence = 0

        self.last_tx_time = 0.0
        self.last_control_log_time = 0.0
        self.last_ack_time = 0.0

        self.last_ack_sequence = None
        self.last_ack_status = None
        self.last_ack_failsafe = None

        # ----------------------------------------------------
        # Target state
        # ----------------------------------------------------

        self.selected_receiver_id = None

        # Permanent RX/vehicle pairing reported by the Nano itself
        # (PAIRING command). New-architecture Nanos have a fixed RX
        # address baked into firmware; this is never chosen by the Pi.
        self.receiver_id = None
        self.vehicle_name = None

        self.discovered_receivers = {}

        self.lock = threading.Lock()

        self.command_lock = threading.Lock()

    # ========================================================
    # CONNECTION
    # ========================================================

    def connect(self):
        """Open the Nano serial port."""

        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.05
            )

            # Opening the Arduino serial port resets the Nano.
            time.sleep(2.0)

            self.serial.reset_input_buffer()

            self.connected = True

            print(
                f"[RadioController] Connected: "
                f"{self.port}"
            )

            if self.radio_id:
                print(
                    f"[RadioController] Radio ID: "
                    f"{self.radio_id}"
                )

            return True

        except Exception as e:

            print(
                f"[RadioController] Connection failed "
                f"{self.port}: {e}"
            )

            self.connected = False
            self.serial = None

            return False

    # ========================================================
    # DISCONNECT
    # ========================================================

    def disconnect(self):
        """Close the Nano serial connection."""

        with self.lock:

            self.connected = False

            if self.serial:

                try:
                    self.serial.close()

                except Exception:
                    pass

            self.serial = None

        print(
            f"[RadioController] Disconnected: "
            f"{self.port}"
        )

    # ========================================================
    # SERIAL COMMAND
    # ========================================================

    def _send_command(self, command):
        """Send one command to the Nano."""

        with self.lock:

            if (
                not self.connected
                or not self.serial
            ):
                return False

            try:

                self.serial.write(
                    (command + "\n").encode("ascii")
                )

                self.serial.flush()

                return True

            except Exception as e:

                print(
                    f"[RadioController] "
                    f"Serial write failed: {e}"
                )

                self.connected = False

                return False

    def _drain_lines_until(self, prefixes, timeout=0.25):
        """Read Nano serial until one matching prefix arrives.

        Returns the matched line, or None on timeout.
        """
        if isinstance(prefixes, str):
            prefixes = (prefixes,)

        prefixes = tuple(str(p) for p in prefixes)
        deadline = time.time() + float(timeout)
        matched = None

        while time.time() < deadline:
            line = self.read_line()
            if not line:
                continue

            for prefix in prefixes:
                if line.startswith(prefix):
                    matched = line
                    # Keep draining briefly for related ACK lines.
                    extra_deadline = time.time() + 0.05
                    while time.time() < extra_deadline:
                        extra = self.read_line()
                        if not extra:
                            break
                        if extra.startswith(("ACK_", "TX_")):
                            # Useful RF diagnostics from Nano firmware.
                            if extra.startswith("ACK_") or extra.startswith("TX_"):
                                if not hasattr(self, "_last_nano_rf_lines"):
                                    self._last_nano_rf_lines = []
                                self._last_nano_rf_lines.append(extra)
                    return matched

        return matched

    def _send_rf_command(self, command, ok_prefixes, fail_prefixes, timeout=0.35):
        """Send a Nano command and require an RF result line.

        Serial write success alone is NOT enough. Fixed-pairing Nanos report
        real nRF24 result as TX_OK / TX_FAIL / SESSION_*_SENT / etc.
        """
        with self.command_lock:
            # Drop stale lines so we don't match an old TX_OK.
            try:
                if self.serial:
                    self.serial.reset_input_buffer()
            except Exception:
                pass

            if not self._send_command(command):
                return False, None

            self._last_nano_rf_lines = []
            line = self._drain_lines_until(
                tuple(ok_prefixes) + tuple(fail_prefixes),
                timeout=timeout
            )

            if line is None:
                return False, None

            for prefix in fail_prefixes:
                if line.startswith(prefix):
                    return False, line

            for prefix in ok_prefixes:
                if line.startswith(prefix):
                    return True, line

            return False, line

    # ========================================================
    # CONTROL
    # ========================================================

    def send_control(
        self,
        steering,
        throttle
    ):
        """
        Send steering and throttle to the Nano over USB.

        Matches the proven standalone Pi test path:
          - write CONTROL,<steer>,<thr> immediately
          - do NOT block waiting for TX_OK/TX_FAIL
          - drain Nano replies non-blocking for logs only

        Fixed-pairing firmware already targets its baked-in RX.
        selected_receiver_id is metadata only (not required for TX).
        """

        steering = int(steering)
        throttle = int(throttle)

        steering = max(-32768, min(32767, steering))
        throttle = max(-32768, min(32767, throttle))

        command = f"CONTROL,{steering},{throttle}"

        # Fire-and-forget USB write. Waiting for TX_OK here was the main
        # difference vs the working standalone test and stalled the 50 Hz loop.
        if not self._send_command(command):
            return False

        self.last_tx_time = time.time()
        self.sequence = (self.sequence + 1) & 0xFFFF

        # Non-blocking drain of Nano RF result lines for diagnostics only.
        # Never reset_input_buffer here — that drops live TX_OK/FAIL noise
        # and is not how the working test drives the car.
        result_line = None
        ack_hint = ""
        try:
            while self.serial and self.serial.in_waiting:
                line = self.read_line()
                if not line:
                    break
                if line.startswith(("TX_OK:", "TX_FAIL:", "TX_BLOCKED:")):
                    result_line = line
                elif line.startswith("ACK_VALID"):
                    ack_hint = " ACK_VALID"
                elif line.startswith("ACK_NONE"):
                    ack_hint = " ACK_NONE"
                elif line.startswith("ACK_INVALID"):
                    ack_hint = " ACK_INVALID"
        except Exception:
            pass

        now = time.time()
        if now - self.last_control_log_time >= 1.0:
            self.last_control_log_time = now
            target = self.selected_receiver_id or self.receiver_id or "?"
            if result_line and result_line.startswith("TX_FAIL"):
                print(
                    f"[RadioController] CONTROL_USB_OK RF_FAIL -> {target}: "
                    f"steering={steering} throttle={throttle} "
                    f"({result_line})"
                )
            else:
                print(
                    f"[RadioController] CONTROL_USB_OK -> {target}: "
                    f"steering={steering} throttle={throttle}"
                    f"{f' ({result_line})' if result_line else ''}"
                    f"{ack_hint}"
                )

        # USB write success is enough for the control loop to continue.
        # RF ACK is best-effort and must not gate the next frame.
        return True

    # ========================================================
    # SESSION
    # ========================================================

    def send_session_start(self, duration_seconds):
        """
        Start/reset the RF session via Nano.

        Matches standalone test: write SESSION_START,<seconds> and continue.
        Do not require selected_receiver_id — fixed-pairing Nano already
        knows its RX.
        """
        try:
            duration_seconds = int(duration_seconds)
        except (TypeError, ValueError):
            print(
                "[RadioController] "
                "Invalid session duration"
            )
            return False

        if duration_seconds <= 0:
            print(
                "[RadioController] "
                "Invalid session duration: "
                f"{duration_seconds}"
            )
            return False

        if duration_seconds > 0xFFFFFFFF:
            duration_seconds = 0xFFFFFFFF

        command = f"SESSION_START,{duration_seconds}"

        # Same as working test.py: write and drain replies, don't hard-fail
        # the whole session if RF ACK is slow.
        if not self._send_command(command):
            return False

        time.sleep(0.3)
        replies = []
        try:
            while self.serial and self.serial.in_waiting:
                line = self.read_line()
                if line:
                    replies.append(line)
        except Exception:
            pass

        target = self.selected_receiver_id or self.receiver_id or "?"
        print(
            "[RadioController] "
            f"SESSION_START sent: {duration_seconds}s -> {target} "
            f"replies={replies}"
        )
        return True

    def send_session_stop(self):
        """Stop RF session via Nano (fire-and-forget like standalone test)."""
        if not self._send_command("SESSION_STOP"):
            return False

        time.sleep(0.2)
        replies = []
        try:
            while self.serial and self.serial.in_waiting:
                line = self.read_line()
                if line:
                    replies.append(line)
        except Exception:
            pass

        target = self.selected_receiver_id or self.receiver_id or "?"
        print(
            "[RadioController] "
            f"SESSION_STOP sent -> {target} replies={replies}"
        )
        return True

    def send_lap(
        self,
        lap_count,
        last_lap_ms,
        best_lap_ms
    ):
        """
        Send lap timing data to the selected RX.
        """

        try:
            lap_count = int(lap_count)
            last_lap_ms = int(last_lap_ms)
            best_lap_ms = int(best_lap_ms)
        except (TypeError, ValueError):
            print(
                "[RadioController] "
                "Invalid lap data"
            )
            return False

        if lap_count < 0 or lap_count > 0xFFFF:
            print(
                "[RadioController] "
                f"Invalid lap count: {lap_count}"
            )
            return False

        if last_lap_ms < 0 or last_lap_ms > 0xFFFFFFFF:
            print(
                "[RadioController] "
                f"Invalid last lap: {last_lap_ms}"
            )
            return False

        if best_lap_ms < 0 or best_lap_ms > 0xFFFFFFFF:
            print(
                "[RadioController] "
                f"Invalid best lap: {best_lap_ms}"
            )
            return False

        if not self.selected_receiver_id:
            print(
                "[RadioController] "
                "LAP BLOCKED: no RX target selected"
            )
            return False

        command = (
            f"LAP,"
            f"{lap_count},"
            f"{last_lap_ms},"
            f"{best_lap_ms}"
        )

        success = self._send_command(command)

        if success:
            print(
                "[RadioController] "
                f"LAP sent: "
                f"{lap_count},"
                f"{last_lap_ms},"
                f"{best_lap_ms}"
                f" -> {self.selected_receiver_id}"
            )

        return success

    # ========================================================
    # DISCOVER RXs
    # ========================================================

    def discover_receivers(
        self,
        timeout=2.0
    ):
        """
        Ask the Nano to discover all currently reachable RXs.

        The complete DISCOVER request/response transaction is
        protected so control/other commands cannot interleave
        with the discovery response.
        """

        with self.command_lock:

            if not self._send_command(
                "DISCOVER"
            ):
                return {}

            discovered = {}

            deadline = (
                time.time() + timeout
            )

            while time.time() < deadline:

                line = self.read_line()

                if not line:
                    time.sleep(0.005)
                    continue

                if line.startswith(
                    "RX_FOUND:"
                ):

                    receiver_id = (
                        line.split(
                            ":",
                            1
                        )[1].strip()
                    )

                    discovered[
                        receiver_id
                    ] = ""

                    continue

                if line.startswith(
                    "VEHICLE_NAME:"
                ):

                    vehicle_name = (
                        line.split(
                            ":",
                            1
                        )[1].strip()
                    )

                    if discovered:

                        last_receiver = (
                            list(
                                discovered.keys()
                            )[-1]
                        )

                        discovered[
                            last_receiver
                        ] = vehicle_name

                    continue

                if line == (
                    "DISCOVERY_COMPLETE"
                ):
                    break

            self.discovered_receivers = (
                discovered.copy()
            )

            print(
                f"[RadioController] "
                f"Discovered {len(discovered)} RX(s)"
            )

            for (
                receiver_id,
                vehicle_name
            ) in discovered.items():

                print(
                    f"  RX: {receiver_id}"
                    f"  Name: {vehicle_name}"
                )

            return discovered

    # ========================================================
    # SELECT RX TARGET
    # ========================================================

    def select_target(
        self,
        receiver_id,
        verify=True,
        timeout=2.0
    ):
        """
        Select one RX as the current control target.

        The Nano changes its nRF24 writing address to the
        unique address belonging to this RX.
        """

        receiver_id = (
            str(receiver_id)
            .strip()
            .upper()
        )

        if len(receiver_id) != 12:

            print(
                "[RadioController] "
                f"Invalid RX ID: {receiver_id}"
            )

            return False

        # ----------------------------------------------------
        # If we have a discovery list, require the target
        # to have been discovered.
        # ----------------------------------------------------

        if (
            self.discovered_receivers
            and receiver_id
            not in self.discovered_receivers
        ):

            print(
                "[RadioController] "
                f"RX not discovered: {receiver_id}"
            )

            return False

        with self.command_lock:

            if self.serial is not None:
                try:
                    self.serial.reset_input_buffer()
                except Exception:
                    pass

            if not self._send_command(
                f"TARGET,{receiver_id}"
            ):
                return False

            if not verify:

                self.selected_receiver_id = (
                    receiver_id
                )

                return True

            deadline = (
                time.time() + timeout
            )

            while time.time() < deadline:

                line = self.read_line()

                if not line:
                    time.sleep(0.005)
                    continue

                expected = (
                    f"TARGET_SELECTED:{receiver_id}"
                )

                if line == expected:

                    self.selected_receiver_id = (
                        receiver_id
                    )

                    print(
                        "[RadioController] "
                        f"Target selected: {receiver_id}"
                    )

                    return True

                if line.startswith(
                    "TARGET_NOT_FOUND:"
                ):

                    print(
                        "[RadioController] "
                        f"Nano rejected target: "
                        f"{receiver_id}"
                    )

                    return False

                if line == "TARGET_INVALID":

                    print(
                        "[RadioController] "
                        "Nano rejected invalid target"
                    )

                    return False

        print(
            "[RadioController] "
            f"Target selection timeout: "
            f"{receiver_id}"
        )

        return False

    # ========================================================
    # CLEAR TARGET
    # ========================================================

    def clear_target(self):
        """
        Clear the Pi-side target state.

        The Nano currently does not have a CLEAR_TARGET
        command, so this only prevents the Pi from sending
        control through this controller.
        """

        self.selected_receiver_id = None

    # ========================================================
    # FIXED RX/VEHICLE PAIRING
    # ========================================================

    def sync_pairing(self):
        """Query the Nano's permanent RX/vehicle pairing.

        New-architecture Nanos are hardcoded to one RX and report it via
        PAIRING (RF_ID / VEHICLE) instead of accepting a dynamic TARGET
        selection. The paired RX becomes the control target immediately;
        no TARGET command is ever sent for normal cockpit assignment.

        Accepted response lines from Nano firmware:
          RF_ID:<12 hex>
          PAIRED_RF_ID:<12 hex>
          VEHICLE:<name>
        """

        with self.command_lock:

            if not self._send_command("PAIRING"):
                return False

            rx_id = None
            vehicle_name = None

            deadline = time.time() + 1.5

            while time.time() < deadline:

                line = self.read_line()

                if not line:
                    continue

                upper = line.upper()

                if upper.startswith("RF_ID:") or upper.startswith("PAIRED_RF_ID:"):
                    rx_id = line.split(":", 1)[1].strip().upper()

                elif upper.startswith("VEHICLE:"):
                    vehicle_name = line.split(":", 1)[1].strip()
                    # Keep reading briefly in case RF_ID arrives after VEHICLE.
                    if rx_id:
                        break

        if not rx_id or len(rx_id) != 12:
            print(
                f"[RadioController] PAIRING failed for "
                f"{self.radio_id}: invalid RF ID {rx_id!r}"
            )
            return False

        self.receiver_id = rx_id
        self.vehicle_name = vehicle_name or None
        self.selected_receiver_id = rx_id

        print(
            f"[RadioController] Paired: {self.radio_id} -> "
            f"{rx_id} ({self.vehicle_name})"
        )

        return True

    # ========================================================
    # WHO
    # ========================================================

    def who(self):
        """Ask the Nano for its permanent radio ID."""

        with self.command_lock:

            if not self._send_command(
                "WHO"
            ):
                return None

            deadline = (
                time.time() + 1.0
            )

            while time.time() < deadline:

                line = self.read_line()

                if not line:
                    continue

                if line.startswith(
                    "RADIO_ID:"
                ):

                    return line.split(
                        ":",
                        1
                    )[1].strip()

            return None

    # ========================================================
    # STATUS
    # ========================================================

    def status(self):
        """Ask the Nano for radio status."""

        with self.command_lock:

            if not self._send_command(
                "STATUS"
            ):
                return []

            lines = []

            deadline = (
                time.time() + 1.0
            )

            while time.time() < deadline:

                line = self.read_line()

                if not line:
                    continue

                lines.append(line)

                if line.startswith(
                    "NRF24:"
                ):
                    break

            return lines

    # ========================================================
    # PING
    # ========================================================

    def ping(self):
        """Check whether the Nano responds."""
        with self.command_lock:

            if not self._send_command(
                "PING"
            ):
                return False

            deadline = (
                time.time() + 1.0
            )

            while time.time() < deadline:

                line = self.read_line()

                if line == "PONG":
                    return True

            return False

    # ========================================================
    # READ SERIAL
    # ========================================================

    def read_line(self):
        """Read one line from the Nano."""

        if (
            not self.connected
            or not self.serial
        ):
            return None

        try:

            if self.serial.in_waiting <= 0:
                return None

            line = (
                self.serial.readline()
                .decode(
                    "ascii",
                    errors="replace"
                )
                .strip()
            )

            if not line:
                return None

            self._process_line(line)

            return line

        except Exception as e:

            print(
                f"[RadioController] "
                f"Serial read failed: {e}"
            )

            self.connected = False

            return None

    # ========================================================
    # PROCESS NANO RESPONSE
    # ========================================================

    def _process_line(self, line):

        if line.startswith(
            "ACK_SEQUENCE:"
        ):

            try:

                self.last_ack_sequence = int(
                    line.split(
                        ":",
                        1
                    )[1]
                )

            except ValueError:
                pass

        elif line.startswith(
            "ACK_STATUS:"
        ):

            try:

                self.last_ack_status = int(
                    line.split(
                        ":",
                        1
                    )[1]
                )

            except ValueError:
                pass

        elif line.startswith(
            "ACK_FAILSAFE:"
        ):

            try:

                self.last_ack_failsafe = int(
                    line.split(
                        ":",
                        1
                    )[1]
                )

            except ValueError:
                pass

        elif line == "ACK_VALID":

            self.last_ack_time = (
                time.time()
            )

    # ========================================================
    # TEST
    # ========================================================

    def test(self):
        """
        Test:

        1. Connect
        2. WHO
        3. PING
        4. STATUS
        5. Discover RXs
        6. Select first RX
        7. Send control
        """

        print()
        print("==============================")
        print("RadioController Test")
        print("==============================")

        # ----------------------------------------------------
        # Connect
        # ----------------------------------------------------

        if not self.connect():

            print(
                "RESULT: CONNECT_FAILED"
            )

            return False

        # ----------------------------------------------------
        # WHO
        # ----------------------------------------------------

        print()

        radio_id = self.who()

        print(
            f"WHO: {radio_id}"
        )

        # ----------------------------------------------------
        # PING
        # ----------------------------------------------------

        print()

        print(
            "PING:",
            self.ping()
        )

        # ----------------------------------------------------
        # STATUS
        # ----------------------------------------------------

        print()

        print(
            "STATUS:"
        )

        for line in self.status():

            print(
                " ",
                line
            )

        # ----------------------------------------------------
        # DISCOVERY
        # ----------------------------------------------------

        print()

        print(
            "DISCOVERY:"
        )

        receivers = (
            self.discover_receivers()
        )

        if not receivers:

            print(
                "RESULT: NO_RX"
            )

            self.disconnect()

            return False

        # ----------------------------------------------------
        # Select first RX
        # ----------------------------------------------------

        receiver_id = (
            next(
                iter(receivers)
            )
        )

        print()

        print(
            f"Selecting RX: "
            f"{receiver_id}"
        )

        if not self.select_target(
            receiver_id
        ):

            print(
                "RESULT: TARGET_FAILED"
            )

            self.disconnect()

            return False

        # ----------------------------------------------------
        # Control
        # ----------------------------------------------------

        print()

        print(
            "Sending control..."
        )

        if not self.send_control(
            1000,
            2000
        ):

            print(
                "RESULT: CONTROL_FAILED"
            )

            self.disconnect()

            return False

        time.sleep(
            0.2
        )

        # ----------------------------------------------------
        # Drain Nano responses
        # ----------------------------------------------------

        deadline = (
            time.time() + 1.0
        )

        while time.time() < deadline:

            line = self.read_line()

            if line:

                print(
                    " ",
                    line
                )

            else:

                time.sleep(
                    0.01
                )

        print()

        print(
            "RESULT: TEST_COMPLETE"
        )

        self.disconnect()

        return True


# ============================================================
# STANDALONE TEST
# ============================================================

if __name__ == "__main__":

    controller = RadioController(
        "/dev/ttyUSB0"
    )

    controller.test()
