import hid
import time

# CP2110 USB HID identity
VID = 0x10C4
PID = 0x86B9

# Validated CP2110 UART configuration: 38400 8N1
UART_CONFIG = [
    0x50,
    0x00, 0x00, 0x96, 0x00,
    0x00,
    0x00,
    0x03,
    0x00
]

UART_ENABLE = [
    0x41,
    0x01
]


class LapTimer:
    """
    Standalone DriveMatrix CP2110 lap-timer reader.

    This module only performs:
        CP2110 HID -> UART bytes -> 14-byte packet -> detection callback

    Lap/session/leaderboard logic is intentionally handled by the caller.
    """

    def __init__(self, on_detection=None):
        self.on_detection = on_detection
        self.dev = None
        self.stream = bytearray()
        self.running = False

    def open(self):
        print("Opening CP2110...")

        self.dev = hid.device()
        self.dev.open(VID, PID)
        self.dev.set_nonblocking(True)

        print("CP2110 connected!")

        self.dev.send_feature_report(UART_CONFIG)
        print("UART configured: 38400 8N1")

        self.dev.send_feature_report(UART_ENABLE)
        print("UART enabled")
        print()
        print("Waiting for transponder...")
        print("----------------------------------------")

    def _process_stream(self):
        while True:
            if len(self.stream) < 3:
                return

            # Valid packet header
            if self.stream[0] == 0x0D and self.stream[2] == 0x84:

                if len(self.stream) < 14:
                    return

                packet = bytes(self.stream[:14])

                transponder_id = (
                    packet[3]
                    | (packet[4] << 8)
                )

                timer = (
                    packet[7]
                    | (packet[8] << 8)
                    | (packet[9] << 16)
                    | (packet[10] << 24)
                )

                if self.on_detection is not None:
                    try:
                        self.on_detection(
                            transponder_id,
                            timer
                        )
                    except Exception as exc:
                        print(
                            f"[LapTimer] Detection callback error: {exc}"
                        )
                else:
                    print(
                        f"TRANSPONDER={transponder_id} "
                        f"TIMER={timer}"
                    )

                del self.stream[:14]
                continue

            # Discard one byte and continue searching for a valid packet.
            del self.stream[0]

    def run(self):
        if self.dev is None:
            self.open()

        self.running = True

        try:
            while self.running:
                raw = self.dev.read(64)

                if raw:
                    count = raw[0]

                    if 1 <= count <= 63:
                        uart_data = raw[1:1 + count]
                        self.stream.extend(uart_data)
                        self._process_stream()

                time.sleep(0.001)

        except KeyboardInterrupt:
            print()
            print("Stopping...")

        finally:
            self.close()

    def stop(self):
        self.running = False

    def close(self):
        self.running = False

        if self.dev is not None:
            try:
                self.dev.close()
            except Exception:
                pass

            self.dev = None

        print("CP2110 closed.")


def print_detection(transponder_id, timer):
    print(
        f"TRANSPONDER={transponder_id} "
        f"TIMER={timer}"
    )


if __name__ == "__main__":
    lap_timer = LapTimer(on_detection=print_detection)
    lap_timer.run()
