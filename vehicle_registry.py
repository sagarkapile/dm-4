import json
import os
import threading


REGISTRY_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "vehicles.json"
)


class VehicleRegistry:
    """
    Persistent mapping between permanent RX IDs and
    DriveMatrix vehicle names.

    Example:

        9C52F7020F3C -> Car 1
        AABBCCDDEEFF -> Car 2

    RX ID is permanent.
    Vehicle name is assigned by the Pi.
    """

    def __init__(self, filename=REGISTRY_FILE):

        self.filename = filename
        self.lock = threading.Lock()

        self.vehicles = {}

        self.load()

    # ========================================================
    # LOAD
    # ========================================================

    def load(self):

        with self.lock:

            if not os.path.exists(self.filename):

                self.vehicles = {}

                return

            try:

                with open(
                    self.filename,
                    "r",
                    encoding="utf-8"
                ) as file:

                    data = json.load(file)

                if not isinstance(data, dict):

                    raise ValueError(
                        "Vehicle registry must be a JSON object"
                    )

                # Add new vehicle fields to older registry records
                # without changing any existing vehicle settings.
                for receiver_id, vehicle in data.items():
                    if not isinstance(vehicle, dict):
                        continue

                    vehicle.setdefault("steering_reverse", False)
                    vehicle.setdefault("transponder_id", None)
                    vehicle.setdefault("nano_id", None)

                self.vehicles = data

            except Exception as e:

                print(
                    f"[VehicleRegistry] "
                    f"Failed to load registry: {e}"
                )

                self.vehicles = {}

    # ========================================================
    # SAVE
    # ========================================================

    def save(self):

        with self.lock:

            directory = os.path.dirname(
                self.filename
            )

            if directory:
                os.makedirs(
                    directory,
                    exist_ok=True
                )

            temp_file = (
                self.filename +
                ".tmp"
            )

            with open(
                temp_file,
                "w",
                encoding="utf-8"
            ) as file:

                json.dump(
                    self.vehicles,
                    file,
                    indent=4,
                    sort_keys=True
                )

                file.write("\n")

            os.replace(
                temp_file,
                self.filename
            )

    # ========================================================
    # GET VEHICLE
    # ========================================================

    def get_vehicle(self, receiver_id):

        with self.lock:

            return self.vehicles.get(
                receiver_id
            )

    # ========================================================
    # HAS RECEIVER
    # ========================================================

    def has_receiver(self, receiver_id):

        with self.lock:

            return receiver_id in self.vehicles

    # ========================================================
    # ASSIGN NEXT NAME
    # ========================================================

    def assign_next(self, receiver_id):

        receiver_id = receiver_id.strip()

        if not receiver_id:

            raise ValueError(
                "receiver_id cannot be empty"
            )

        with self.lock:

            # Already assigned.
            if receiver_id in self.vehicles:

                return self.vehicles[
                    receiver_id
                ]

            used_numbers = []

            for vehicle in self.vehicles.values():

                name = str(vehicle.get("name", ""))

                if name.startswith("Car "):

                    number_text = name[4:].strip()

                    try:

                        number = int(
                            number_text
                        )

                        if number > 0:
                            used_numbers.append(
                                number
                            )

                    except ValueError:
                        pass

            next_number = 1

            while next_number in used_numbers:

                next_number += 1

            vehicle_name = (
                f"Car {next_number}"
            )

            self.vehicles[
                receiver_id
            ] = {
                "name": vehicle_name,
                "steering_reverse": False,
                "transponder_id": None
            }

        self.save()

        print(
            f"[VehicleRegistry] Assigned "
            f"{receiver_id} -> {vehicle_name}"
        )

        return self.vehicles[
            receiver_id
        ]

    # ========================================================
    # SET NAME
    # ========================================================

    def set_name(
        self,
        receiver_id,
        vehicle_name
    ):

        receiver_id = receiver_id.strip()
        vehicle_name = vehicle_name.strip()

        if not receiver_id:

            raise ValueError(
                "receiver_id cannot be empty"
            )

        if not vehicle_name:

            raise ValueError(
                "vehicle_name cannot be empty"
            )

        with self.lock:

            vehicle = self.vehicles.get(receiver_id)

            if vehicle is None:
                vehicle = {
                    "name": vehicle_name,
                    "steering_reverse": False,
                    "transponder_id": None
                }
            else:
                vehicle["name"] = vehicle_name
                vehicle.setdefault("steering_reverse", False)
                vehicle.setdefault("transponder_id", None)

            self.vehicles[receiver_id] = vehicle

        self.save()

    # ========================================================
    # CREATE VEHICLE (full CRUD, explicit fields)
    # ========================================================

    def create_vehicle(
        self,
        receiver_id,
        name,
        nano_id=None,
        transponder_id=None,
        steering_reverse=False
    ):
        """Create a new vehicle record with admin-provided fields.

        Unlike assign_next(), this does not auto-generate a name; the
        admin supplies the ESP MAC (receiver_id), vehicle name, paired
        Nano ID and transponder ID directly.
        """

        receiver_id = str(receiver_id).strip().upper()
        name = str(name).strip()

        if not receiver_id:
            raise ValueError("receiver_id cannot be empty")

        if not name:
            raise ValueError("name cannot be empty")

        nano_id = str(nano_id).strip() if nano_id else None

        if transponder_id is None or str(transponder_id).strip() == "":
            transponder_id = None
        else:
            try:
                transponder_id = int(transponder_id)
            except (TypeError, ValueError):
                raise ValueError("transponder_id must be an integer")

            if transponder_id < 0:
                raise ValueError("transponder_id cannot be negative")

        with self.lock:
            if receiver_id in self.vehicles:
                raise ValueError(
                    f"Vehicle already exists for {receiver_id}"
                )

            self.vehicles[receiver_id] = {
                "name": name,
                "steering_reverse": bool(steering_reverse),
                "transponder_id": transponder_id,
                "nano_id": nano_id
            }

        self.save()

        print(f"[VehicleRegistry] Created {receiver_id} -> {name}")

        return self.vehicles[receiver_id]

    # ========================================================
    # SET NANO ID (fixed cockpit -> Nano pairing)
    # ========================================================

    def set_nano_id(self, receiver_id, nano_id):
        receiver_id = receiver_id.strip().upper()

        if not receiver_id:
            raise ValueError("receiver_id cannot be empty")

        nano_id = str(nano_id).strip() if nano_id else None

        with self.lock:
            vehicle = self.vehicles.get(receiver_id)

            if vehicle is None:
                raise ValueError(
                    f"Unknown receiver_id: {receiver_id}"
                )

            vehicle["nano_id"] = nano_id

        self.save()

    # ========================================================
    # UPSERT FROM NANO PAIRING
    # ========================================================

    def upsert_from_nano_pairing(
        self,
        receiver_id,
        vehicle_name,
        nano_id
    ):
        """Create or update a vehicle from Nano PAIRING data.

        Rules:
          - A Nano ID already paired to an existing vehicle (any RX id) is
            left completely untouched: no new entry, no re-pairing. This
            keeps every already-known Nano's car assignment stable across
            restarts/refreshes, even if a PAIRING read momentarily reports
            a different RX id.
          - Only a genuinely new (never-seen) Nano ID may create a vehicle,
            or attach to an existing RX entry that has no nano_id yet.
          - Never overwrite an existing non-empty vehicle name with blank.
        """
        receiver_id = str(receiver_id or "").strip().upper()
        vehicle_name = str(vehicle_name or "").strip()
        nano_id = str(nano_id or "").strip() or None

        if not receiver_id:
            raise ValueError("receiver_id cannot be empty")

        if len(receiver_id) != 12:
            raise ValueError(
                f"receiver_id must be 12 hex chars: {receiver_id}"
            )

        if not nano_id:
            raise ValueError("nano_id cannot be empty")

        action = "unchanged"

        with self.lock:
            # If this Nano is already paired to a different RX entry, do not
            # move the pairing or create a duplicate car; leave it as-is.
            for existing_id, existing in self.vehicles.items():
                if not isinstance(existing, dict):
                    continue
                if existing_id == receiver_id:
                    continue
                if str(existing.get("nano_id") or "").strip() == nano_id:
                    print(
                        f"[VehicleRegistry] Nano {nano_id} already paired "
                        f"to {existing_id}; ignoring PAIRING report of "
                        f"{receiver_id}"
                    )
                    return "unchanged", dict(existing)

            vehicle = self.vehicles.get(receiver_id)

            if vehicle is None:
                if not vehicle_name:
                    vehicle_name = f"Car {receiver_id[-4:]}"

                self.vehicles[receiver_id] = {
                    "name": vehicle_name,
                    "steering_reverse": False,
                    "transponder_id": None,
                    "nano_id": nano_id
                }
                action = "created"
            else:
                changed = False
                existing_nano = str(vehicle.get("nano_id") or "").strip() or None
                existing_name = str(vehicle.get("name") or "").strip()

                # A known Nano's own pairing (receiver_id already matches)
                # only needs its nano_id filled in if it was ever empty.
                if not existing_nano:
                    vehicle["nano_id"] = nano_id
                    changed = True

                # Fill missing name from Nano VEHICLE report; do not clobber
                # an operator-edited non-empty name.
                if vehicle_name and not existing_name:
                    vehicle["name"] = vehicle_name
                    changed = True

                vehicle.setdefault("steering_reverse", False)
                vehicle.setdefault("transponder_id", None)

                if changed:
                    action = "updated"

            result = dict(self.vehicles[receiver_id])

        if action != "unchanged":
            self.save()
            print(
                f"[VehicleRegistry] PAIRING {action}: "
                f"{receiver_id} -> {result.get('name')} "
                f"(nano={nano_id})"
            )

        return action, result

    # ========================================================
    # FIND BY NAME
    # ========================================================

    def find_by_name(self, name):
        """Return (receiver_id, vehicle) for the given vehicle name."""
        name = str(name).strip()

        with self.lock:
            for receiver_id, vehicle in self.vehicles.items():
                if not isinstance(vehicle, dict):
                    continue
                if str(vehicle.get("name", "")).strip() == name:
                    return receiver_id, dict(vehicle)

        return None, None

    # ========================================================
    # SET STEERING REVERSE
    # ========================================================

    def set_steering_reverse(self, receiver_id, reverse):
        receiver_id = receiver_id.strip()

        if not receiver_id:
            raise ValueError(
                "receiver_id cannot be empty"
            )

        with self.lock:
            vehicle = self.vehicles.get(receiver_id)

            if vehicle is None:
                raise ValueError(
                    f"Unknown receiver_id: {receiver_id}"
                )

            vehicle["steering_reverse"] = bool(reverse)

        self.save()


    # ========================================================
    # SET TRANSPONDER ID
    # ========================================================

    def set_transponder_id(self, receiver_id, transponder_id):
        receiver_id = receiver_id.strip().upper()

        if not receiver_id:
            raise ValueError(
                "receiver_id cannot be empty"
            )

        if transponder_id is None or str(transponder_id).strip() == "":
            transponder_id = None
        else:
            try:
                transponder_id = int(transponder_id)
            except (TypeError, ValueError):
                raise ValueError(
                    "transponder_id must be an integer"
                )

            if transponder_id < 0:
                raise ValueError(
                    "transponder_id cannot be negative"
                )

        with self.lock:
            vehicle = self.vehicles.get(receiver_id)

            if vehicle is None:
                raise ValueError(
                    f"Unknown receiver_id: {receiver_id}"
                )

            vehicle["transponder_id"] = transponder_id

        self.save()


    # ========================================================
    # REMOVE
    # ========================================================

    def remove(self, receiver_id):

        with self.lock:

            if receiver_id not in self.vehicles:

                return False

            del self.vehicles[
                receiver_id
            ]

        self.save()

        return True

    # ========================================================
    # ALL VEHICLES
    # ========================================================

    def all(self):

        with self.lock:

            return dict(
                self.vehicles
            )

    # ========================================================
    # PRINT
    # ========================================================

    def print_all(self):

        print()
        print(
            "=============================="
        )

        print(
            "DriveMatrix Vehicle Registry"
        )

        print(
            "=============================="
        )

        if not self.vehicles:

            print(
                "No vehicles registered."
            )

            return

        print(
            f"Vehicles: {len(self.vehicles)}"
        )

        print()

        for receiver_id in sorted(
            self.vehicles
        ):

            vehicle = self.vehicles[
                receiver_id
            ]

            print(
                f"RX ID:  {receiver_id}"
            )

            print(
                f"  Name: "
                f"{vehicle.get('name', '')}"
            )

            print(
                f"  Transponder ID: "
                f"{vehicle.get('transponder_id')}"
            )

            print(
                f"  Steering Reverse: "
                f"{vehicle.get('steering_reverse', False)}"
            )

            print()


# ============================================================
# TEST
# ============================================================

if __name__ == "__main__":

    registry = VehicleRegistry()

    print(
        "Registry file:"
    )

    print(
        registry.filename
    )

    print()

    print(
        "Assigning test receiver..."
    )

    vehicle = registry.assign_next(
        "9C52F7020F3C"
    )

    print(
        f"Assigned name: "
        f"{vehicle['name']}"
    )

    registry.print_all()
