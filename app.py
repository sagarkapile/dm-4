import time
import select
import evdev
import glob
import sqlite3
import os
import minimal_drive
from g29_ffb import G29FFB
import threading
from flask import Flask, render_template, request, jsonify
from cockpit_manager import CockpitManager
from radio_discovery import discover_radios
from telemetry_receiver import TelemetryReceiver
from lap_timer import LapTimer
from radio_manager_wifi_sync import get_active_wifi_credentials
import radio_manager_wifi_sync
import subprocess
import sys
import json
import re

app = Flask(__name__)

session_active = True
autocenter_enabled = True



# Legacy/global vehicle control settings retained for the existing drive loop.
# Per-cockpit settings are stored separately below and are exposed through
# the cockpit API. The actual multi-cockpit drive loop will use these later
# when wheel->radio control is integrated.
steering_sensitivity = 100
throttle_limit = 100

# Live Haptic Engine Tuning Parameters
haptic_settings = {
    "grip_threshold": 0.65,
    "bump_sensitivity": 2.0,
    "centering_gain": 25.0,
    "alpha": 0.20
}

start_time = time.time()
# Per-cockpit G29 FFB instances. Each instance is bound to the same
# wheel device owned by that cockpit's control worker.
cockpit_ffb_instances = {}
cockpit_ffb_lock = threading.Lock()

esp_ffb_instances = {}
esp_ffb_lock = threading.Lock()

# Telemetry receiver: RX identity + impact detection + per-cockpit FFB routing.

# Legacy/global reference retained only for compatibility with the existing
# single-cockpit API. It is never bound to a hardcoded /dev/input/eventX.
g29_ffb_instance = None

# Logical cockpit / vehicle assignment manager.
cockpit_manager = CockpitManager()
cockpit_system_initialized = False
cockpit_init_lock = threading.Lock()
cockpit_wifi_sync_lock = threading.Lock()

# ============================================================
# LAP TIMER
# ============================================================

lap_timer = None
lap_timer_thread = None

MIN_LAP_TIME_MS = 5000

lap_timer_lock = threading.Lock()

lap_timer_vehicle_state = {}

leaderboard_driver_state = {}

lap_timer_running = False

# Latest detection information.
lap_timer_last_detection = {
    "transponder_id": None,
    "timer": None,
    "timestamp": None
}

# Per-transponder race/lap state.
lap_timer_vehicle_state = {}

# ============================================================
# USER MANAGEMENT
# ============================================================

USER_DATABASE_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "database",
    "drivematrix.db"
)

# Persistent cockpit -> preferred Nano selection.
# This is intentionally separate from cockpit.radio_id because the latter
# is the currently active Nano assignment and is cleared when a session
# ends. The preferred vehicle survives sessions, refreshes and Pi reboots.
# The paired Nano is resolved from the vehicle registry every time (not
# fixed at selection time), so changing a car's Nano pairing in Vehicle
# Management is picked up automatically on the next restore.
PREFERRED_CAR_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "cockpit_vehicle_assignments.json"
)
preferred_car_lock = threading.Lock()
preferred_cars = {}


def handle_lap_timer_detection(transponder_id, timer):
    """
    Called by LapTimer whenever a transponder is detected.

    lap_timer.py only reports:
        transponder_id
        timer

    All DriveMatrix-specific interpretation happens here.
    """

    now = time.time()

    transponder_id = int(transponder_id)
    timer = int(timer)

    with lap_timer_lock:
        lap_timer_last_detection["transponder_id"] = transponder_id
        lap_timer_last_detection["timer"] = timer
        lap_timer_last_detection["timestamp"] = now

    print(
        f"[Lap Timer] Detection: "
        f"transponder={transponder_id} "
        f"timer={timer}"
    )

    vehicle = find_vehicle_by_transponder_id(transponder_id)

    if vehicle is not None:
        print(
            f"[Lap Timer] Vehicle matched: "
            f"{vehicle['name']} "
            f"RX={vehicle['receiver_id']}"
        )

        cockpit_id = find_active_cockpit_for_vehicle(vehicle)

        if cockpit_id is not None:
            print(
                f"[Lap Timer] Active cockpit: "
                f"{cockpit_id}"
            )

            process_lap_detection(
                vehicle,
                cockpit_id,
                timer
            )
        else:
            print(
                f"[Lap Timer] Vehicle is not in "
                f"an active cockpit session"
            )

    else:
        print(
            f"[Lap Timer] No vehicle mapped to "
            f"transponder={transponder_id}"
        )

def lap_timer_worker():
    global lap_timer, lap_timer_running

    print("[Lap Timer] Worker starting")

    try:
        lap_timer = LapTimer(
            on_detection=handle_lap_timer_detection
        )

        with lap_timer_lock:
            lap_timer_running = True

        print("[Lap Timer] CP2110 starting")

        lap_timer.run()

    except Exception as exc:
        print(f"[Lap Timer] Worker error: {exc}")

    finally:
        with lap_timer_lock:
            lap_timer_running = False

        print("[Lap Timer] Worker stopped")

def start_lap_timer():
    global lap_timer_thread

    with lap_timer_lock:
        if (
            lap_timer_thread is not None
            and lap_timer_thread.is_alive()
        ):
            print("[Lap Timer] Already running")
            return

        lap_timer_thread = threading.Thread(
            target=lap_timer_worker,
            name="lap-timer",
            daemon=True
        )

        lap_timer_thread.start()

    print("[Lap Timer] Background worker started")

def stop_lap_timer():
    global lap_timer_thread

    with lap_timer_lock:
        timer = lap_timer
        thread = lap_timer_thread

    if timer is not None:
        try:
            timer.stop()
        except Exception as exc:
            print(f"[Lap Timer] Stop error: {exc}")

    if thread is not None:
        thread.join(timeout=3.0)

    with lap_timer_lock:
        lap_timer_thread = None

    print("[Lap Timer] Background worker stopped")

def _load_preferred_cars():
    """Load persistent cockpit car preferences from disk."""
    global preferred_cars

    try:
        with open(PREFERRED_CAR_FILE, "r", encoding="utf-8") as handle:
            data = json.load(handle)

        if not isinstance(data, dict):
            raise ValueError("preferred car data is not an object")

        loaded = {}
        for cockpit_id, vehicle_name in data.items():
            try:
                cockpit_id = int(cockpit_id)
            except (TypeError, ValueError):
                continue

            if not 1 <= cockpit_id <= cockpit_manager.max_cockpits:
                continue

            vehicle_name = str(vehicle_name).strip()
            if vehicle_name:
                loaded[cockpit_id] = vehicle_name

        with preferred_car_lock:
            preferred_cars = loaded

        print(f"[Car Preference] Loaded: {preferred_cars}")
    except FileNotFoundError:
        with preferred_car_lock:
            preferred_cars = {}
        print("[Car Preference] No saved car preferences yet")
    except Exception as exc:
        with preferred_car_lock:
            preferred_cars = {}
        print(f"[Car Preference] Load error: {exc}")


def _save_preferred_cars():
    """Atomically save cockpit car preferences to disk."""
    try:
        with preferred_car_lock:
            data = {str(k): v for k, v in preferred_cars.items() if v}

        temp_path = PREFERRED_CAR_FILE + ".tmp"
        with open(temp_path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temp_path, PREFERRED_CAR_FILE)
    except Exception as exc:
        print(f"[Car Preference] Save error: {exc}")


def _set_preferred_car(cockpit_id, vehicle_name):
    """Persist the car explicitly selected by the operator."""
    with preferred_car_lock:
        if vehicle_name is None:
            preferred_cars.pop(cockpit_id, None)
        else:
            preferred_cars[cockpit_id] = str(vehicle_name).strip()
    _save_preferred_cars()


def _get_preferred_car(cockpit_id):
    with preferred_car_lock:
        return preferred_cars.get(cockpit_id)


def restore_preferred_car_assignments():
    """Restore remembered cars from Vehicle Management pairing.

    Nano serial presence is not required. A remembered car is only skipped
    when its Nano ID is already claimed by another cockpit.
    """
    for cockpit_id in range(1, cockpit_manager.max_cockpits + 1):
        cockpit = cockpit_manager.get_cockpit(cockpit_id)
        if cockpit is None or cockpit.radio_id is not None:
            continue

        preferred = _get_preferred_car(cockpit_id)
        if not preferred:
            continue

        try:
            if cockpit_manager.select_vehicle_by_registry(cockpit_id, preferred):
                print(
                    f"[Car Preference] Restored Cockpit {cockpit_id} "
                    f"-> {preferred}"
                )
        except Exception as exc:
            print(
                f"[Car Preference] Cockpit {cockpit_id}: "
                f"preferred car {preferred} not restored ({exc})"
            )



# Wi-Fi provisioning:
# Pi -> Nano USB serial -> nRF24 -> paired RX.
# This is deliberately kept separate from the RF control loop.
WIFI_FALLBACK_SSID = "DriveMatrix-AP"
WIFI_FALLBACK_PASSWORD = "drivematrix"


_load_preferred_cars()

def provision_wifi_to_radio(radio_id):
    """Provision the Pi Wi-Fi credentials using the live RadioManager."""
    try:
        _, ssid, password = get_active_wifi_credentials()

        if not ssid:
            print("[WiFi Sync] Active Wi-Fi SSID is empty")
            return False

        if not password:
            print("[WiFi Sync] Active Wi-Fi password is empty")
            return False

        print(
            f"[WiFi Sync] Requesting RadioManager provisioning for "
            f"radio {radio_id}"
        )

        return cockpit_manager.radio_manager.provision_wifi_credentials(
            radio_id=radio_id,
            ssid=ssid,
            password=password,
            fallback_ssid=WIFI_FALLBACK_SSID,
            fallback_password=WIFI_FALLBACK_PASSWORD,
        )

    except Exception as exc:
        print(f"[WiFi Sync] Provisioning error: {exc}")
        return False

# Per-cockpit session/settings state.
#
# Session duration is configured by the user. The timer itself is owned by
# the Raspberry Pi, not by the browser, so a page refresh/disconnect cannot
# extend or cancel a running session.
cockpit_settings_lock = threading.Lock()
cockpit_settings = {
    cockpit_id: {
        "session_duration_minutes": 10,
        "steering_sensitivity": 100,
        "throttle_sensitivity": 100,
        "autocenter_enabled": True
    }
    for cockpit_id in range(1, cockpit_manager.max_cockpits + 1)
}

COCKPIT_SETTINGS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "cockpit_settings.json"
)

ESP_COCKPIT_SETTINGS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "esp_cockpit_settings.json"
)

cockpit_sessions_lock = threading.Lock()
cockpit_sessions = {
    cockpit_id: {
        "active": False,
        "started_at": None,
        "ends_at": None,
    }
    for cockpit_id in range(1, cockpit_manager.max_cockpits + 1)
}


# Per-cockpit driver assignment.
# A driver becomes active only when that cockpit session starts.
cockpit_driver_lock = threading.Lock()

cockpit_drivers = {
    cockpit_id: {
        "user_id": None,
        "name": None,
        "phone": None,
    }
    for cockpit_id in range(1, cockpit_manager.max_cockpits + 1)
}

ESP_COCKPIT_COUNT = 4

ESP_VEHICLE_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "esp_vehicles.json"
)

esp_vehicle_lock = threading.Lock()

esp_vehicles = {
    1: {
        "name": "Car 1",
        "transponder_id": None
    },
    2: {
        "name": "Car 2",
        "transponder_id": None
    },
    3: {
        "name": "Car 3",
        "transponder_id": None
    }
}

def load_esp_vehicles():
    global esp_vehicles

    if not os.path.exists(ESP_VEHICLE_FILE):
        return

    try:
        with open(
            ESP_VEHICLE_FILE,
            "r",
            encoding="utf-8"
        ) as file:
            data = json.load(file)

        if not isinstance(data, dict):
            return

        with esp_vehicle_lock:
            for cockpit_id in range(1, 4):
                item = data.get(str(cockpit_id))

                if not isinstance(item, dict):
                    continue

                name = str(
                    item.get(
                        "name",
                        f"Car {cockpit_id}"
                    )
                ).strip()

                if not name:
                    name = f"Car {cockpit_id}"

                transponder_id = item.get("transponder_id")

                if transponder_id is not None:
                    try:
                        transponder_id = int(transponder_id)
                    except (TypeError, ValueError):
                        transponder_id = None

                esp_vehicles[cockpit_id] = {
                    "name": name,
                    "transponder_id": transponder_id
                }

    except Exception as exc:
        print(f"[ESP Vehicle] Load error: {exc}")


def save_esp_vehicles():
    with esp_vehicle_lock:
        data = {
            str(cockpit_id): dict(vehicle)
            for cockpit_id, vehicle in esp_vehicles.items()
        }

    temp_path = ESP_VEHICLE_FILE + ".tmp"

    with open(
        temp_path,
        "w",
        encoding="utf-8"
    ) as file:
        json.dump(data, file, indent=2)
        file.write("\n")

    os.replace(temp_path, ESP_VEHICLE_FILE)


def get_esp_vehicle(cockpit_id):
    with esp_vehicle_lock:
        vehicle = esp_vehicles.get(cockpit_id)

        if vehicle is None:
            return None

        return dict(vehicle)


load_esp_vehicles()

esp_cockpit_driver_lock = threading.Lock()
esp_cockpit_drivers = {
    cockpit_id: {
        "user_id": None,
        "name": None,
        "phone": None,
    }
    for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1)
}

# ESP cockpit -> ESP vehicle assignment.
# Default mapping keeps the original configuration:
# ESP Cockpit 1 -> Car 1
# ESP Cockpit 2 -> Car 2
esp_cockpit_vehicle_lock = threading.Lock()

esp_cockpit_vehicles = {
    cockpit_id: cockpit_id
    for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1)
}

esp_cockpit_settings_lock = threading.Lock()
esp_cockpit_settings = {
    cockpit_id: {
        "session_duration_minutes": 10,
        "steering_sensitivity": 100,
        "throttle_sensitivity": 100,
        "autocenter_enabled": True
    }
    for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1)
}

esp_cockpit_sessions_lock = threading.Lock()
esp_cockpit_sessions = {
    cockpit_id: {
        "active": False,
        "started_at": None,
        "ends_at": None,
    }
    for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1)
}

esp_cockpits_lock = threading.Lock()
esp_cockpits = {
    cockpit_id: {
        "wheel": None,
        "esp": None,
    }
    for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1)
}

# FFB impact configuration. These values are intentionally identical to the
# standalone, already-validated ffb_test.py implementation.
FFB_MIN = 30.0
FFB_MAX = 100.0
FFB_MULTIPLIER = 50.0
FFB_KICK_DURATION = 0.25


def _load_esp_cockpit_settings():
    """Load persistent per-ESP-cockpit settings from disk."""
    try:
        if not os.path.exists(ESP_COCKPIT_SETTINGS_FILE):
            print("[ESP Settings] No saved settings found; using defaults")
            return

        with open(ESP_COCKPIT_SETTINGS_FILE, "r", encoding="utf-8") as handle:
            saved = json.load(handle)

        if not isinstance(saved, dict):
            print("[ESP Settings] Invalid settings file; using defaults")
            return

        with esp_cockpit_settings_lock:
            for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1):
                saved_settings = saved.get(str(cockpit_id))

                if not isinstance(saved_settings, dict):
                    continue

                current = esp_cockpit_settings[cockpit_id]

                try:
                    value = int(
                        saved_settings.get(
                            "session_duration_minutes",
                            current["session_duration_minutes"]
                        )
                    )
                    if 1 <= value <= 120:
                        current["session_duration_minutes"] = value
                except (TypeError, ValueError):
                    pass

                try:
                    value = int(
                        saved_settings.get(
                            "steering_sensitivity",
                            current["steering_sensitivity"]
                        )
                    )
                    if 10 <= value <= 200:
                        current["steering_sensitivity"] = value
                except (TypeError, ValueError):
                    pass

                try:
                    value = int(
                        saved_settings.get(
                            "throttle_sensitivity",
                            current["throttle_sensitivity"]
                        )
                    )
                    if 10 <= value <= 100:
                        current["throttle_sensitivity"] = value
                except (TypeError, ValueError):
                    pass

                try:
                    val = saved_settings.get(
                        "autocenter_enabled",
                        current["autocenter_enabled"]
                    )
                    if isinstance(val, bool):
                        current["autocenter_enabled"] = val
                    elif isinstance(val, str):
                        current["autocenter_enabled"] = val.lower() == "true"
                except Exception:
                    pass

        print("[ESP Settings] Persistent settings loaded")

    except Exception as exc:
        print(f"[ESP Settings] Load error: {exc}")


def _save_esp_cockpit_settings():
    try:
        with esp_cockpit_settings_lock:
            data = {
                str(cockpit_id): dict(settings)
                for cockpit_id, settings in esp_cockpit_settings.items()
            }

        temp_path = ESP_COCKPIT_SETTINGS_FILE + ".tmp"

        with open(temp_path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
            handle.write("\n")

        os.replace(temp_path, ESP_COCKPIT_SETTINGS_FILE)

        print("[ESP Settings] Saved successfully")

    except Exception as exc:
        print(f"[ESP Settings] Save error: {exc}")

_load_esp_cockpit_settings()

def stop_esp_cockpit_session(cockpit_id):
    with esp_cockpit_sessions_lock:
        esp_cockpit_sessions[cockpit_id] = {
            "active": False,
            "started_at": None,
            "ends_at": None,
        }

    # Clear lap timing state for the ESP vehicle.
    receiver_id = f"ESP{cockpit_id}"

    lap_timer_vehicle_state.pop(receiver_id, None)

    with esp_cockpit_driver_lock:
        esp_cockpit_drivers[cockpit_id] = {
            "user_id": None,
            "name": None,
            "phone": None,
        }

    print(f"[ESP{cockpit_id}] SESSION_STOPPED")

    with esp_cockpits_lock:
        esp = esp_cockpits[cockpit_id]["esp"]

    if esp is not None:
        try:
            esp.send_control(
                _esp_throttle_to_dac(0),
                _esp_steering_to_dac(0)
            )
        except Exception:
            pass

    return True, None


def esp_cockpit_session_worker():
    """Background timer for ESP cockpit sessions."""

    while True:
        now = time.time()
        expired = []

        with esp_cockpit_sessions_lock:
            for cockpit_id, session in esp_cockpit_sessions.items():
                if (
                    session["active"]
                    and session["ends_at"] is not None
                    and now >= session["ends_at"]
                ):
                    expired.append(cockpit_id)

        for cockpit_id in expired:
            print(
                f"[ESP Session] Cockpit {cockpit_id} expired"
            )
            stop_esp_cockpit_session(cockpit_id)

        time.sleep(0.25)

threading.Thread(
    target=esp_cockpit_session_worker,
    daemon=True
).start()

def get_esp_cockpit_session_state(cockpit_id):
    now = time.time()

    with esp_cockpit_sessions_lock:
        session = esp_cockpit_sessions[cockpit_id]

        if session["active"] and session["ends_at"] is not None:
            remaining = max(
                0,
                int(session["ends_at"] - now)
            )
        else:
            remaining = 0

        return {
            "active": bool(session["active"]),
            "remaining_seconds": remaining
        }

def start_esp_cockpit_session(cockpit_id):
    """Start a timed session for one ESP cockpit."""

    if cockpit_id not in esp_cockpits:
        return False, "Invalid ESP cockpit ID"

    with esp_cockpits_lock:
        cockpit = esp_cockpits[cockpit_id]
        wheel = cockpit["wheel"]
        esp = cockpit["esp"]

    if wheel is None:
        return False, f"ESP Cockpit {cockpit_id} has no wheel connected"

    if esp is None:
        return False, f"ESP Cockpit {cockpit_id} has no ESP32 connected"

    if esp.serial is None or not esp.serial.is_open:
        return False, f"ESP Cockpit {cockpit_id} ESP32 serial is not open"

    with esp_cockpit_driver_lock:
        driver = dict(esp_cockpit_drivers[cockpit_id])

    if not driver["user_id"]:
        return False, f"ESP Cockpit {cockpit_id} has no driver selected"

    user = get_user_by_id(driver["user_id"])

    if user is None:
        with esp_cockpit_driver_lock:
            esp_cockpit_drivers[cockpit_id] = {
                "user_id": None,
                "name": None,
                "phone": None,
            }

        return False, f"ESP Cockpit {cockpit_id} driver no longer exists"

    with esp_cockpit_sessions_lock:
        if esp_cockpit_sessions[cockpit_id]["active"]:
            return False, f"ESP Cockpit {cockpit_id} session is already active"

    with esp_cockpit_settings_lock:
        duration_minutes = esp_cockpit_settings[cockpit_id][
            "session_duration_minutes"
        ]

    now = time.time()
    duration_seconds = int(duration_minutes * 60)
    ends_at = now + duration_seconds

    # Send neutral control before allowing the session to run.
    try:
        esp.send_control(124, 128)
    except Exception as e:
        return False, f"ESP32 neutral command failed: {e}"

    with esp_cockpit_sessions_lock:
        esp_cockpit_sessions[cockpit_id] = {
            "active": True,
            "started_at": now,
            "ends_at": ends_at,
        }

    # The control worker only reads the wheel that was assigned at the time
    # it started. A prior unplug/replug or a boot without a wheel present
    # leaves no worker running, so make sure one is alive before driving.
    start_esp_cockpit_control_worker(cockpit_id)

    update_user_last_used(user["user_id"])

    print(
        f"[ESP Session] Cockpit {cockpit_id} started: "
        f"{duration_minutes} minute(s), Car {cockpit_id}"
    )

    return True, None

def stop_esp_cockpit_session(cockpit_id):
    """Stop an ESP session and force neutral output."""

    if cockpit_id not in esp_cockpits:
        return False, "Invalid ESP cockpit ID"

    with esp_cockpits_lock:
        esp = esp_cockpits[cockpit_id]["esp"]

    # Always force neutral when stopping.
    if esp is not None:
        try:
            esp.send_control(124, 128)
        except Exception as e:
            print(
                f"[ESP Session] Cockpit {cockpit_id} neutral warning: {e}"
            )

    with esp_cockpit_sessions_lock:
        esp_cockpit_sessions[cockpit_id] = {
            "active": False,
            "started_at": None,
            "ends_at": None,
        }

    # Clear lap timing state for the ESP vehicle.
    receiver_id = f"ESP{cockpit_id}"
    lap_timer_vehicle_state.pop(receiver_id, None)

    print(f"[ESP Session] Cockpit {cockpit_id} stopped")

    print(f"[ESP Session] Cockpit {cockpit_id} stopped")

    return True, None

def _load_cockpit_settings():
    """Load persistent per-cockpit settings from disk."""
    try:
        if not os.path.exists(COCKPIT_SETTINGS_FILE):
            print("[Cockpit Settings] No saved settings found; using defaults")
            return

        with open(COCKPIT_SETTINGS_FILE, "r", encoding="utf-8") as handle:
            saved = json.load(handle)

        if not isinstance(saved, dict):
            print("[Cockpit Settings] Invalid settings file; using defaults")
            return

        with cockpit_settings_lock:
            for cockpit_id in range(
                1,
                cockpit_manager.max_cockpits + 1
            ):
                saved_settings = saved.get(str(cockpit_id))

                if not isinstance(saved_settings, dict):
                    continue

                current = cockpit_settings[cockpit_id]

                try:
                    value = int(
                        saved_settings.get(
                            "session_duration_minutes",
                            current["session_duration_minutes"]
                        )
                    )
                    if 1 <= value <= 120:
                        current["session_duration_minutes"] = value
                except (TypeError, ValueError):
                    pass

                try:
                    value = int(
                        saved_settings.get(
                            "steering_sensitivity",
                            current["steering_sensitivity"]
                        )
                    )
                    if 10 <= value <= 200:
                        current["steering_sensitivity"] = value
                except (TypeError, ValueError):
                    pass

                try:
                    value = int(
                        saved_settings.get(
                            "throttle_sensitivity",
                            current["throttle_sensitivity"]
                        )
                    )
                    if 10 <= value <= 100:
                        current["throttle_sensitivity"] = value
                except (TypeError, ValueError):
                    pass
                
                try:
                    val = saved_settings.get("autocenter_enabled", current["autocenter_enabled"])
                    if isinstance(val, bool):
                        current["autocenter_enabled"] = val
                    elif isinstance(val, str):
                        current["autocenter_enabled"] = val.lower() == "true"
                except Exception:
                    pass

        print("[Cockpit Settings] Persistent settings loaded")

    except Exception as exc:
        print(f"[Cockpit Settings] Load error: {exc}")


def _save_cockpit_settings():
    """Atomically save per-cockpit settings to disk."""
    try:
        with cockpit_settings_lock:
            data = {
                str(cockpit_id): dict(settings)
                for cockpit_id, settings in cockpit_settings.items()
            }

        temp_path = COCKPIT_SETTINGS_FILE + ".tmp"

        with open(temp_path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2, sort_keys=True)
            handle.write("\n")

        os.replace(temp_path, COCKPIT_SETTINGS_FILE)

    except Exception as exc:
        print(f"[Cockpit Settings] Save error: {exc}")

_load_cockpit_settings()


def _active_ffb_cockpit_ids():
    active = []
    with cockpit_sessions_lock:
        for cockpit_id, session in cockpit_sessions.items():
            if session["active"]:
                active.append(cockpit_id)
    return active


def _release_cockpit_impact_ffb(cockpit_id):
    try:
        enabled = True
        with cockpit_settings_lock:
            if cockpit_id in cockpit_settings:
                enabled = cockpit_settings[cockpit_id].get("autocenter_enabled", True)
        
        set_autocenter_hardware(enabled, cockpit_id=cockpit_id)
        print(f"[FFB] Cockpit {cockpit_id} impact kick released")
    except Exception as exc:
        print(
            f"[FFB] Cockpit {cockpit_id} impact release error: {exc}"
        )


def handle_telemetry_impact(impact):
    """Route an impact from an RX to the cockpit currently driving that vehicle.

    Telemetry carries the permanent ESP32 RX ID. The active vehicle mapping
    maintained by RadioManager provides the same RX ID for each selected
    vehicle. Therefore routing is deterministic even when vehicles are
    dynamically swapped between cockpits.

    This implementation is generic for all four logical cockpits and does not
    depend on the number of physically connected cockpits or RXs.
    """
    receiver_id = str(impact.get("receiver_id", "")).strip().upper()

    with cockpit_sessions_lock:
        print(
            "[FFB DEBUG] sessions=",
            {
                cockpit_id: session["active"]
                for cockpit_id, session in cockpit_sessions.items()
            }
        )

    print(
        "[FFB DEBUG] vehicles=",
        {
            cockpit_id: getattr(
                cockpit_manager.get_cockpit(cockpit_id),
                "vehicle_name",
                None
            )
            for cockpit_id in cockpit_sessions
        }
    )

    source_ip = impact.get("source_ip", "unknown")
    active_cockpits = _active_ffb_cockpit_ids()

    if not receiver_id:
        print(
            f"[FFB] Impact not routed | missing RX ID | "
            f"source={source_ip} | active_cockpits={active_cockpits}"
        )
        return

    radio_manager = cockpit_manager.radio_manager

    # Find the logical cockpit whose currently selected vehicle is backed by
    # this exact permanent RX ID. This naturally supports dynamic vehicle
    # swaps because the lookup is performed for every impact.
    matched_cockpit_id = None
    matched_vehicle_name = None

    for cockpit_id in active_cockpits:
        cockpit = cockpit_manager.get_cockpit(cockpit_id)
        if cockpit is None or cockpit.vehicle_name is None:
            continue

        active_vehicle = radio_manager.get_active_vehicle(
            cockpit.vehicle_name
        )

        if active_vehicle is None:
            continue

        active_receiver_id = str(
            active_vehicle.get("receiver_id", "")
        ).strip().upper()

        if active_receiver_id == receiver_id:
            matched_cockpit_id = cockpit_id
            matched_vehicle_name = cockpit.vehicle_name
            break

    if matched_cockpit_id is None:
        print(
            f"[FFB] Impact not routed | RX={receiver_id} | "
            f"source={source_ip} | active_cockpits={active_cockpits} | "
            "no matching active vehicle"
        )
        return

    ffb = get_ffb_device(matched_cockpit_id)

    if ffb is None:
        print(
            f"[FFB] Impact not routed | RX={receiver_id} | "
            f"Cockpit {matched_cockpit_id} has no FFB device"
        )
        return

    effect_type = impact.get("effect_type", "kick")
    jerk = float(impact["jerk"])

    print(
        f"[FFB] EVENT: {effect_type.upper()} → Cockpit {matched_cockpit_id} | "
        f"vehicle={matched_vehicle_name} | jerk={jerk:.1f}"
    )

    cockpit = cockpit_manager.get_cockpit(matched_cockpit_id)
    with cockpit_settings_lock:
        settings = dict(cockpit_settings[matched_cockpit_id])
    scale = settings.get("haptic_scale", 100) / 100.0

    if effect_type == "kick":
        severity = jerk / 50.0
        strength = min(FFB_MAX, max(FFB_MIN, severity * FFB_MULTIPLIER)) * scale
        if hasattr(ffb, 'play_kick'):
            ffb.play_kick(strength, 250)
    elif effect_type == "terrain":
        if hasattr(ffb, 'play_terrain'):
            ffb.play_terrain(500, scale=scale)
    elif effect_type == "rumble":
        if hasattr(ffb, 'play_rumble'):
            ffb.play_rumble(500, scale=scale)



telemetry_receiver = TelemetryReceiver(
    on_telemetry=lambda event: None,
    on_impact=handle_telemetry_impact,
)

telemetry_session_lock = threading.Lock()


def sync_telemetry_receiver():
    """
    Run telemetry reception only while at least one RF cockpit
    session is active.
    """
    with cockpit_sessions_lock:
        session_active = any(
            session["active"]
            for session in cockpit_sessions.values()
        )

    with telemetry_session_lock:
        running = bool(
            telemetry_receiver._thread
            and telemetry_receiver._thread.is_alive()
        )

        if session_active and not running:
            print("[Telemetry] Starting - RF session active")
            telemetry_receiver.start()

        elif not session_active and running:
            print("[Telemetry] Stopping - no RF session active")
            telemetry_receiver.stop()


# Multi-cockpit wheel -> RadioManager control.
# One generic worker implementation is used for every logical cockpit.
# Only cockpits with successfully discovered wheel/radio pairs will actively
# send control commands; unused cockpits remain idle.
COCKPIT_CONTROL_INTERVAL = 0.02  # 50 Hz
COCKPIT_CONTROL_SCALE = 1000
COCKPIT_CONTROL_DEADZONE = 10
cockpit_control_stop = threading.Event()
cockpit_control_threads = {}

esp_control_stop = threading.Event()
esp_control_threads = {}

# Runtime wheel hot-plug monitor. This watches for G29 connection changes
# while Flask remains running; Nano/radio connections are not rediscovered.
COCKPIT_DEVICE_SCAN_INTERVAL = 1.0
cockpit_device_monitor_stop = threading.Event()
cockpit_device_monitor_thread = None
cockpit_wheel_state_lock = threading.Lock()


VEHICLE_DISCOVERY_INTERVAL = 1.0
vehicle_discovery_stop = threading.Event()
vehicle_discovery_thread = None
vehicle_discovery_lock = threading.Lock()

radio_discovery_lock = threading.Lock()

vehicle_refresh_lock = threading.Lock()

def get_ffb_device(cockpit_id=None):
    """Return the FFB instance bound to a logical cockpit's G29."""
    global g29_ffb_instance

    with cockpit_ffb_lock:
        if cockpit_id is not None:
            return cockpit_ffb_instances.get(cockpit_id)

        # Compatibility path for the existing global API: return the first
        # currently bound cockpit FFB instance, if any.
        for ffb in cockpit_ffb_instances.values():
            if ffb is not None:
                g29_ffb_instance = ffb
                return ffb
        return g29_ffb_instance


def _set_cockpit_ffb_instance(cockpit_id, ffb):
    global g29_ffb_instance
    with cockpit_ffb_lock:
        old_ffb = cockpit_ffb_instances.get(cockpit_id)
        cockpit_ffb_instances[cockpit_id] = ffb
        if g29_ffb_instance is None:
            g29_ffb_instance = ffb
    if old_ffb is not None and old_ffb is not ffb:
        try:
            old_ffb.stop()
        except Exception:
            pass


def _remove_cockpit_ffb_instance(cockpit_id, expected=None):
    global g29_ffb_instance
    with cockpit_ffb_lock:
        current = cockpit_ffb_instances.get(cockpit_id)
        if expected is not None and current is not expected:
            return current
        cockpit_ffb_instances.pop(cockpit_id, None)
        if g29_ffb_instance is current:
            g29_ffb_instance = next(
                (ffb for ffb in cockpit_ffb_instances.values() if ffb is not None),
                None,
            )
        return current


# Hardware spring strength used whenever autocenter is enabled.
# Previous value of 22% was too weak and felt like "no autocenter".
G29_AUTOCENTER_STRENGTH = 25


def set_autocenter_hardware(enabled: bool, cockpit_id=None):
    """Set hardware centering for one cockpit, or all active cockpit wheels."""
    strength = G29_AUTOCENTER_STRENGTH if enabled else 0

    if cockpit_id is not None:
        ffb = get_ffb_device(cockpit_id)
        if not ffb:
            print(
                f"[FFB] Autocenter skip: no FFB device for cockpit {cockpit_id}"
            )
            return False
        ok = ffb.set_autocenter(strength)
        print(
            f"[FFB] Cockpit {cockpit_id} autocenter "
            f"{'ON' if enabled else 'OFF'} strength={strength} ok={ok}"
        )
        return ok

    with cockpit_ffb_lock:
        items = list(cockpit_ffb_instances.items())

    if not items:
        print("[FFB] Autocenter skip: no cockpit FFB instances")
        return False

    any_ok = False
    for cid, ffb in items:
        if not ffb:
            continue
        try:
            ok = ffb.set_autocenter(strength)
            any_ok = any_ok or bool(ok)
            print(
                f"[FFB] Cockpit {cid} autocenter "
                f"{'ON' if enabled else 'OFF'} strength={strength} ok={ok}"
            )
        except Exception as exc:
            print(f"[FFB Error] Auto-center update failed cockpit {cid}: {exc}")
    return any_ok


def drive_worker():
    # Preserve the existing single-drive-loop behavior for now.
    # Per-cockpit control becomes active at the wheel->Nano multi-cockpit stage.
    set_autocenter_hardware(autocenter_enabled)
    minimal_drive.run_loop(
        is_active_callback=lambda: session_active,
        get_settings=lambda: (steering_sensitivity, throttle_limit),
        is_autocenter_enabled=lambda: autocenter_enabled,
        get_haptic_settings=lambda: haptic_settings
    )

def find_vehicle_by_transponder_id(transponder_id):
    transponder_id = int(transponder_id)

    # -------------------------------------------------
    # RF vehicles
    # -------------------------------------------------
    registry = cockpit_manager.radio_manager.vehicle_registry

    for receiver_id, vehicle in registry.all().items():
        if not isinstance(vehicle, dict):
            continue

        stored_id = vehicle.get("transponder_id")

        if stored_id is None:
            continue

        try:
            stored_id = int(stored_id)
        except (TypeError, ValueError):
            continue

        if stored_id == transponder_id:
            return {
                "type": "rf",
                "receiver_id": str(receiver_id).strip().upper(),
                "name": str(
                    vehicle.get("name", "")
                ).strip(),
                "transponder_id": stored_id
            }

    # -------------------------------------------------
    # ESP vehicles
    # -------------------------------------------------
    with esp_vehicle_lock:
        for cockpit_id, vehicle in esp_vehicles.items():

            stored_id = vehicle.get("transponder_id")

            if stored_id is None:
                continue

            try:
                stored_id = int(stored_id)
            except (TypeError, ValueError):
                continue

            if stored_id == transponder_id:
                print(
                    f"[Lap Timer] ESP vehicle mapped: "
                    f"cockpit={cockpit_id} "
                    f"vehicle={vehicle['name']} "
                    f"transponder={stored_id}"
                )

                return {
                    "type": "esp",
                    "cockpit_id": cockpit_id,
                    "receiver_id": f"ESP{cockpit_id}",
                    "name": str(
                        vehicle.get("name", "")
                    ).strip(),
                    "transponder_id": stored_id
                }

    return None

def find_active_cockpit_for_vehicle(vehicle):
    """
    Find the cockpit currently running a session for this vehicle.
    Supports both RF and ESP vehicles.
    """

    if vehicle is None:
        return None

    vehicle_type = vehicle.get("type")

    # -------------------------------------------------
    # ESP vehicle
    # -------------------------------------------------
    if vehicle_type == "esp":

        cockpit_id = vehicle.get("cockpit_id")

        if cockpit_id is None:
            return None

        try:
            cockpit_id = int(cockpit_id)
        except (TypeError, ValueError):
            return None

        if cockpit_id not in esp_cockpits:
            return None

        with esp_cockpit_sessions_lock:
            session = esp_cockpit_sessions[cockpit_id]

            if not session["active"]:
                return None

        # Verify that this ESP car is actually assigned
        # to this cockpit.
        with esp_cockpit_vehicle_lock:
            assigned_car_id = esp_cockpit_vehicles.get(cockpit_id)

        if assigned_car_id != cockpit_id:
            return None

        print(
            f"[Lap Timer] ESP vehicle active in "
            f"Cockpit {cockpit_id}"
        )

        return cockpit_id

    # -------------------------------------------------
    # RF vehicle
    # -------------------------------------------------
    vehicle_name = vehicle.get("name")
    receiver_id = vehicle.get("receiver_id")

    for cockpit_id in range(
        1,
        cockpit_manager.max_cockpits + 1
    ):
        cockpit = cockpit_manager.get_cockpit(cockpit_id)

        if cockpit is None:
            continue

        with cockpit_sessions_lock:
            session = cockpit_sessions[cockpit_id]

            if not session["active"]:
                continue

        if cockpit.vehicle_name != vehicle_name:
            continue

        active_vehicle = (
            cockpit_manager.radio_manager.get_active_vehicle(
                cockpit.vehicle_name
            )
        )

        if active_vehicle is not None:
            active_receiver_id = str(
                active_vehicle.get("receiver_id", "")
            ).strip().upper()

            if active_receiver_id != receiver_id:
                continue

        return cockpit_id

    return None


def process_lap_detection(vehicle, cockpit_id, timer):
    """
    Process a transponder detection for an active cockpit session.

    The first detection establishes the lap timing reference.
    Subsequent detections are accepted only when they are at least
    MIN_LAP_TIME_MS apart.
    """

    receiver_id = vehicle["receiver_id"]

    state = lap_timer_vehicle_state.get(receiver_id)

    if state is None:

        if vehicle.get("type") == "esp":
            with esp_cockpit_driver_lock:
                driver = dict(
                    esp_cockpit_drivers.get(cockpit_id, {})
                )
        else:
            with cockpit_driver_lock:
                driver = dict(
                    cockpit_drivers.get(cockpit_id, {})
                )

        driver_name = driver.get("name") or "Unknown Driver"

        lap_timer_vehicle_state[receiver_id] = {
            "cockpit_id": cockpit_id,
            "driver_id": driver.get("user_id"),
            "driver_name": driver_name,
            "last_timer": timer,
            "lap_count": 0,
            "last_lap_ms": None,
            "best_lap_ms": None
        }

        print(
            f"[Lap Timer] First detection: "
            f"cockpit={cockpit_id} "
            f"vehicle={vehicle['name']} "
            f"timer={timer}"
        )

        return

    elapsed_ms = timer - state["last_timer"]

    if elapsed_ms < MIN_LAP_TIME_MS:
        print(
            f"[Lap Timer] Detection ignored: "
            f"cockpit={cockpit_id} "
            f"elapsed={elapsed_ms}ms "
            f"< {MIN_LAP_TIME_MS}ms"
        )
        return

    state["last_timer"] = timer
    state["lap_count"] += 1
    state["last_lap_ms"] = elapsed_ms

    if (
        state["best_lap_ms"] is None
        or elapsed_ms < state["best_lap_ms"]
    ):
        state["best_lap_ms"] = elapsed_ms

    # Store completed lap statistics separately from the
    # current-session lap timer state.
    driver_id = state.get("driver_id")

    if driver_id:
        driver_id = str(driver_id)

        leaderboard = leaderboard_driver_state.get(driver_id)

        if leaderboard is None:
            leaderboard_driver_state[driver_id] = {
                "driver_name": state.get(
                    "driver_name",
                    "Unknown Driver"
                ),
                "best_lap_ms": elapsed_ms,
                "last_lap_ms": elapsed_ms,
                "lap_count": 1
            }
        else:
            leaderboard["driver_name"] = state.get(
                "driver_name",
                leaderboard["driver_name"]
            )

            leaderboard["last_lap_ms"] = elapsed_ms

            leaderboard["lap_count"] += 1

            if elapsed_ms < leaderboard["best_lap_ms"]:
                leaderboard["best_lap_ms"] = elapsed_ms

        print(
            f"[Leaderboard State] "
            f"driver={driver_id} "
            f"best={leaderboard_driver_state[driver_id]['best_lap_ms']}ms "
            f"last={leaderboard_driver_state[driver_id]['last_lap_ms']}ms "
            f"laps={leaderboard_driver_state[driver_id]['lap_count']}"
        )

        # Save completed lap permanently.
        if driver_id:
            conn = _get_user_db()

            conn.execute(
                """
                INSERT INTO lap_history
                (
                    user_id,
                    driver_name,
                    lap_time_ms,
                    recorded_at,
                    cockpit_id,
                    vehicle_name
                )
                VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    str(driver_id),
                    state.get("driver_name", "Unknown Driver"),
                    int(elapsed_ms),
                    time.strftime(
                        "%Y-%m-%d %H:%M:%S"
                    ),
                    int(cockpit_id),
                    vehicle.get("name")
                )
            )

        conn.commit()
        conn.close()

        print(
            f"[Lap History] Saved: "
            f"driver={driver_id} "
            f"time={elapsed_ms}ms"
        )

    # -------------------------------------------------
    # Send lap data to RF RX overlay
    # -------------------------------------------------
    if vehicle.get("type") == "rf":
        cockpit = cockpit_manager.get_cockpit(cockpit_id)

        if cockpit is not None and cockpit.radio_id is not None:
            radio = cockpit_manager.radio_manager.get_radio(
                cockpit.radio_id
            )

            if radio is not None:
                radio.send_lap(
                    state["lap_count"],
                    state["last_lap_ms"],
                    state["best_lap_ms"]
                )

                print(
                    f"[Lap TX] RF Cockpit {cockpit_id}: "
                    f"lap={state['lap_count']} "
                    f"last={state['last_lap_ms']}ms "
                    f"best={state['best_lap_ms']}ms"
                )

    print(
        f"[Lap Timer] VALID LAP: "
        f"cockpit={cockpit_id} "
        f"vehicle={vehicle['name']} "
        f"lap={state['lap_count']} "
        f"time={elapsed_ms}ms "
        f"best={state['best_lap_ms']}ms"
    )

def _get_user_db():
    conn = sqlite3.connect(USER_DATABASE_FILE)
    conn.row_factory = sqlite3.Row
    return conn


def _ensure_user_table():
    os.makedirs(
        os.path.dirname(USER_DATABASE_FILE),
        exist_ok=True
    )

    conn = _get_user_db()

    conn.execute("""
        CREATE TABLE IF NOT EXISTS users
        (
            user_id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            dob TEXT,
            phone TEXT,
            email TEXT,
            throttle_limit INTEGER DEFAULT 100,
            steering_sensitivity INTEGER DEFAULT 100,
            created_at TEXT,
            updated_at TEXT,
            last_used TEXT
        )
    """)

    conn.execute("""
        CREATE TABLE IF NOT EXISTS lap_history
        (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id TEXT NOT NULL,
            driver_name TEXT NOT NULL,
            lap_time_ms INTEGER NOT NULL,
            recorded_at TEXT NOT NULL,
            cockpit_id INTEGER,
            vehicle_name TEXT
        )
    """)

    conn.commit()
    conn.close()

def _get_user_db():
    conn = sqlite3.connect(USER_DATABASE_FILE)
    conn.row_factory = sqlite3.Row
    return conn


def _ensure_user_table():
    os.makedirs(
        os.path.dirname(USER_DATABASE_FILE),
        exist_ok=True
    )

    conn = _get_user_db()

    conn.execute("""
        CREATE TABLE IF NOT EXISTS users
        (
            user_id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            dob TEXT,
            phone TEXT,
            email TEXT,
            throttle_limit INTEGER DEFAULT 100,
            steering_sensitivity INTEGER DEFAULT 100,
            created_at TEXT,
            updated_at TEXT,
            last_used TEXT
        )
    """)

    conn.commit()
    conn.close()

_ensure_user_table()

def get_user_by_id(user_id):
    user_id = str(user_id).strip()

    conn = _get_user_db()

    row = conn.execute("""
        SELECT user_id, name, phone
        FROM users
        WHERE user_id = ?
    """, (user_id,)).fetchone()

    conn.close()

    if row is None:
        return None

    return {
        "user_id": row["user_id"],
        "name": row["name"],
        "phone": row["phone"] or "",
    }


def update_user_last_used(user_id):
    user_id = str(user_id).strip()

    if not user_id:
        return

    now = time.strftime("%Y-%m-%d %H:%M:%S")

    conn = _get_user_db()

    conn.execute("""
        UPDATE users
        SET
            last_used = ?,
            updated_at = ?
        WHERE user_id = ?
    """, (
        now,
        now,
        user_id
    ))

    conn.commit()
    conn.close()


def find_driver_cockpit(user_id, exclude_type=None, exclude_cockpit_id=None):
    user_id = str(user_id)

    # Check RF cockpits.
    with cockpit_driver_lock:
        for cockpit_id, driver in cockpit_drivers.items():
            if (
                exclude_type == "rf"
                and cockpit_id == exclude_cockpit_id
            ):
                continue

            if (
                driver.get("user_id")
                and str(driver["user_id"]) == user_id
            ):
                return "RF", cockpit_id

    # Check ESP cockpits.
    with esp_cockpit_driver_lock:
        for cockpit_id, driver in esp_cockpit_drivers.items():
            if (
                exclude_type == "esp"
                and cockpit_id == exclude_cockpit_id
            ):
                continue

            if (
                driver.get("user_id")
                and str(driver["user_id"]) == user_id
            ):
                return "ESP", cockpit_id

    return None

esp_ap_lock = threading.Lock()
esp_ap_macs = set()


def _reverse_mac_bytes(mac):
    """
    Convert Wi-Fi MAC format to the byte-reversed vehicle registry format.

    Example:
        28:84:85:57:65:5C
        -> 5C6557858428
    """
    parts = str(mac).strip().upper().split(":")

    if len(parts) != 6:
        return None

    if any(
        len(part) != 2
        or any(char not in "0123456789ABCDEF" for char in part)
        for part in parts
    ):
        return None

    return "".join(reversed(parts))


def _get_pi_ap_esp_macs():
    """
    Return MAC addresses of devices currently associated with wlan0 AP.
    """
    try:
        output = subprocess.check_output(
            ["iw", "dev", "wlan0", "station", "dump"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=2.0
        )

    except Exception as exc:
        print(f"[ESP AP] Station scan error: {exc}")
        return set()

    macs = set()

    for line in output.splitlines():
        line = line.strip()

        if not line.startswith("Station "):
            continue

        parts = line.split()

        if len(parts) < 2:
            continue

        mac = parts[1].strip().upper()

        if re.fullmatch(
            r"[0-9A-F]{2}(:[0-9A-F]{2}){5}",
            mac
        ):
            macs.add(mac)

    return macs


def update_esp_ap_presence():
    """
    Update live ESP vehicle presence from Pi AP associations.
    """
    wifi_macs = _get_pi_ap_esp_macs()

    registry = cockpit_manager.radio_manager.vehicle_registry

    matched_vehicle_ids = set()

    for wifi_mac in wifi_macs:
        vehicle_id = _reverse_mac_bytes(wifi_mac)

        if vehicle_id is None:
            continue

        if registry.get_vehicle(vehicle_id) is not None:
            matched_vehicle_ids.add(vehicle_id)

    with esp_ap_lock:
        esp_ap_macs.clear()
        esp_ap_macs.update(matched_vehicle_ids)


def esp_ap_monitor_worker():
    """
    Continuously monitor ESP devices associated with the Pi AP.
    """
    while not vehicle_discovery_stop.wait(1.0):
        try:
            update_esp_ap_presence()

        except Exception as exc:
            print(f"[ESP AP] Monitor error: {exc}")

def vehicle_discovery_worker():
    """Disabled continuous RF discovery.

    RF receiver discovery is performed only during startup
    and explicit Refresh.
    """
    while not vehicle_discovery_stop.wait(1.0):
        pass

def start_vehicle_discovery():
    global vehicle_discovery_thread

    with vehicle_discovery_lock:
        if (
            vehicle_discovery_thread is not None
            and vehicle_discovery_thread.is_alive()
        ):
            print("[Vehicle Discovery] Already running")
            return

        vehicle_discovery_stop.clear()

        vehicle_discovery_thread = threading.Thread(
            target=vehicle_discovery_worker,
            name="vehicle-discovery",
            daemon=True
        )

        vehicle_discovery_thread.start()

    print("[Vehicle Discovery] Background worker started")

def stop_vehicle_discovery():
    global vehicle_discovery_thread

    vehicle_discovery_stop.set()

    thread = vehicle_discovery_thread

    if thread is not None:
        thread.join(timeout=3.0)

    vehicle_discovery_thread = None

    print("[Vehicle Discovery] Background worker stopped")

@app.route("/leaderboard")
def leaderboard_page():
    return render_template("leaderboard.html")


@app.route("/api/users/last-active", methods=["GET"])
def user_management_last_active_api():
    try:
        conn = _get_user_db()

        rows = conn.execute("""
            SELECT
                user_id,
                name,
                last_used
            FROM users
            WHERE last_used IS NOT NULL
              AND TRIM(last_used) != ''
            ORDER BY last_used DESC
            LIMIT 10
        """).fetchall()

        conn.close()

        users = []

        for row in rows:
            user_id = str(row["user_id"])

            best_lap_ms = None
            last_lap_ms = None
            lap_count = 0

            leaderboard = leaderboard_driver_state.get(user_id)

            if leaderboard is not None:
                best_lap_ms = leaderboard.get("best_lap_ms")
                last_lap_ms = leaderboard.get("last_lap_ms")
                lap_count = leaderboard.get("lap_count", 0)

            users.append({
                "user_id": user_id,
                "name": row["name"],
                "best_lap_ms": best_lap_ms,
                "last_lap_ms": last_lap_ms,
                "lap_count": lap_count,
                "last_used": row["last_used"]
            })

        return jsonify({
            "success": True,
            "users": users
        })

    except Exception as exc:
        print(
            f"[User Management] LAST ACTIVE error: {exc}"
        )

        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500



@app.route("/api/leaderboard", methods=["GET"])
def leaderboard_api():
    try:
        conn = _get_user_db()

        rows = conn.execute("""
            SELECT
                user_id,
                driver_name,
                MIN(lap_time_ms) AS best_lap_ms,
                COUNT(*) AS lap_count
            FROM lap_history
            GROUP BY user_id, driver_name
            ORDER BY best_lap_ms ASC
        """).fetchall()

        leaderboard = []

        for row in rows:
            last_lap = conn.execute("""
                SELECT lap_time_ms
                FROM lap_history
                WHERE user_id = ?
                ORDER BY id DESC
                LIMIT 1
            """, (row["user_id"],)).fetchone()

            leaderboard.append({
                "driver_name": row["driver_name"],
                "best_lap_ms": int(row["best_lap_ms"]),
                "last_lap_ms": (
                    int(last_lap["lap_time_ms"])
                    if last_lap is not None
                    else None
                ),
                "lap_count": int(row["lap_count"])
            })

        conn.close()

        return jsonify({
            "success": True,
            "leaderboard": leaderboard
        })

    except Exception as exc:
        print(f"[Leaderboard] GET error: {exc}")

        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route(
    "/api/cockpits/<int:cockpit_id>/driver",
    methods=["POST"]
)
def set_cockpit_driver(cockpit_id):

    if not validate_cockpit_id(cockpit_id):
        return jsonify({
            "success": False,
            "error": "Invalid cockpit ID"
        }), 400

    with cockpit_sessions_lock:
        if cockpit_sessions[cockpit_id]["active"]:
            return jsonify({
                "success": False,
                "error": "Cannot change driver while session is active"
            }), 400

    data = request.get_json() or {}
    user_id = data.get("user_id")

    if not user_id:
        return jsonify({
            "success": False,
            "error": "Driver is required"
        }), 400

    user = get_user_by_id(user_id)

    if user is None:
        return jsonify({
            "success": False,
            "error": "Driver not found"
        }), 404

    # Prevent the same driver from being assigned to
    # multiple cockpits simultaneously.

    existing = find_driver_cockpit(
        user["user_id"],
        exclude_type="rf",
        exclude_cockpit_id=cockpit_id
    )

    if existing is not None:
        cockpit_type, other_cockpit_id = existing

        return jsonify({
            "success": False,
            "error": (
                f"Driver is already selected in "
                f"{cockpit_type} Cockpit {other_cockpit_id}"
            )
        }), 409

    with cockpit_driver_lock:
        for other_cockpit_id, driver in cockpit_drivers.items():

            if other_cockpit_id == cockpit_id:
                continue

            if driver["user_id"] == user["user_id"]:

                with cockpit_sessions_lock:
                    other_active = cockpit_sessions[
                        other_cockpit_id
                    ]["active"]

                if other_active:
                    return jsonify({
                        "success": False,
                        "error": (
                            f"Driver is already active in "
                            f"Cockpit {other_cockpit_id}"
                        )
                    }), 409

        cockpit_drivers[cockpit_id] = {
            "user_id": user["user_id"],
            "name": user["name"],
            "phone": user["phone"],
        }

    print(
        f"[Driver] Cockpit {cockpit_id} selected: "
        f"{user['name']} ({user['user_id']})"
    )

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id,
        "driver": user
    })

@app.route("/api/users", methods=["GET"])
def user_management_list_api():
    try:
        conn = _get_user_db()

        rows = conn.execute("""
            SELECT
                user_id,
                name,
                dob,
                phone,
                email
            FROM users
            ORDER BY created_at ASC
        """).fetchall()

        conn.close()

        users = []

        for row in rows:
            users.append({
                "user_id": row["user_id"],
                "name": row["name"],
                "dob": row["dob"] or "",
                "phone": row["phone"] or "",
                "email": row["email"] or ""
            })

        return jsonify({
            "success": True,
            "users": users
        })

    except Exception as exc:
        print(f"[User Management] GET error: {exc}")

        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route("/api/users", methods=["POST"])
def user_management_create_api():
    try:
        data = request.get_json() or {}

        name = str(data.get("name", "")).strip()
        dob = str(data.get("dob", "")).strip()
        phone = str(data.get("phone", "")).strip()
        email = str(data.get("email", "")).strip()

        if not name:
            return jsonify({
                "success": False,
                "error": "Name is required"
            }), 400

        # Generate a simple permanent user ID.
        timestamp = int(time.time() * 1000)
        user_id = f"USR{timestamp}"

        now = time.strftime(
            "%Y-%m-%d %H:%M:%S"
        )

        conn = _get_user_db()

        conn.execute("""
            INSERT INTO users
            (
                user_id,
                name,
                dob,
                phone,
                email,
                throttle_limit,
                steering_sensitivity,
                created_at,
                updated_at
            )
            VALUES (?, ?, ?, ?, ?, 100, 100, ?, ?)
        """, (
            user_id,
            name,
            dob,
            phone,
            email,
            now,
            now
        ))

        conn.commit()
        conn.close()

        print(
            f"[User Management] User created: "
            f"{user_id} -> {name}"
        )

        return jsonify({
            "success": True,
            "user": {
                "user_id": user_id,
                "name": name,
                "dob": dob,
                "phone": phone,
                "email": email
            }
        })

    except sqlite3.IntegrityError:
        return jsonify({
            "success": False,
            "error": "User ID already exists"
        }), 409

    except Exception as exc:
        print(f"[User Management] CREATE error: {exc}")
        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route("/api/users/<user_id>", methods=["PUT"])
def user_management_update_api(user_id):
    try:
        user_id = str(user_id).strip()

        data = request.get_json() or {}

        name = str(data.get("name", "")).strip()
        dob = str(data.get("dob", "")).strip()
        phone = str(data.get("phone", "")).strip()
        email = str(data.get("email", "")).strip()

        if not name:
            return jsonify({
                "success": False,
                "error": "Name is required"
            }), 400

        now = time.strftime(
            "%Y-%m-%d %H:%M:%S"
        )

        conn = _get_user_db()

        cursor = conn.execute("""
            UPDATE users
            SET
                name = ?,
                dob = ?,
                phone = ?,
                email = ?,
                updated_at = ?
            WHERE user_id = ?
        """, (
            name,
            dob,
            phone,
            email,
            now,
            user_id
        ))

        conn.commit()
        conn.close()

        if cursor.rowcount == 0:
            return jsonify({
                "success": False,
                "error": "User not found"
            }), 404

        print(
            f"[User Management] User updated: "
            f"{user_id}"
        )

        return jsonify({
            "success": True
        })

    except Exception as exc:
        print(f"[User Management] UPDATE error: {exc}")
        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route("/api/users/<user_id>", methods=["DELETE"])
def user_management_delete_api(user_id):

    try:
        user_id = str(user_id).strip()

        conn = _get_user_db()

        cursor = conn.execute("""
            DELETE FROM users
            WHERE user_id = ?
        """, (user_id,))

        conn.commit()
        conn.close()

        if cursor.rowcount == 0:
            return jsonify({
                "success": False,
                "error": "User not found"
            }), 404

        print(
            f"[User Management] User deleted: "
            f"{user_id}"
        )

        return jsonify({
            "success": True
        })

    except Exception as exc:
        print(f"[User Management] DELETE error: {exc}")
        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500



@app.route("/user-management")
def user_management_page():
    return render_template("user_mnagement.html")


@app.route('/api/lap-timer/status')
def lap_timer_status_api():
    with lap_timer_lock:
        last_detection = dict(lap_timer_last_detection)

        return jsonify({
            "success": True,
            "running": lap_timer_running,
            "last_detection": last_detection
        })


@app.route('/vehicle-management')
def vehicle_management_page():
    return render_template('vehicle_management.html')

@app.route(
    "/api/esp-cockpits/<int:cockpit_id>/driver",
    methods=["POST"]
)
def set_esp_cockpit_driver(cockpit_id):

    if cockpit_id not in esp_cockpit_drivers:
        return jsonify({
            "success": False,
            "error": "Invalid ESP cockpit ID"
        }), 400

    with esp_cockpit_sessions_lock:
        if esp_cockpit_sessions[cockpit_id]["active"]:
            return jsonify({
                "success": False,
                "error": "Cannot change driver while session is active"
            }), 400

    data = request.get_json() or {}
    user_id = data.get("user_id")

    if not user_id:
        return jsonify({
            "success": False,
            "error": "Driver is required"
        }), 400

    user = get_user_by_id(user_id)

    if user is None:
        return jsonify({
            "success": False,
            "error": "Driver not found"
        }), 404

    # Prevent the same driver from being active in two ESP cockpits.
    existing = find_driver_cockpit(
        user["user_id"],
        exclude_type="esp",
        exclude_cockpit_id=cockpit_id
    )

    if existing is not None:
        cockpit_type, other_cockpit_id = existing

        return jsonify({
            "success": False,
            "error": (
                f"Driver is already selected in "
                f"{cockpit_type} Cockpit {other_cockpit_id}"
            )
        }), 409

    with esp_cockpit_driver_lock:
        for other_cockpit_id, driver in esp_cockpit_drivers.items():

            if other_cockpit_id == cockpit_id:
                continue

            if driver["user_id"] == user["user_id"]:

                with esp_cockpit_sessions_lock:
                    other_active = esp_cockpit_sessions[
                        other_cockpit_id
                    ]["active"]

                if other_active:
                    return jsonify({
                        "success": False,
                        "error": (
                            f"Driver is already active in "
                            f"ESP Cockpit {other_cockpit_id}"
                        )
                    }), 409

        esp_cockpit_drivers[cockpit_id] = {
            "user_id": user["user_id"],
            "name": user["name"],
            "phone": user["phone"],
        }

    print(
        f"[ESP Driver] Cockpit {cockpit_id} selected: "
        f"{user['name']} ({user['user_id']})"
    )

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id,
        "driver": user
    })

@app.route(
    '/api/esp-cockpits/<int:cockpit_id>/vehicle',
    methods=['POST']
)
def set_esp_cockpit_vehicle(cockpit_id):

    if cockpit_id not in esp_cockpits:
        return jsonify({
            "success": False,
            "error": "Invalid ESP cockpit ID"
        }), 400

    data = request.get_json() or {}

    try:
        car_id = int(data.get("car_id"))
    except (TypeError, ValueError):
        return jsonify({
            "success": False,
            "error": "Invalid ESP vehicle"
        }), 400

    with esp_vehicle_lock:
        vehicle = esp_vehicles.get(car_id)

        if vehicle is None:
            return jsonify({
                "success": False,
                "error": "ESP vehicle not found"
            }), 404

    with esp_cockpit_sessions_lock:
        if esp_cockpit_sessions[cockpit_id]["active"]:
            return jsonify({
                "success": False,
                "error": "Cannot change vehicle while session is active"
            }), 400

    # Do not allow the same ESP car to be assigned to
    # two ESP cockpits simultaneously.
    with esp_cockpit_vehicle_lock:
        for other_cockpit_id, assigned_car_id in esp_cockpit_vehicles.items():

            if other_cockpit_id == cockpit_id:
                continue

            if assigned_car_id == car_id:
                return jsonify({
                    "success": False,
                    "error": (
                        f"Vehicle is already assigned to "
                        f"ESP Cockpit {other_cockpit_id}"
                    )
                }), 409

        esp_cockpit_vehicles[cockpit_id] = car_id

    print(
        f"[ESP Vehicle] Cockpit {cockpit_id} -> "
        f"{vehicle['name']}"
    )

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id,
        "vehicle": {
            "car_id": car_id,
            "name": vehicle["name"],
            "transponder_id": vehicle["transponder_id"]
        }
    })

@app.route('/api/esp-cockpits/<int:cockpit_id>/session/start', methods=['POST'])
def esp_cockpit_session_start_api(cockpit_id):
    success, error = start_esp_cockpit_session(cockpit_id)

    if not success:
        return jsonify({
            "success": False,
            "error": error
        }), 400

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id
    })


@app.route('/api/esp-cockpits/<int:cockpit_id>/session/stop', methods=['POST'])
def esp_cockpit_session_stop_api(cockpit_id):
    success, error = stop_esp_cockpit_session(cockpit_id)

    if not success:
        return jsonify({
            "success": False,
            "error": error
        }), 400

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id
    })

@app.route('/api/esp-cockpits/<int:cockpit_id>/settings', methods=['GET', 'POST'])
def esp_cockpit_settings_api(cockpit_id):

    if cockpit_id not in esp_cockpit_settings:
        return jsonify({
            "success": False,
            "error": "Invalid ESP cockpit ID"
        }), 400

    if request.method == 'GET':
        with esp_cockpit_settings_lock:
            return jsonify({
                "success": True,
                "cockpit_id": cockpit_id,
                "settings": dict(
                    esp_cockpit_settings[cockpit_id]
                )
            })

    data = request.get_json() or {}

    with esp_cockpit_sessions_lock:
        if esp_cockpit_sessions[cockpit_id]["active"]:
            return jsonify({
                "success": False,
                "error": "Cannot change settings while session is active"
            }), 400

    with esp_cockpit_settings_lock:
        settings = esp_cockpit_settings[cockpit_id]

        if "autocenter_enabled" in data:
            enabled = bool(data["autocenter_enabled"])

            with esp_ffb_lock:
                ffb = esp_ffb_instances.get(cockpit_id)

            if ffb is not None:
                ffb.apply_enabled(
                    enabled,
                    strength_pct=G29_AUTOCENTER_STRENGTH
                )

        if "session_duration_minutes" in data:
            value = int(data["session_duration_minutes"])
            if not 1 <= value <= 120:
                return jsonify({
                    "success": False,
                    "error": "Session duration must be 1-120 minutes"
                }), 400
            settings["session_duration_minutes"] = value

        if "steering_sensitivity" in data:
            value = int(data["steering_sensitivity"])
            if not 10 <= value <= 200:
                return jsonify({
                    "success": False,
                    "error": "Steering sensitivity must be 10-200"
                }), 400
            settings["steering_sensitivity"] = value
        if "throttle_sensitivity" in data:
            value = int(data["throttle_sensitivity"])
            if not 10 <= value <= 100:
                return jsonify({
                    "success": False,
                    "error": "Throttle sensitivity must be 10-100"
                }), 400
            settings["throttle_sensitivity"] = value
            
        if "driver_name" in data:
            settings["driver_name"] = str(data["driver_name"]).strip()


        if "autocenter_enabled" in data:
            settings["autocenter_enabled"] = bool(
                data["autocenter_enabled"]
            )

    _save_esp_cockpit_settings()

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id,
        "settings": dict(esp_cockpit_settings[cockpit_id])
    })

@app.route('/api/esp-cockpits', methods=['GET'])
def esp_cockpit_status():
    try:
#        reconcile_esp_devices()

        cockpits = []

        with esp_cockpits_lock:
            device_snapshot = {
                cockpit_id: {
                    "wheel": cockpit["wheel"],
                    "esp": cockpit["esp"]
                }
                for cockpit_id, cockpit in esp_cockpits.items()
            }

        with esp_cockpit_settings_lock:
            settings_snapshot = {
                cockpit_id: dict(settings)
                for cockpit_id, settings in esp_cockpit_settings.items()
            }

        with esp_cockpit_driver_lock:
            driver_snapshot = {
                cockpit_id: dict(driver)
                for cockpit_id, driver in esp_cockpit_drivers.items()
            }

        for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1):

            device = device_snapshot[cockpit_id]

            wheel = None
            if device["wheel"] is not None:
                wheel = {
                    "name": device["wheel"].name,
                    "path": device["wheel"].path,
                    "phys": device["wheel"].phys
                }

            esp = None
            if device["esp"] is not None:
                esp = {
                    "port": device["esp"].device,
                    "connected": bool(
                        device["esp"].serial
                        and device["esp"].serial.is_open
                    )
                }

            with esp_cockpit_vehicle_lock:
                assigned_car_id = esp_cockpit_vehicles[cockpit_id]

            assigned_vehicle = get_esp_vehicle(assigned_car_id)

            cockpits.append({
                "cockpit_id": cockpit_id,
                "wheel": wheel,
                "esp": esp,
                "car_id": assigned_car_id,
                "car": assigned_vehicle["name"] if assigned_vehicle else None,
                "driver": driver_snapshot[cockpit_id],
                "settings": settings_snapshot[cockpit_id],
                "session": get_esp_cockpit_session_state(
                    cockpit_id
                )
            })

        return jsonify({
            "success": True,
            "cockpits": cockpits
        })

    except Exception as e:
        print(f"[ESP Cockpit API] Status error: {e}")

        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/esp-cockpits/refresh', methods=['POST'])
def esp_cockpit_refresh():
    """Explicit hardware rescan so an unplug/replug is picked up live.

    GET /api/esp-cockpits intentionally never scans hardware, so a wheel
    moved to a different USB port (or replugged into the same one) is
    invisible until this runs.
    """
    try:
        reconcile_esp_devices()

        # The control worker is otherwise only ever started once at Flask
        # boot. Restart it here for any cockpit that now has a wheel and an
        # ESP32 but no live worker, so replugged hardware works immediately.
        with esp_cockpits_lock:
            ready_cockpits = [
                cockpit_id
                for cockpit_id, cockpit in esp_cockpits.items()
                if cockpit["wheel"] is not None and cockpit["esp"] is not None
            ]

        for cockpit_id in ready_cockpits:
            start_esp_cockpit_control_worker(cockpit_id)

        return jsonify({"success": True})
    except Exception as e:
        print(f"[ESP Cockpit API] Refresh error: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route("/telemetry_status")
def telemetry_status():
    return jsonify(telemetry_receiver.get_status())


@app.route('/')
def home():
    return render_template('index.html')


def sync_vehicles_from_connected_nanos():
    """Scan connected Nanos, send PAIRING, upsert Vehicle Management records.

    For each connected Nano:
      WHO  -> Nano ID
      PAIRING -> RF_ID + VEHICLE name
    Then create the vehicle if missing, or fill empty nano_id if present.
    """
    try:
        summary = cockpit_manager.radio_manager.sync_registry_from_nano_pairings()
        return True, summary
    except Exception as exc:
        print(f"[Vehicle Sync] Nano pairing sync failed: {exc}")
        return False, {"error": str(exc)}


def initialize_cockpit_system():
    """Initial discovery of wheels and Nanos + registry pairing sync.

    Wheels and Nanos are discovered independently. Connected Nanos are
    queried with PAIRING so Vehicle Management is auto-populated from the
    permanent Nano -> RF ID -> vehicle name firmware pairing.
    """
    global cockpit_system_initialized

    with cockpit_init_lock:
        if cockpit_system_initialized:
            return

        refresh_cockpit_devices()

        # Startup: discover connected Nanos and materialize/update vehicles
        # from their fixed PAIRING response.
        ok, summary = sync_vehicles_from_connected_nanos()
        if ok:
            print(
                f"[Startup] Nano pairing sync complete: "
                f"created={summary.get('created', 0)} "
                f"updated={summary.get('updated', 0)} "
                f"unchanged={summary.get('unchanged', 0)} "
                f"failed={summary.get('failed', 0)}"
            )
        else:
            print(f"[Startup] Nano pairing sync skipped: {summary}")

        restore_preferred_car_assignments()
        cockpit_system_initialized = True


def _wheel_identity(wheel):
    """Return the best available runtime identity for a discovered G29."""
    uniq = str(getattr(wheel, "uniq", "") or "").strip()
    phys = str(getattr(wheel, "phys", "") or "").strip()
    if uniq:
        return ("uniq", uniq)
    if phys:
        return ("phys", phys)
    return ("path", str(getattr(wheel, "path", "")))

def _usb_parent_port_for_wheel(wheel):
    """Return the Pi root USB port for a discovered G29."""
    phys = str(getattr(wheel, "phys", "") or "").strip()

    match = re.search(r"-(\d+(?:\.\d+)*)/input\d+$", phys)
    if not match:
        return None

    chain = match.group(1).split(".")

    try:
        return int(chain[1])
    except (IndexError, ValueError):
        return None


def _is_rf_wheel(wheel):
    return _usb_parent_port_for_wheel(wheel) == 4


def _is_esp_wheel(wheel):
    return _usb_parent_port_for_wheel(wheel) == 3

def discover_esp_wheels():
    """Discover only G29 wheels assigned to the ESP wheel USB group."""
    wheels = []
    g29_name = "Logitech G29 Driving Force Racing Wheel"

    for path in sorted(glob.glob("/dev/input/event*")):
        device = None
        try:
            device = evdev.InputDevice(path)

            if device.name != g29_name:
                device.close()
                continue

            if not _is_esp_wheel(device):
                device.close()
                continue

            wheels.append(device)

        except Exception:
            if device is not None:
                try:
                    device.close()
                except Exception:
                    pass

    return wheels

def discover_esp_serial_ports():
    """Discover ESP32 devices using the stable Silicon Labs CP2102 USB identity."""
    import serial.tools.list_ports

    ports = []

    for port in serial.tools.list_ports.comports():
        if "Silicon Labs" in (port.manufacturer or ""):
            ports.append(port)

    return sorted(ports, key=lambda p: str(p.device))


class ESPUsbController:
    """USB serial controller for one ESP32 vehicle receiver."""

    BAUD_RATE = 115200

    def __init__(self, port_info):
        import serial

        self.device = str(port_info.device)
        self.serial = serial.Serial(
            self.device,
            self.BAUD_RATE,
            timeout=0.1
        )

    def send_control(self, throttle, steering):
        throttle = int(_clamp(throttle, 0, 255))
        steering = int(_clamp(steering, 0, 255))
        checksum = throttle ^ steering

        packet = bytes([
            0xAA,
            throttle,
            steering,
            checksum
        ])

        self.serial.write(packet)
        self.serial.flush()

    def close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass

def _fixed_esp_cockpit_id_for_wheel(wheel):
    """Map ESP G29 USB topology to a fixed ESP cockpit ID."""
    phys = str(getattr(wheel, "phys", "") or "").strip()

    match = re.search(r"-1\.3\.(\d+)/input0$", phys)
    if not match:
        return None

    cockpit_id = int(match.group(1))

    if 1 <= cockpit_id <= ESP_COCKPIT_COUNT:
        return cockpit_id

    return None


def reconcile_esp_devices():
    """Discover and assign ESP wheels and ESP32 controllers."""
    wheels = discover_esp_wheels()
    serial_ports = discover_esp_serial_ports()

    print("\n[ESP Reconciliation]")

    # Assign ESP wheels using the fixed USB topology.
    wheel_assignments = {}

    for wheel in wheels:
        cockpit_id = _fixed_esp_cockpit_id_for_wheel(wheel)

        if cockpit_id is None:
            print(
                f"  ESP wheel ignored: {wheel.path} "
                f"| phys={wheel.phys}"
            )
            try:
                wheel.close()
            except Exception:
                pass
            continue

        wheel_assignments[cockpit_id] = wheel

    # Assign ESP serial controllers in deterministic order.
    serial_assignments = {
        cockpit_id: port
        for cockpit_id, port in zip(
            range(1, ESP_COCKPIT_COUNT + 1),
            serial_ports
        )
    }

    with esp_cockpits_lock:
        for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1):
            new_wheel = wheel_assignments.get(cockpit_id)
            new_port = serial_assignments.get(cockpit_id)

            current = esp_cockpits[cockpit_id]

            # Keep the existing wheel object if it is still the same device.
            if new_wheel is not None:
                if (
                    current["wheel"] is not None
                    and current["wheel"].path == new_wheel.path
                ):
                    try:
                        new_wheel.close()
                    except Exception:
                        pass
                else:
                    if current["wheel"] is not None:
                        try:
                            current["wheel"].close()
                        except Exception:
                            pass

                    current["wheel"] = new_wheel

            else:
                # Wheel was disconnected.
                if current["wheel"] is not None:
                    try:
                        current["wheel"].close()
                    except Exception:
                        pass

                current["wheel"] = None

            # Create/reuse the ESP serial controller.
            if new_port is not None:
                new_device = str(new_port.device)
                old_controller = current["esp"]

                if (
                    old_controller is None
                    or old_controller.device != new_device
                ):
                    if old_controller is not None:
                        old_controller.close()

                    try:
                        current["esp"] = ESPUsbController(new_port)
                    except Exception as e:
                        print(
                            f"  ESP{cockpit_id}: serial open failed "
                            f"{new_device}: {e}"
                        )
                        current["esp"] = None

            print(
                f"  ESP Cockpit {cockpit_id}: "
                f"wheel={getattr(current['wheel'], 'path', None)} "
                f"| esp={getattr(current['esp'], 'device', None)}"
            )


def _current_wheel_identities():
    identities = {}
    for cockpit_id in range(1, cockpit_manager.max_cockpits + 1):
        cockpit = cockpit_manager.get_cockpit(cockpit_id)
        if cockpit is not None and cockpit.wheel is not None:
            identities[cockpit_id] = _wheel_identity(cockpit.wheel)
    return identities


def _fixed_cockpit_id_for_wheel(wheel):
    """Return the logical cockpit assigned to this G29 USB port."""
    phys = str(getattr(wheel, "phys", "") or "").strip()

    if not phys:
        return None

    match = re.search(r"-1\.4\.(\d+)/input0$", phys)

    if not match:
        return None

    port_number = int(match.group(1))

    if 1 <= port_number <= cockpit_manager.max_cockpits:
        return port_number

    return None


def _fixed_cockpit_wheel_pairs(wheels):
    """Map known G29 USB physical ports to logical cockpits."""
    pairs = {}

    for wheel in wheels:
        cockpit_id = _fixed_cockpit_id_for_wheel(wheel)

        if cockpit_id is None:
            continue

        if cockpit_id in pairs:
            print(
                f"[Hotplug] Multiple G29s map to Cockpit {cockpit_id}; "
                f"leaving duplicate {wheel.path} unassigned"
            )
            continue

        pairs[cockpit_id] = wheel

    return pairs

def _current_radio_ids():
    return {
        cockpit_id: cockpit_manager.get_cockpit(cockpit_id).radio_id
        for cockpit_id in range(1, cockpit_manager.max_cockpits + 1)
        if cockpit_manager.get_cockpit(cockpit_id) is not None
        and cockpit_manager.get_cockpit(cockpit_id).radio_id is not None
    }


def _stop_session_for_device_loss(cockpit_id, reason):
    if cockpit_sessions[cockpit_id]["active"]:
        stop_cockpit_session(cockpit_id)
    cockpit = cockpit_manager.get_cockpit(cockpit_id)
    if cockpit is not None and cockpit.vehicle_name is not None:
        # The radio may already have disappeared, so clear the logical state
        # directly if normal RadioManager cleanup is no longer possible.
        cockpit.vehicle_name = None
    print(f"[Hotplug] Cockpit {cockpit_id}: {reason}")

def get_active_rf_radio_ids():
    """
    Return RF Nano IDs currently being used by active RF sessions.

    Active radios must not be probed, reconnected, or used for
    receiver discovery during a Full Refresh.
    """
    active_radio_ids = set()

    with cockpit_sessions_lock:
        active_cockpit_ids = {
            cockpit_id
            for cockpit_id, session in cockpit_sessions.items()
            if session["active"]
        }

    for cockpit_id in active_cockpit_ids:
        cockpit = cockpit_manager.get_cockpit(cockpit_id)

        if cockpit is None:
            continue

        if cockpit.radio_id is None:
            continue

        active_radio_ids.add(
            str(cockpit.radio_id).strip().upper()
        )

    return active_radio_ids

def reconcile_cockpit_devices():
    """Rediscover steering wheels and keep registry car pairings.

    Vehicle/Nano/RF pairing is permanent in Vehicle Management.
    This path does NOT scan vehicles or RF receivers, and it does NOT
    require Nano presence for car assignment.
    """
    with cockpit_wheel_state_lock:
        old_wheel_ids = _current_wheel_identities()

        try:
            # Wheel-only discovery for cockpit Refresh / hotplug.
            wheels = cockpit_manager.wheel_manager.discover()

            print("[USB Groups] G29 discovery:")
            for wheel in wheels:
                port = _usb_parent_port_for_wheel(wheel)
                print(
                    f"  {wheel.path} | "
                    f"phys={wheel.phys} | "
                    f"USB_GROUP={port} | "
                    f"{'RF' if port == 4 else 'ESP' if port == 3 else 'UNKNOWN'}"
                )

            # Keep existing Nano serial handles alive if already open, but
            # do not treat Nano discovery as a UI/selection requirement.
            try:
                with radio_discovery_lock:
                    cockpit_manager.radio_manager.discover()
                    cockpit_manager.connect_radios()
            except Exception as radio_exc:
                print(f"[Discovery] Nano connect best-effort skipped: {radio_exc}")

            discovered_wheels = {
                _wheel_identity(wheel): wheel for wheel in wheels
            }

            used_wheels = set()

            for cockpit_id in range(1, cockpit_manager.max_cockpits + 1):
                cockpit = cockpit_manager.get_cockpit(cockpit_id)
                if cockpit is None:
                    continue

                old_wheel = old_wheel_ids.get(cockpit_id)
                if old_wheel is not None:
                    replacement = discovered_wheels.get(old_wheel)
                    if replacement is not None:
                        cockpit.wheel = replacement
                        used_wheels.add(old_wheel)
                    else:
                        if cockpit.wheel is not None:
                            _stop_session_for_device_loss(
                                cockpit_id, "wheel disconnected"
                            )
                        cockpit.wheel = None

                # Keep registry Nano pairing as-is. Never clear radio_id
                # just because serial presence changed.

            new_wheels = [
                wheel for identity, wheel in discovered_wheels.items()
                if identity not in used_wheels
            ]

            fixed_new_wheels = _fixed_cockpit_wheel_pairs(new_wheels)

            for cockpit_id, wheel in sorted(fixed_new_wheels.items()):
                cockpit = cockpit_manager.get_cockpit(cockpit_id)

                if cockpit is None or cockpit.wheel is not None:
                    continue

                index = wheels.index(wheel)

                try:
                    cockpit_manager.assign_wheel(cockpit_id, index)
                    print(
                        f"[Hotplug] Fixed G29 mapping: "
                        f"{wheel.phys} -> Cockpit {cockpit_id}"
                    )
                except Exception as exc:
                    print(
                        f"[Hotplug] Cockpit {cockpit_id} "
                        f"wheel assignment failed: {exc}"
                    )

            # Restore remembered cars from Vehicle Management pairings.
            # No vehicle/RX scan is performed.
            restore_preferred_car_assignments()

            for cockpit_id in range(1, cockpit_manager.max_cockpits + 1):
                cockpit = cockpit_manager.get_cockpit(cockpit_id)
                if (
                    cockpit is not None
                    and cockpit.wheel is not None
                    and cockpit.vehicle_name is not None
                ):
                    start_cockpit_control_worker(cockpit_id)

            return True, None

        except Exception as exc:
            print(f"[Discovery] Device reconciliation error: {exc}")
            for cockpit_id in range(1, cockpit_manager.max_cockpits + 1):
                cockpit = cockpit_manager.get_cockpit(cockpit_id)
                if (
                    cockpit is not None
                    and cockpit.wheel is not None
                    and cockpit.vehicle_name is not None
                ):
                    start_cockpit_control_worker(cockpit_id)
            return False, str(exc)

def refresh_cockpit_devices():
    """Manual refresh used by startup and the Refresh button (wheels only)."""
    print("[Discovery] Refreshing steering wheels...")
    return reconcile_cockpit_devices()

def refresh_vehicle_discovery():
    """
    Discover RF vehicles only.

    Does not scan wheels.
    Does not scan Nanos.
    Does not reconnect radios.
    Does not reconcile cockpit assignments.
    Does not stop control workers.
    Does not change active vehicle assignments.
    """

    with vehicle_refresh_lock:

        radio_manager = cockpit_manager.radio_manager

        skip_radio_ids = set()

        with cockpit_sessions_lock:
            active_cockpit_ids = {
                cockpit_id
                for cockpit_id, session in cockpit_sessions.items()
                if session["active"]
            }

        for cockpit_id in active_cockpit_ids:

            cockpit = cockpit_manager.get_cockpit(cockpit_id)

            if cockpit is None:
                continue

            if cockpit.radio_id is None:
                continue

            skip_radio_ids.add(
                str(cockpit.radio_id).strip().upper()
            )

        print("[Vehicle Discovery] Refresh requested")

        if skip_radio_ids:
            print(
                "[Vehicle Discovery] "
                f"Protected active radios: "
                f"{sorted(skip_radio_ids)}"
            )

        return radio_manager.discover_receivers(
            skip_radio_ids=skip_radio_ids
        )


def _scan_g29_identities():
    """Scan G29 identities without closing WheelManager control handles."""
    identities = set()
    g29_name = "Logitech G29 Driving Force Racing Wheel"

    for path in sorted(glob.glob("/dev/input/event*")):
        device = None
        try:
            device = evdev.InputDevice(path)
            if device.name != g29_name:
                continue
            identities.add(_wheel_identity(device))
        except Exception:
            pass
        finally:
            if device is not None:
                try:
                    device.close()
                except Exception:
                    pass

    return identities


def _scan_radio_ids():
    """Probe Nano identities, bypassing ports already in use by RadioManager."""
    ids = set()
    try:
        # First add IDs already held by the active RadioManager
        in_use_ports = set()
        for radio in cockpit_manager.radio_manager.radios.values():
            ids.add(str(radio.radio_id).strip())
            in_use_ports.add(radio.port)

        # Then manually probe ports that are NOT in use
        from radio_discovery import find_serial_ports, identify_radio
        for port in find_serial_ports():
            if port in in_use_ports:
                continue
            radio = identify_radio(port)
            if radio:
                ids.add(str(radio.radio_id).strip())
    except Exception as exc:
        print(f"[Hotplug] Nano probe error: {exc}")
    return ids


def cockpit_device_monitor_worker():
    """Detect G29/Nano plug or unplug changes without disturbing live handles."""
    previous_wheels = _scan_g29_identities()
    previous_radios = _scan_radio_ids()

    while not cockpit_device_monitor_stop.wait(COCKPIT_DEVICE_SCAN_INTERVAL):
        try:
            current_wheels = _scan_g29_identities()
            current_radios = _scan_radio_ids()

            if current_wheels != previous_wheels or current_radios != previous_radios:
                print(
                    f"[Hotplug] Device change detected: "
                    f"wheels {len(previous_wheels)} -> {len(current_wheels)}, "
                    f"Nanos {len(previous_radios)} -> {len(current_radios)}"
                )
                # A wheel move can land on either the RF or the ESP USB
                # group, so both device tables must be reconciled here.
                reconcile_cockpit_devices()
                reconcile_esp_devices()

                with esp_cockpits_lock:
                    ready_esp_cockpits = [
                        cockpit_id
                        for cockpit_id, cockpit in esp_cockpits.items()
                        if cockpit["wheel"] is not None
                        and cockpit["esp"] is not None
                    ]

                for cockpit_id in ready_esp_cockpits:
                    start_esp_cockpit_control_worker(cockpit_id)

                previous_wheels = _scan_g29_identities()
                previous_radios = _scan_radio_ids()

        except Exception as exc:
            print(f"[Hotplug] Monitor error: {exc}")


def start_cockpit_device_monitor():
    global cockpit_device_monitor_thread
    if (
        cockpit_device_monitor_thread is not None
        and cockpit_device_monitor_thread.is_alive()
    ):
        return
    cockpit_device_monitor_stop.clear()
    cockpit_device_monitor_thread = threading.Thread(
        target=cockpit_device_monitor_worker,
        name="cockpit-device-monitor",
        daemon=True
    )
    cockpit_device_monitor_thread.start()
    print("[Hotplug] Wheel/Nano auto-detection monitor started")


def stop_cockpit_device_monitor():
    global cockpit_device_monitor_thread

    cockpit_device_monitor_stop.set()

    thread = cockpit_device_monitor_thread

    if thread is not None:
        # The monitor can already be inside a Nano discovery operation.
        # Wait for it to fully exit before another process is allowed to
        # access the Nano serial ports.
        thread.join(timeout=15.0)

    cockpit_device_monitor_thread = None


def _clamp(value, low, high):
    return max(low, min(high, value))


def _steering_from_event(event_value, device=None):
    """Normalize G29 ABS_X using the proven standalone test mapping.

    Working Pi test formula:
        steering = int((event.value - 32768) * 500 / 32768)

    We keep the same center (32768) and return -1..+1 so the control
    worker can scale by COCKPIT_CONTROL_SCALE / sensitivity.
    """
    # G29 ABS_X is observed as 0..65535 with true center at 32768.
    # Do not use absinfo min/max normalization here — that path caused
    # false full-left (-1000) when startup absinfo was pinned at min.
    raw = float(event_value)
    normalized = (raw - 32768.0) / 32768.0
    return _clamp(normalized, -1.0, 1.0)


def _read_steering_axis(device):
    """Read current G29 steering from live absinfo, not only events."""
    try:
        info = device.absinfo(evdev.ecodes.ABS_X)
        if getattr(info, "value", None) is None:
            return None, None
        return float(info.value), _steering_from_event(info.value, device)
    except Exception:
        return None, None


def _pedal_from_event(event_value, device, code):
    """Normalize a G29 pedal axis to 0..1 pressed."""
    info = device.absinfo(code)
    minimum = float(info.min)
    maximum = float(info.max)

    if maximum <= minimum:
        return 0.0

    released_to_pressed = (
        (float(event_value) - minimum) / (maximum - minimum)
    )

    # G29 pedal axes observed in the standalone test use released ~= 1
    # and pressed ~= 0.
    return _clamp(1.0 - released_to_pressed, 0.0, 1.0)



def _esp_throttle_to_dac(value):
    value = _clamp(value, -1000, 1000)

    if value >= 0:
        return int(round(124 + (value / 1000.0) * (255 - 124)))

    return int(round(124 + (value / 1000.0) * 124))


def _esp_steering_to_dac(value):
    value = _clamp(value, -1000, 1000)

    if value >= 0:
        return int(round(128 + (value / 1000.0) * (255 - 128)))

    return int(round(128 + (value / 1000.0) * 128))

def _send_cockpit_zero(radio_id):
    try:
        cockpit_manager.radio_manager.send_control(
            radio_id,
            0,
            0
        )
    except Exception as exc:
        print(
            f"[Control] Zero command failed for radio "
            f"{radio_id}: {exc}"
        )


def cockpit_control_worker(cockpit_id):
    """
    Per-cockpit control path:
        G29 -> per-cockpit sensitivity -> RadioManager -> selected vehicle

    Each cockpit has its own wheel device and RadioManager radio. This worker
    owns only the wheel input device. RadioManager remains the owner of the
    Nano serial connection.
    """
    device = None
    ffb = None
    radio_id = None

    print(f"[Control] Cockpit {cockpit_id} control worker starting")

    try:
        initialize_cockpit_system()

        cockpit = cockpit_manager.get_cockpit(cockpit_id)
        if cockpit is None:
            print(f"[Control] Cockpit {cockpit_id} does not exist")
            return

        if cockpit.wheel is None:
            print(f"[Control] Cockpit {cockpit_id} has no wheel assigned")
            return

        # Resolve Nano from Vehicle Management pairing. Live serial presence
        # is not required to start the worker; send_control soft-fails offline.
        radio_id = cockpit.radio_id
        if cockpit.vehicle_name:
            paired_nano, _rx, _ctrl = _resolve_cockpit_pairing(cockpit)
            if paired_nano:
                radio_id = paired_nano

        if not radio_id and not cockpit.vehicle_name:
            print(f"[Control] Cockpit {cockpit_id} has no car/Nano pairing")
            return

        device = evdev.InputDevice(cockpit.wheel.path)

        # Exclusive grab helps keep FFB commands on this process and avoids
        # other readers interfering with spring force.
        try:
            device.grab()
            print(f"[Control] Cockpit {cockpit_id}: wheel grabbed for FFB/input")
        except Exception as grab_exc:
            print(
                f"[Control] Cockpit {cockpit_id}: wheel grab skipped "
                f"({grab_exc})"
            )

        # Bind FFB to this cockpit's exact G29 device. Do not scan for the
        # first EV_FF device and do not use a hardcoded event node.
        ffb = G29FFB(device)
        _set_cockpit_ffb_instance(cockpit_id, ffb)

        enabled = True
        with cockpit_settings_lock:
            if cockpit_id in cockpit_settings:
                enabled = cockpit_settings[cockpit_id].get(
                    "autocenter_enabled", True
                )

        ok, strength = ffb.apply_enabled(
            enabled,
            strength_pct=G29_AUTOCENTER_STRENGTH
        )

        print(
            f"[Control] Cockpit {cockpit_id}: "
            f"wheel={cockpit.wheel.name}, "
            f"device={cockpit.wheel.path}, "
            f"FFB=READY, "
            f"autocenter={'ON' if enabled else 'OFF'} "
            f"strength={strength} ok={ok}, "
            f"radio={radio_id}"
        )

        last_autocenter_refresh = time.monotonic()
        AUTOCENTER_REFRESH_INTERVAL = 2.0

        # Read the wheel's CURRENT physical state immediately. Do not assume
        # pedals are released just because the worker has started; evdev may
        # not generate a fresh ABS event until the pedal moves.
        #
        # Steering starts at 0 (center) until we see a trustworthy ABS_X
        # sample. Linux often reports ABS_X at min on open, which would
        # otherwise lock control at steering=-1000 forever.
        steering_axis = 0.0
        steering_raw = None
        steering_seen = False
        throttle_axis = 0.0
        brake_axis = 0.0
        real_brake_axis = 0.0
        last_steering_debug = 0.0
        STEERING_CENTER_DEADZONE = 0.03  # ignore tiny center noise

        # Match standalone test behavior: start at center, update from events.
        # Optional one-shot absinfo read is only accepted near center.
        raw0, norm0 = _read_steering_axis(device)
        if raw0 is not None and abs(norm0) < 0.20:
            steering_raw = raw0
            steering_axis = norm0
            steering_seen = True
        else:
            if raw0 is not None:
                print(
                    f"[Control] Cockpit {cockpit_id}: startup ABS_X "
                    f"raw={raw0:.0f} norm={norm0:.3f} ignored; "
                    f"using center until live motion"
                )

        try:
            throttle_info = device.absinfo(evdev.ecodes.ABS_Z)
            if getattr(throttle_info, "value", None) is not None:
                throttle_axis = _pedal_from_event(
                    throttle_info.value,
                    device,
                    evdev.ecodes.ABS_Z
                )
        except Exception:
            pass

        try:
            brake_info = device.absinfo(evdev.ecodes.ABS_Y)
            if getattr(brake_info, "value", None) is not None:
                brake_axis = _pedal_from_event(
                    brake_info.value,
                    device,
                    evdev.ecodes.ABS_Y
                )
        except Exception:
            pass

        try:
            real_brake_info = device.absinfo(evdev.ecodes.ABS_RZ)
            if getattr(real_brake_info, "value", None) is not None:
                real_brake_axis = _pedal_from_event(
                    real_brake_info.value,
                    device,
                    evdev.ecodes.ABS_RZ
                )
        except Exception:
            pass

        # Session arming: every new session starts disarmed. The pedals must
        # first be confirmed released before live throttle/brake commands can
        # reach the vehicle. This prevents stale G29 state from causing a
        # startup creep. If the operator starts a session while holding a
        # pedal, the vehicle remains stopped until the pedal is released.
        controls_armed = False
        session_was_active = False
        neutral_cycles = 0
        NEUTRAL_PEDAL_THRESHOLD = 0.05
        NEUTRAL_CONFIRM_CYCLES = 3
        last_send = 0.0

        while not cockpit_control_stop.is_set():
            now = time.monotonic()

            # Drain all currently available wheel events without blocking.
            readable, _, _ = select.select([device.fd], [], [], 0)

            if readable:
                for event in device.read():
                    if event.type != evdev.ecodes.EV_ABS:
                        continue

                    if event.code == evdev.ecodes.ABS_X:
                        steering_raw = float(event.value)
                        steering_axis = _steering_from_event(
                            event.value,
                            device
                        )
                        steering_seen = True

                    elif event.code == evdev.ecodes.ABS_Z:
                        throttle_axis = _pedal_from_event(
                            event.value,
                            device,
                            evdev.ecodes.ABS_Z
                        )

                    elif event.code == evdev.ecodes.ABS_Y:
                        brake_axis = _pedal_from_event(
                            event.value,
                            device,
                            evdev.ecodes.ABS_Y
                        )

                    elif event.code == evdev.ecodes.ABS_RZ:
                        real_brake_axis = _pedal_from_event(
                            event.value,
                            device,
                            evdev.ecodes.ABS_RZ
                        )

            # Always re-poll live absinfo for steering. Event-only reading
            # can stick on a stale extreme if ABS events are sparse.
            polled_raw, polled_norm = _read_steering_axis(device)
            if polled_raw is not None and polled_norm is not None:
                steering_raw = polled_raw
                if steering_seen or abs(polled_norm) < 0.95:
                    steering_axis = polled_norm
                    steering_seen = True

            if now - last_send < COCKPIT_CONTROL_INTERVAL:
                time.sleep(0.001)
                continue

            # Re-apply spring periodically. Some G29/hid-logitech paths drop
            # FF_AUTOCENTER after effects or idle periods.
            if (
                ffb is not None
                and now - last_autocenter_refresh >= AUTOCENTER_REFRESH_INTERVAL
            ):
                last_autocenter_refresh = now
                enabled_now = True
                with cockpit_settings_lock:
                    if cockpit_id in cockpit_settings:
                        enabled_now = cockpit_settings[cockpit_id].get(
                            "autocenter_enabled", True
                        )
                try:
                    ffb.apply_enabled(
                        enabled_now,
                        strength_pct=G29_AUTOCENTER_STRENGTH
                    )
                except Exception as ac_exc:
                    print(
                        f"[Control] Cockpit {cockpit_id} "
                        f"autocenter refresh error: {ac_exc}"
                    )

            session = get_cockpit_session_state(cockpit_id)
            cockpit = cockpit_manager.get_cockpit(cockpit_id)

            session_active_now = bool(session["active"])

            # A session transition always requires a fresh neutral-pedal
            # confirmation. This is intentionally independent of the normal
            # throttle deadzone so low-speed driving remains controllable.
            if not session_active_now:
                controls_armed = False
                session_was_active = False
                neutral_cycles = 0

            elif not session_was_active:
                session_was_active = True
                controls_armed = False
                neutral_cycles = 0

            registry_vehicle = None
            if cockpit is not None and cockpit.vehicle_name:
                _, registry_vehicle = (
                    cockpit_manager.radio_manager.vehicle_registry.find_by_name(
                        cockpit.vehicle_name
                    )
                )
                if registry_vehicle is not None:
                    paired_nano = str(
                        registry_vehicle.get("nano_id") or ""
                    ).strip()
                    if paired_nano:
                        radio_id = paired_nano
                        cockpit.radio_id = paired_nano

            if (
                not session_active_now
                or cockpit is None
                or cockpit.vehicle_name is None
                or not controls_armed
            ):
                steering = 0
                throttle = 0

                if (
                    session_active_now
                    and cockpit is not None
                    and cockpit.vehicle_name is not None
                ):
                    if (
                        abs(throttle_axis) <= NEUTRAL_PEDAL_THRESHOLD
                        and abs(brake_axis) <= NEUTRAL_PEDAL_THRESHOLD
                        and abs(real_brake_axis) <= NEUTRAL_PEDAL_THRESHOLD
                    ):
                        neutral_cycles += 1
                        if neutral_cycles >= NEUTRAL_CONFIRM_CYCLES:
                            controls_armed = True
                    else:
                        neutral_cycles = 0
            else:
                with cockpit_settings_lock:
                    settings = dict(cockpit_settings[cockpit_id])

                steering_sensitivity = (
                    settings["steering_sensitivity"] / 100.0
                )
                throttle_sensitivity = (
                    settings["throttle_sensitivity"] / 100.0
                )

                live_steering_axis = steering_axis if steering_seen else 0.0
                if abs(live_steering_axis) < STEERING_CENTER_DEADZONE:
                    live_steering_axis = 0.0

                steering = int(
                    _clamp(
                        live_steering_axis
                        * COCKPIT_CONTROL_SCALE
                        * steering_sensitivity,
                        -COCKPIT_CONTROL_SCALE,
                        COCKPIT_CONTROL_SCALE
                    )
                )

                # -------------------------------------------------
                # Vehicle steering direction from registry pairing
                # -------------------------------------------------
                if (
                    registry_vehicle is not None
                    and registry_vehicle.get("steering_reverse", False)
                ):
                    steering = -steering

                throttle = int(
                    _clamp(
                        (throttle_axis - brake_axis)
                        * COCKPIT_CONTROL_SCALE
                        * throttle_sensitivity,
                        -COCKPIT_CONTROL_SCALE,
                        COCKPIT_CONTROL_SCALE
                    )
                )

                if abs(throttle) < COCKPIT_CONTROL_DEADZONE:
                    throttle = 0
                
                if real_brake_axis > 0.05:
                    throttle = 0

                # Always print wheel axis mapping at 1 Hz while armed so we
                # can see raw ABS_X -> normalized -> command values.
                if now - last_steering_debug >= 1.0:
                    last_steering_debug = now
                    raw_txt = (
                        f"{steering_raw:.0f}"
                        if steering_raw is not None else "None"
                    )
                    print(
                        f"[Control] Cockpit {cockpit_id} AXIS "
                        f"raw={raw_txt} "
                        f"norm={live_steering_axis:+.3f} "
                        f"cmd_steer={steering:+d} "
                        f"cmd_thr={throttle:+d} "
                        f"sens={steering_sensitivity:.2f} "
                        f"seen={steering_seen} "
                        f"thr_axis={throttle_axis:.3f} "
                        f"brk_axis={brake_axis:.3f}"
                    )
            # -------------------------------------------------
            # Do not send RF control when:
            #   1. Session is inactive
            #   2. Cockpit does not exist
            #   3. No vehicle is assigned
            #   4. Controls are not yet armed
            #
            # RF SESSION_STOP already puts the vehicle into its
            # safe state, so there is no reason to send another
            # zero-control packet here.
            # -------------------------------------------------
            if (
                not session_active_now
                or cockpit is None
                or cockpit.vehicle_name is None
                or not controls_armed
                or not radio_id
            ):
                last_send = now
                time.sleep(COCKPIT_CONTROL_INTERVAL)
                continue

            ok = cockpit_manager.radio_manager.send_control(
                radio_id,
                steering,
                throttle
            )

            # Soft-fail only: keep streaming next frames even on TX_FAIL.
            last_send = now
            if not ok:
                time.sleep(0.01)

    except Exception as exc:
        print(f"[Control] Cockpit {cockpit_id} worker error: {exc}")
        if radio_id is not None:
            _send_cockpit_zero(radio_id)

    finally:
        if radio_id is not None:
            _send_cockpit_zero(radio_id)

        if ffb is not None:
            try:
                ffb.stop()
            except Exception:
                pass
            _remove_cockpit_ffb_instance(cockpit_id, expected=ffb)

        if device is not None:
            try:
                try:
                    device.ungrab()
                except Exception:
                    pass
                device.close()
            except Exception:
                pass

        print(f"[Control] Cockpit {cockpit_id} control worker stopped")

def esp_cockpit_control_worker(cockpit_id):
    """
    Per-ESP-cockpit control path:

        G29
         ↓
        session check
         ↓
        neutral pedal arming
         ↓
        sensitivity
         ↓
        ESP DAC mapping
         ↓
        USB serial
         ↓
        ESP32
    """
    print(f"[ESP{cockpit_id}] CONTROL WORKER ENTERED")
    ffb = None
    print(f"[ESP{cockpit_id}] Control worker starting")

    with esp_cockpits_lock:
        cockpit = esp_cockpits.get(cockpit_id)

        if not cockpit:
            print(f"[ESP{cockpit_id}] No cockpit configuration")
            return

        wheel = cockpit["wheel"]
        esp = cockpit["esp"]

    if wheel is None:
        print(f"[ESP{cockpit_id}] No wheel assigned")
        return

    if esp is None:
        print(f"[ESP{cockpit_id}] No ESP controller assigned")
        return

    try:
        wheel.grab()
    except Exception as e:
        print(f"[ESP{cockpit_id}] Failed to grab wheel: {e}")
        return

    # Bind FFB to this ESP cockpit's exact G29 device
    ffb = G29FFB(wheel)
    print(f"[ESP{cockpit_id}] G29FFB initialized")
    with esp_ffb_lock:
        esp_ffb_instances[cockpit_id] = ffb

    with esp_cockpit_settings_lock:
        autocenter_enabled = esp_cockpit_settings[cockpit_id].get(
            "autocenter_enabled", True
        )

    ok, strength = ffb.apply_enabled(
        autocenter_enabled,
        strength_pct=G29_AUTOCENTER_STRENGTH
    )
    print(
        f"[ESP{cockpit_id}] Hardware autocenter set: "
        f"{'ON' if autocenter_enabled else 'OFF'} "
        f"strength={strength} ok={ok}"
    )

    steering_axis = 0.0
    throttle_axis = 0.0
    brake_axis = 0.0
    real_brake_axis = 0.0

    controls_armed = False
    session_was_active = False
    neutral_cycles = 0

    NEUTRAL_PEDAL_THRESHOLD = 0.05
    NEUTRAL_CONFIRM_CYCLES = 3

    try:
        while not esp_control_stop.is_set():

            # -------------------------------------------------
            # Read available G29 events
            # -------------------------------------------------
            ready, _, _ = select.select([wheel.fd], [], [], 0)

            if ready:
                for event in wheel.read():

                    if event.type != evdev.ecodes.EV_ABS:
                        continue

                    if event.code == evdev.ecodes.ABS_X:
                        steering_axis = _steering_from_event(
                            event.value,
                            wheel
                        )

                    elif event.code == evdev.ecodes.ABS_Z:
                        throttle_axis = _pedal_from_event(
                            event.value,
                            wheel,
                            evdev.ecodes.ABS_Z
                        )

                    elif event.code == evdev.ecodes.ABS_Y:
                        brake_axis = _pedal_from_event(
                            event.value,
                            wheel,
                            evdev.ecodes.ABS_Y
                        )

                    elif event.code == evdev.ecodes.ABS_RZ:
                        real_brake_axis = _pedal_from_event(
                            event.value,
                            wheel,
                            evdev.ecodes.ABS_RZ
                        )

            # -------------------------------------------------
            # Check session state
            # -------------------------------------------------

            with esp_cockpit_sessions_lock:
                session = esp_cockpit_sessions[cockpit_id]
                session_active = session["active"]
                session_ends_at = session["ends_at"]

            # Check automatic session expiry
            if (
                session_active
                and session_ends_at is not None
                and time.time() >= session_ends_at
            ):
                with esp_cockpit_sessions_lock:
                    esp_cockpit_sessions[cockpit_id] = {
                        "active": False,
                        "started_at": None,
                        "ends_at": None,
                    }

                print(f"[ESP{cockpit_id}] SESSION_EXPIRED")

                session_active = False

            if not session_active:

                controls_armed = False
                session_was_active = False
                neutral_cycles = 0

                esp.send_control(
                    _esp_throttle_to_dac(0),
                    _esp_steering_to_dac(0)
                )

                time.sleep(COCKPIT_CONTROL_INTERVAL)
                continue

            # -------------------------------------------------
            # New session → reset arming
            # -------------------------------------------------
            if not session_was_active:

                controls_armed = False
                neutral_cycles = 0
                session_was_active = True

            # -------------------------------------------------
            # Pedal neutral arming
            # -------------------------------------------------
            if not controls_armed:

                pedals_neutral = (
                    abs(throttle_axis) <= NEUTRAL_PEDAL_THRESHOLD
                    and abs(brake_axis) <= NEUTRAL_PEDAL_THRESHOLD
                    and abs(real_brake_axis) <= NEUTRAL_PEDAL_THRESHOLD
                )

                if pedals_neutral:
                    neutral_cycles += 1
                else:
                    neutral_cycles = 0

                if neutral_cycles >= NEUTRAL_CONFIRM_CYCLES:
                    controls_armed = True

                # Stay neutral until successfully armed
                esp.send_control(
                    _esp_throttle_to_dac(0),
                    _esp_steering_to_dac(0)
                )

                time.sleep(COCKPIT_CONTROL_INTERVAL)
                continue

            # -------------------------------------------------
            # Get cockpit sensitivity settings
            # -------------------------------------------------
            with esp_cockpit_settings_lock:
                settings = dict(
                    esp_cockpit_settings[cockpit_id]
                )

            steering_sensitivity = (
                settings["steering_sensitivity"] / 100.0
            )

            throttle_sensitivity = (
                settings["throttle_sensitivity"] / 100.0
            )

            # -------------------------------------------------
            # Steering
            # Same calculation as RF cockpit
            # -------------------------------------------------
            steering = int(_clamp(
                steering_axis
                * COCKPIT_CONTROL_SCALE
                * steering_sensitivity,
                -COCKPIT_CONTROL_SCALE,
                COCKPIT_CONTROL_SCALE
            ))

            # -------------------------------------------------
            # Throttle
            # Same calculation as RF cockpit
            # -------------------------------------------------
            throttle = int(_clamp(
                (throttle_axis - brake_axis)
                * COCKPIT_CONTROL_SCALE
                * throttle_sensitivity,
                -COCKPIT_CONTROL_SCALE,
                COCKPIT_CONTROL_SCALE
            ))

            # -------------------------------------------------
            # Deadzone
            # -------------------------------------------------
            if abs(throttle) < COCKPIT_CONTROL_DEADZONE:
                throttle = 0

            # -------------------------------------------------
            # Real brake safety
            # -------------------------------------------------
            if real_brake_axis > 0.05:
                throttle = 0

            # -------------------------------------------------
            # Convert to ESP DAC values and send
            # -------------------------------------------------
            esp.send_control(
                _esp_throttle_to_dac(throttle),
                _esp_steering_to_dac(steering)
            )

            time.sleep(COCKPIT_CONTROL_INTERVAL)

    except Exception as e:
        print(f"[ESP{cockpit_id}] Control worker error: {e}")

    finally:

        try:
            ffb.stop()
        except Exception:
            pass

        with esp_ffb_lock:
            if esp_ffb_instances.get(cockpit_id) is ffb:
                esp_ffb_instances.pop(cockpit_id, None)

        # Always return ESP to neutral on worker exit
        try:
            esp.send_control(
                _esp_throttle_to_dac(0),
                _esp_steering_to_dac(0)
            )
        except Exception:
            pass

        try:
            wheel.ungrab()
        except Exception:
            pass

        print(f"[ESP{cockpit_id}] Control worker stopped")

def start_esp_cockpit_control_worker(cockpit_id):
    print(f"[ESP{cockpit_id}] START WORKER REQUEST")

    existing_thread = esp_control_threads.get(cockpit_id)

    if existing_thread is not None:
        if existing_thread.is_alive():
            print(f"[ESP{cockpit_id}] WORKER ALREADY RUNNING")
            return

        # Old thread has finished.
        # A Python Thread object cannot be started again.
        print(f"[ESP{cockpit_id}] Removing old worker thread")
        esp_control_threads.pop(cockpit_id, None)

    esp_control_stop.clear()

    thread = threading.Thread(
        target=esp_cockpit_control_worker,
        args=(cockpit_id,),
        name=f"esp{cockpit_id}-control",
        daemon=True
    )

    esp_control_threads[cockpit_id] = thread

    print(f"[ESP{cockpit_id}] STARTING THREAD")
    thread.start()
    print(f"[ESP{cockpit_id}] THREAD STARTED")

def stop_esp_cockpit_control_workers():
    esp_control_stop.set()

    for thread in esp_control_threads.values():
        if thread is not None:
            thread.join(timeout=2.0)

    esp_control_threads.clear()

def start_cockpit_control_worker(cockpit_id):
    if cockpit_id in cockpit_control_threads:
        thread = cockpit_control_threads[cockpit_id]
        if thread.is_alive():
            return

    cockpit_control_stop.clear()
    thread = threading.Thread(
        target=cockpit_control_worker,
        args=(cockpit_id,),
        name=f"cockpit{cockpit_id}-control",
        daemon=True
    )
    cockpit_control_threads[cockpit_id] = thread
    thread.start()


def stop_cockpit_control_workers():
    cockpit_control_stop.set()

    for thread in cockpit_control_threads.values():
        if thread is not None:
            thread.join(timeout=2.0)

    cockpit_control_threads.clear()

    with cockpit_ffb_lock:
        remaining_ffb = list(cockpit_ffb_instances.items())
        cockpit_ffb_instances.clear()

    for cockpit_id, ffb in remaining_ffb:
        try:
            ffb.stop()
        except Exception:
            pass

    global g29_ffb_instance
    g29_ffb_instance = None


def get_cockpit_session_state(cockpit_id):
    """Return the current session state for one cockpit."""
    now = time.time()

    with cockpit_sessions_lock:
        session = cockpit_sessions[cockpit_id]

        if session["active"] and session["ends_at"] is not None:
            remaining = max(0, int(session["ends_at"] - now))
        else:
            remaining = 0

        return {
            "active": bool(session["active"]),
            "remaining_seconds": remaining,
        }


def _resolve_cockpit_pairing(cockpit):
    """Resolve Nano ID + RX ID from Vehicle Management pairing.

    Live Nano serial presence is optional. Returns
    (nano_id, receiver_id, controller_or_none).
    """
    if cockpit is None:
        return None, None, None

    nano_id = str(cockpit.radio_id or "").strip() or None
    receiver_id = None
    vehicle_name = str(cockpit.vehicle_name or "").strip() or None

    if vehicle_name:
        rid, vehicle = cockpit_manager.radio_manager.vehicle_registry.find_by_name(
            vehicle_name
        )
        if vehicle is not None:
            receiver_id = str(rid or "").strip().upper() or None
            paired_nano = str(vehicle.get("nano_id") or "").strip() or None
            if paired_nano:
                nano_id = paired_nano
                cockpit.radio_id = paired_nano

    controller = (
        cockpit_manager.radio_manager.get_radio(nano_id)
        if nano_id else None
    )

    if controller is not None:
        selected_rx = getattr(controller, "selected_receiver_id", None) or getattr(
            controller, "receiver_id", None
        )
        if selected_rx:
            receiver_id = str(selected_rx).strip().upper()
        elif receiver_id:
            controller.receiver_id = receiver_id
            controller.selected_receiver_id = receiver_id
            if vehicle_name and not controller.vehicle_name:
                controller.vehicle_name = vehicle_name

    return nano_id, receiver_id, controller


def _send_cockpit_session_start(cockpit, duration_seconds):
    """Send SESSION_START using registry Nano→RF pairing when possible."""
    nano_id, selected_rx, controller = _resolve_cockpit_pairing(cockpit)

    if cockpit is None or not nano_id:
        return False, "No Nano paired for selected car"

    if not selected_rx:
        return False, "Selected car has no RF receiver ID in Vehicle Management"

    try:
        duration_seconds = int(duration_seconds)
        if duration_seconds <= 0:
            return False, "Session duration must be greater than zero"

        # Best-effort hardware notify. If Nano serial is offline, still allow
        # logical session start so steering can flow once Nano is present.
        if controller is None or not getattr(controller, "connected", False):
            print(
                f"[Session] Nano {nano_id} offline; starting logical session "
                f"for RX {selected_rx} without RF SESSION_START"
            )
            return True, None

        send_session_start = getattr(controller, "send_session_start", None)
        if callable(send_session_start):
            ok = send_session_start(duration_seconds)
        else:
            ok = controller._send_command(
                f"SESSION_START,{duration_seconds}"
            )

        if not ok:
            print(
                f"[Session] RF SESSION_START failed for {selected_rx}; "
                f"continuing with logical session"
            )
            return True, None

        print(
            f"[Session] RF SESSION_START sent: "
            f"Cockpit {getattr(cockpit, 'cockpit_id', '?')} "
            f"-> {selected_rx}, {duration_seconds}s"
        )
        return True, None
    except Exception as exc:
        print(f"[Session] RF session start error: {exc}")
        return True, None


def _send_cockpit_session_stop(cockpit):
    """Send SESSION_STOP using registry Nano→RF pairing when possible."""
    nano_id, selected_rx, controller = _resolve_cockpit_pairing(cockpit)

    if cockpit is None or not nano_id:
        return True, None

    if controller is None or not getattr(controller, "connected", False):
        return True, None

    if not selected_rx:
        return True, None

    try:
        send_session_stop = getattr(controller, "send_session_stop", None)
        if callable(send_session_stop):
            ok = send_session_stop()
        else:
            ok = controller._send_command("SESSION_STOP")

        if not ok:
            return False, "Failed to send session stop to RX"

        print(
            f"[Session] RF SESSION_STOP sent: "
            f"Cockpit {cockpit.cockpit_id if hasattr(cockpit, 'cockpit_id') else '?'} "
            f"-> {selected_rx}"
        )
        return True, None
    except Exception as exc:
        print(f"[Session] RF session stop error: {exc}")
        return False, str(exc)


def expire_cockpit_session(cockpit_id):
    """Stop a cockpit session, stop the RX session, then release the vehicle."""
    cockpit = cockpit_manager.get_cockpit(cockpit_id)

    # RF stop is deliberately sent before releasing the logical assignment.
    # The RX remains authoritative even if the Pi-side timer fires first.
    if cockpit is not None:
        stop_ok, stop_error = _send_cockpit_session_stop(cockpit)
        if not stop_ok:
            print(
                f"[Session] Cockpit {cockpit_id} RX stop warning: {stop_error}"
            )

    with cockpit_sessions_lock:
        session = cockpit_sessions[cockpit_id]
        if not session["active"]:
            return
        session["active"] = False
        session["started_at"] = None
        session["ends_at"] = None

    sync_telemetry_receiver()
    
    # Clear lap timing state for the vehicle being released.
    if cockpit is not None and cockpit.vehicle_name is not None:
        active_vehicle = cockpit_manager.radio_manager.get_active_vehicle(
            cockpit.vehicle_name
        )

        if active_vehicle is not None:
            receiver_id = str(
                active_vehicle.get("receiver_id", "")
            ).strip().upper()

            if receiver_id:
                lap_timer_vehicle_state.pop(receiver_id, None)

    with cockpit_driver_lock:
        cockpit_drivers[cockpit_id] = {
            "user_id": None,
            "name": None,
            "phone": None,
        }


    try:
        if cockpit is not None and cockpit.vehicle_name is not None:
            success = cockpit_manager.clear_nano(cockpit_id)
            if success:
                print(
                    f"[Session] Cockpit {cockpit_id} expired; vehicle released"
                )
            else:
                print(
                    f"[Session] Cockpit {cockpit_id} expired; vehicle release failed"
                )
        else:
            print(
                f"[Session] Cockpit {cockpit_id} expired; no vehicle to release"
            )
    except Exception as exc:
        print(
            f"[Session] Cockpit {cockpit_id} expiry cleanup error: {exc}"
        )


def cockpit_session_worker():
    """Background Pi-owned timer for all logical cockpits."""
    while True:
        now = time.time()
        expired = []

        with cockpit_sessions_lock:
            for cockpit_id, session in cockpit_sessions.items():
                if (
                    session["active"]
                    and session["ends_at"] is not None
                    and now >= session["ends_at"]
                ):
                    expired.append(cockpit_id)

        for cockpit_id in expired:
            expire_cockpit_session(cockpit_id)

        time.sleep(0.25)


def start_cockpit_session(cockpit_id):
    """Start a timed cockpit session only after RF SESSION_START succeeds."""
    initialize_cockpit_system()

    cockpit = cockpit_manager.get_cockpit(cockpit_id)
    if cockpit is None:
        return False, "Invalid cockpit ID"

    if cockpit.wheel is None:
        return False, f"Cockpit {cockpit_id} has no wheel assigned"

    # If the previous session released the car, restore the cockpit's
    # persistent preference from Vehicle Management pairing.
    if cockpit.vehicle_name is None:
        preferred = _get_preferred_car(cockpit_id)
        if preferred:
            try:
                if cockpit_manager.select_vehicle_by_registry(cockpit_id, preferred):
                    print(
                        f"[Car Preference] Session start restored Cockpit "
                        f"{cockpit_id} -> {preferred}"
                    )
            except Exception as exc:
                print(
                    f"[Car Preference] Session restore error for Cockpit "
                    f"{cockpit_id}: {exc}"
                )

    if cockpit.vehicle_name is None:
        return False, f"Cockpit {cockpit_id} has no vehicle assigned"

    nano_id, receiver_id, _controller = _resolve_cockpit_pairing(cockpit)
    if not nano_id:
        return False, (
            f"Cockpit {cockpit_id} car has no Nano pairing in Vehicle Management"
        )
    if not receiver_id:
        return False, (
            f"Cockpit {cockpit_id} car has no RF receiver ID in Vehicle Management"
        )

    with cockpit_settings_lock:
        duration_minutes = cockpit_settings[cockpit_id][
            "session_duration_minutes"
        ]

    with cockpit_driver_lock:
        driver = dict(cockpit_drivers[cockpit_id])

    if not driver["user_id"]:
        return False, f"Cockpit {cockpit_id} has no driver selected"

    # Verify the driver still exists in the database.
    user = get_user_by_id(driver["user_id"])

    if user is None:
        with cockpit_driver_lock:
            cockpit_drivers[cockpit_id] = {
                "user_id": None,
                "name": None,
                "phone": None,
            }

        return False, f"Cockpit {cockpit_id} driver no longer exists"

    with cockpit_sessions_lock:
        if cockpit_sessions[cockpit_id]["active"]:
            return False, f"Cockpit {cockpit_id} session is already active"

    now = time.time()
    duration_seconds = int(duration_minutes * 60)
    ends_at = now + duration_seconds

    # CRITICAL: send RF session start first. Do not mark the Pi session active
    # if the Nano/RX did not receive the command successfully.
    session_ok, session_error = _send_cockpit_session_start(
        cockpit,
        duration_seconds
    )
    if not session_ok:
        print(
            f"[Session] Cockpit {cockpit_id} NOT started locally: "
            f"{session_error}"
        )
        return False, session_error

    with cockpit_sessions_lock:
        cockpit_sessions[cockpit_id] = {
            "active": True,
            "started_at": now,
            "ends_at": ends_at,
        }

    # A stop/reselect cycle (or a worker that exited on wheel loss) may have
    # left no worker running for this cockpit. Ensure one is alive before
    # the operator expects the vehicle to respond.
    start_cockpit_control_worker(cockpit_id)

    sync_telemetry_receiver()

    # Reset previous lap timing state for this vehicle.
    active_vehicle = cockpit_manager.radio_manager.get_active_vehicle(
        cockpit.vehicle_name
    )

    if active_vehicle is not None:
        receiver_id = str(
            active_vehicle.get("receiver_id", "")
        ).strip().upper()

        if receiver_id:
            lap_timer_vehicle_state.pop(receiver_id, None)

    update_user_last_used(user["user_id"])

    with cockpit_driver_lock:
        cockpit_drivers[cockpit_id] = {
            "user_id": user["user_id"],
            "name": user["name"],
            "phone": user["phone"],
        }

    print(
        f"[Session] Cockpit {cockpit_id} started: "
        f"{duration_minutes} minute(s), "
        f"vehicle={cockpit.vehicle_name}, "
        f"driver={user['name']}"
    )

    return True, None


def stop_cockpit_session(cockpit_id):
    """Stop RF session first, then clear Pi session and release the vehicle."""
    initialize_cockpit_system()

    cockpit = cockpit_manager.get_cockpit(cockpit_id)
    if cockpit is None:
        return False, "Invalid cockpit ID"

    with cockpit_sessions_lock:
        was_active = cockpit_sessions[cockpit_id]["active"]

    # If a session is active, stop the authoritative RX session before
    # releasing the vehicle assignment.
    if was_active:
        stop_ok, stop_error = _send_cockpit_session_stop(cockpit)
        if not stop_ok:
            print(
                f"[Session] Cockpit {cockpit_id} RX stop warning: {stop_error}"
            )

    # Clear lap timing state for the vehicle being released.
    if cockpit.vehicle_name is not None:
        active_vehicle = cockpit_manager.radio_manager.get_active_vehicle(
            cockpit.vehicle_name
        )

        if active_vehicle is not None:
            receiver_id = str(
                active_vehicle.get("receiver_id", "")
            ).strip().upper()

            if receiver_id:
                lap_timer_vehicle_state.pop(receiver_id, None)

    with cockpit_sessions_lock:
        cockpit_sessions[cockpit_id] = {
            "active": False,
            "started_at": None,
            "ends_at": None,
        }

    with cockpit_driver_lock:
        cockpit_drivers[cockpit_id] = {
            "user_id": None,
            "name": None,
            "phone": None,
        }

    sync_telemetry_receiver()

    if cockpit.vehicle_name is not None:
        try:
            if not cockpit_manager.clear_nano(cockpit_id):
                return False, "Vehicle release failed"
        except Exception as exc:
            return False, str(exc)

    print(f"[Session] Cockpit {cockpit_id} stopped")
    return True, None



def get_cockpit_payload():
    """Return UI-safe logical cockpit and car list from Vehicle Management."""
    initialize_cockpit_system()

    # Cars come only from Vehicle Management. The cockpit UI picks a car
    # name; Nano ID + RX RF ID are resolved from the registry pairing.
    # No Nano/radio presence check is exposed to the UI.
    vehicles = []
    for receiver_id, vehicle in cockpit_manager.radio_manager.vehicle_registry.all().items():
        if not isinstance(vehicle, dict):
            continue

        name = str(vehicle.get("name", "")).strip()
        if not name:
            continue

        vehicle_nano_id = str(vehicle.get("nano_id") or "").strip() or None

        vehicles.append({
            "name": name,
            "receiver_id": str(receiver_id).strip().upper(),
            "nano_id": vehicle_nano_id,
            # Always selectable in UI once configured. Live Nano serial
            # presence is irrelevant for car selection.
            "available": bool(vehicle_nano_id)
        })

    vehicles.sort(key=lambda item: item["name"])

    cockpits = []

    with cockpit_settings_lock:
        settings_snapshot = {
            cockpit_id: dict(settings)
            for cockpit_id, settings in cockpit_settings.items()
        }

    for cockpit_id in range(1, cockpit_manager.max_cockpits + 1):
        cockpit = cockpit_manager.get_cockpit(cockpit_id)

        wheel = None
        if cockpit.wheel is not None:
            wheel = {
                "name": cockpit.wheel.name,
                "path": cockpit.wheel.path,
                "phys": cockpit.wheel.phys
            }

        with cockpit_driver_lock:
            driver_snapshot = dict(cockpit_drivers[cockpit_id])

        cockpits.append({
            "cockpit_id": cockpit_id,
            "wheel": wheel,
            # radio/nano kept null in payload: UI is car-only.
            "radio": None,
            "nano": None,
            "vehicle": cockpit.vehicle_name,
            "paired_nano_id": cockpit.radio_id,
            "driver": driver_snapshot,
            "settings": settings_snapshot[cockpit_id],
            "session": {
                **get_cockpit_session_state(cockpit_id),
                "duration_minutes": settings_snapshot[cockpit_id][
                    "session_duration_minutes"
                ],
            },
        })

    return {
        "success": True,
        "cockpits": cockpits,
        "vehicles": vehicles,
        # Empty pools retained for older clients; UI no longer renders them.
        "nanos": [],
        "radios": []
    }


@app.route("/api/vehicles/refresh", methods=["POST"])
def vehicle_refresh():
    try:
        refresh_vehicle_discovery()

        return jsonify(
            get_cockpit_payload()
        )

    except Exception as exc:
        print(
            f"[Vehicle Discovery] Refresh error: {exc}"
        )

        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500


@app.route("/api/vehicles/sync-nanos", methods=["POST"])
def vehicle_sync_nanos_api():
    """Discover connected Nanos, query PAIRING, upsert vehicle registry."""
    try:
        ok, summary = sync_vehicles_from_connected_nanos()
        if not ok:
            return jsonify({
                "success": False,
                "error": summary.get("error", "Nano pairing sync failed")
            }), 500

        registry = cockpit_manager.radio_manager.vehicle_registry
        vehicles = []
        for receiver_id, vehicle in registry.all().items():
            if not isinstance(vehicle, dict):
                continue
            vehicles.append({
                "receiver_id": str(receiver_id).strip().upper(),
                "name": str(vehicle.get("name", "")).strip(),
                "nano_id": vehicle.get("nano_id"),
                "steering_reverse": bool(vehicle.get("steering_reverse", False)),
                "transponder_id": vehicle.get("transponder_id")
            })
        vehicles.sort(key=lambda item: item["name"])

        return jsonify({
            "success": True,
            "created": summary.get("created", 0),
            "updated": summary.get("updated", 0),
            "unchanged": summary.get("unchanged", 0),
            "failed": summary.get("failed", 0),
            "pairings": summary.get("pairings", []),
            "vehicles": vehicles
        })

    except Exception as exc:
        print(f"[Vehicle Sync] API error: {exc}")
        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route('/api/vehicles', methods=['GET'])
def vehicle_management_api():
    try:
        registry = cockpit_manager.radio_manager.vehicle_registry

        vehicles = []

        for receiver_id, vehicle in registry.all().items():
            if not isinstance(vehicle, dict):
                continue

            vehicles.append({
                "receiver_id": str(receiver_id).strip().upper(),
                "name": str(vehicle.get("name", "")).strip(),
                "nano_id": vehicle.get("nano_id"),
                "steering_reverse": bool(
                    vehicle.get("steering_reverse", False)
                ),
                "transponder_id": vehicle.get("transponder_id")
            })

        vehicles.sort(key=lambda item: item["name"])

        with esp_vehicle_lock:
            esp_vehicle_list = [
                {
                    "cockpit_id": cockpit_id,
                    "name": vehicle["name"],
                    "transponder_id": vehicle["transponder_id"],
                    "type": "esp"
                }
                for cockpit_id, vehicle in esp_vehicles.items()
            ]

        return jsonify({
            "success": True,
            "vehicles": vehicles,
            "esp_vehicles": esp_vehicle_list
        })

    except Exception as exc:
        print(f"[Vehicle Management] GET error: {exc}")
        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500


@app.route('/api/vehicles', methods=['POST'])
def vehicle_management_create_api():
    """Create a new RF vehicle: ESP MAC (receiver_id), name, paired Nano,
    and transponder ID are all entered directly by the operator here."""
    try:
        data = request.get_json() or {}

        receiver_id = str(data.get("receiver_id", "")).strip().upper()
        name = str(data.get("name", "")).strip()
        nano_id = data.get("nano_id")
        transponder_id = data.get("transponder_id")
        steering_reverse = bool(data.get("steering_reverse", False))

        if not receiver_id:
            return jsonify({
                "success": False,
                "error": "ESP MAC (receiver_id) cannot be empty"
            }), 400

        if not name:
            return jsonify({
                "success": False,
                "error": "Vehicle name cannot be empty"
            }), 400

        registry = cockpit_manager.radio_manager.vehicle_registry

        vehicle = registry.create_vehicle(
            receiver_id,
            name,
            nano_id=nano_id,
            transponder_id=transponder_id,
            steering_reverse=steering_reverse
        )

        return jsonify({
            "success": True,
            "vehicle": {
                "receiver_id": receiver_id,
                "name": vehicle["name"],
                "nano_id": vehicle.get("nano_id"),
                "steering_reverse": vehicle["steering_reverse"],
                "transponder_id": vehicle["transponder_id"]
            }
        })

    except ValueError as exc:
        return jsonify({"success": False, "error": str(exc)}), 400
    except Exception as exc:
        print(f"[Vehicle Management] CREATE error: {exc}")
        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500


@app.route('/api/vehicles/<receiver_id>', methods=['DELETE'])
def vehicle_management_delete_api(receiver_id):
    try:
        receiver_id = str(receiver_id).strip().upper()
        registry = cockpit_manager.radio_manager.vehicle_registry

        if not registry.has_receiver(receiver_id):
            return jsonify({
                "success": False,
                "error": "Vehicle not found"
            }), 404

        vehicle = registry.get_vehicle(receiver_id)
        vehicle_name = str((vehicle or {}).get("name", "")).strip()

        # Release the vehicle from any cockpit currently using it so the UI
        # never keeps pointing at a deleted car.
        for cockpit_id in range(1, cockpit_manager.max_cockpits + 1):
            cockpit = cockpit_manager.get_cockpit(cockpit_id)
            if cockpit is not None and cockpit.vehicle_name == vehicle_name:
                try:
                    cockpit_manager.clear_nano(cockpit_id)
                except Exception:
                    pass
                _set_preferred_car(cockpit_id, None)

        registry.remove(receiver_id)

        return jsonify({"success": True})

    except Exception as exc:
        print(f"[Vehicle Management] DELETE error: {exc}")
        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route("/api/esp-vehicles", methods=["GET"])
def get_esp_vehicles():
    with esp_vehicle_lock:
        vehicles = [
            {
                "car_id": cockpit_id,
                "name": vehicle["name"],
                "transponder_id": vehicle["transponder_id"],
                "type": "esp"
            }
            for cockpit_id, vehicle in esp_vehicles.items()
        ]

    return jsonify({
        "success": True,
        "vehicles": vehicles
    })


@app.route('/api/esp-vehicles', methods=['GET'])
def esp_vehicle_list_api():

    with esp_vehicle_lock:
        vehicles = [
            {
                "car_id": cockpit_id,
                "name": vehicle["name"],
                "transponder_id": vehicle["transponder_id"]
            }
            for cockpit_id, vehicle in esp_vehicles.items()
        ]

    return jsonify({
        "success": True,
        "vehicles": vehicles
    })

@app.route(
    "/api/esp-vehicles/<int:cockpit_id>/settings",
    methods=["POST"]
)
def esp_vehicle_management_settings_api(cockpit_id):

    if cockpit_id not in esp_vehicles:
        return jsonify({
            "success": False,
            "error": "Invalid ESP vehicle"
        }), 400

    data = request.get_json() or {}

    name = str(
        data.get("name", "")
    ).strip()

    if not name:
        return jsonify({
            "success": False,
            "error": "Vehicle name cannot be empty"
        }), 400

    transponder_id = data.get("transponder_id")

    if (
        transponder_id is not None
        and str(transponder_id).strip() != ""
    ):
        try:
            transponder_id = int(transponder_id)
        except (TypeError, ValueError):
            return jsonify({
                "success": False,
                "error": "Transponder ID must be an integer"
            }), 400
    else:
        transponder_id = None

    with esp_vehicle_lock:

        for other_id, vehicle in esp_vehicles.items():

            if other_id == cockpit_id:
                continue

            if (
                transponder_id is not None
                and vehicle.get("transponder_id")
                == transponder_id
            ):
                return jsonify({
                    "success": False,
                    "error": (
                        f"Transponder ID already assigned "
                        f"to Car {other_id}"
                    )
                }), 400

        esp_vehicles[cockpit_id] = {
            "name": name,
            "transponder_id": transponder_id
        }

    save_esp_vehicles()

    return jsonify({
        "success": True,
        "vehicle": {
            "cockpit_id": cockpit_id,
            "name": name,
            "transponder_id": transponder_id,
            "type": "esp"
        }
    })

@app.route('/api/vehicles/<receiver_id>/settings', methods=['POST'])
def vehicle_management_settings_api(receiver_id):
    try:
        receiver_id = str(receiver_id).strip().upper()

        registry = cockpit_manager.radio_manager.vehicle_registry

        if not registry.has_receiver(receiver_id):
            return jsonify({
                "success": False,
                "error": "Vehicle not found"
            }), 404

        data = request.get_json() or {}

        if "name" in data:
            name = str(data["name"]).strip()

            if not name:
                return jsonify({
                    "success": False,
                    "error": "Vehicle name cannot be empty"
                }), 400

            registry.set_name(receiver_id, name)

        if "steering_reverse" in data:
            registry.set_steering_reverse(
                receiver_id,
                bool(data["steering_reverse"])
            )

        if "transponder_id" in data:
            registry.set_transponder_id(
                receiver_id,
                data["transponder_id"]
            )

        if "nano_id" in data:
            registry.set_nano_id(
                receiver_id,
                data["nano_id"]
            )

        vehicle = registry.get_vehicle(receiver_id)

        return jsonify({
            "success": True,
            "vehicle": {
                "receiver_id": receiver_id,
                "name": str(vehicle.get("name", "")).strip(),
                "nano_id": vehicle.get("nano_id"),
                "steering_reverse": bool(
                    vehicle.get("steering_reverse", False)
                ),
                "transponder_id": vehicle.get("transponder_id")
            }
        })

    except Exception as exc:
        print(f"[Vehicle Management] SAVE error: {exc}")
        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route('/api/cockpits/refresh', methods=['POST'])
def cockpit_refresh():
    """Refresh steering wheels only.

    Vehicle/Nano/RF pairing is permanent in Vehicle Management and is not
    rescanned from this endpoint.
    """
    try:
        scan_ok, scan_reason = refresh_cockpit_devices()
        reconcile_esp_devices()

        # ESP control workers are otherwise only started once at Flask boot;
        # restart any that are missing now that hardware was rescanned.
        with esp_cockpits_lock:
            ready_esp_cockpits = [
                cockpit_id
                for cockpit_id, cockpit in esp_cockpits.items()
                if cockpit["wheel"] is not None and cockpit["esp"] is not None
            ]

        for cockpit_id in ready_esp_cockpits:
            start_esp_cockpit_control_worker(cockpit_id)

        payload = get_cockpit_payload()

        if not scan_ok:
            payload["warning"] = f"Wheel refresh error: {scan_reason}"

        return jsonify(payload)

    except Exception as e:
        print(f"[Cockpit API] Refresh error: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500



@app.route('/api/cockpits', methods=['GET'])
def cockpit_status():
    try:
        return jsonify(get_cockpit_payload())
    except Exception as e:
        print(f"[Cockpit API] Status error: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500


@app.route('/api/cockpits/<int:cockpit_id>/vehicle', methods=['POST'])
def set_cockpit_vehicle(cockpit_id):
    """Assign a car to a cockpit.

    The operator only picks a car name. Its paired Nano, RX RF ID and
    transponder ID are all configured in Vehicle Management. This never
    checks whether the Nano serial device or RF receiver is currently online.
    """
    data = request.get_json() or {}
    vehicle_name = data.get("vehicle_name")

    try:
        if not validate_cockpit_id(cockpit_id):
            return jsonify({
                "success": False,
                "error": "Invalid cockpit ID"
            }), 400

        if vehicle_name in (None, "", "UNASSIGNED"):
            success = cockpit_manager.clear_nano(cockpit_id)
            selected_vehicle = None
            if success:
                # Explicitly unassigning means the operator no longer wants
                # this cockpit to remember its previous car.
                _set_preferred_car(cockpit_id, None)
        else:
            vehicle_name = str(vehicle_name).strip()

            try:
                success = cockpit_manager.select_vehicle_by_registry(
                    cockpit_id,
                    vehicle_name
                )
            except ValueError as exc:
                return jsonify({
                    "success": False,
                    "error": str(exc)
                }), 409

            selected_vehicle = vehicle_name if success else None
            if success:
                # Only a deliberate operator selection changes the
                # persistent preference. Session expiry/stop never does.
                _set_preferred_car(cockpit_id, vehicle_name)

        if success and selected_vehicle is not None:
            cockpit = cockpit_manager.get_cockpit(cockpit_id)
            if cockpit is not None and cockpit.radio_id is not None:
                # Wi-Fi provisioning is best-effort and must never block or
                # disable the already-working RF control path.
                try:
                    provision_wifi_to_radio(cockpit.radio_id)
                except Exception as exc:
                    print(
                        f"[Cockpit API] WiFi provision skipped for "
                        f"{cockpit.radio_id}: {exc}"
                    )

            # Start control when a wheel is present and a car (with registry
            # Nano pairing) is selected. Do not require Nano serial online.
            if (
                cockpit is not None
                and cockpit.wheel is not None
                and cockpit.radio_id is not None
                and cockpit.vehicle_name is not None
            ):
                start_cockpit_control_worker(cockpit_id)

        if not success:
            return jsonify({
                "success": False,
                "error": "Vehicle assignment failed"
            }), 400

        return jsonify({
            "success": True,
            "cockpit_id": cockpit_id,
            "vehicle": selected_vehicle
        })

    except Exception as e:
        print(f"[Cockpit API] Vehicle assignment error: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 400


def validate_cockpit_id(cockpit_id):
    return 1 <= cockpit_id <= cockpit_manager.max_cockpits


@app.route('/api/cockpits/<int:cockpit_id>/settings', methods=['GET', 'POST'])
def cockpit_settings_api(cockpit_id):
    if not validate_cockpit_id(cockpit_id):
        return jsonify({
            "success": False,
            "error": "Invalid cockpit ID"
        }), 400

    if request.method == 'GET':
        with cockpit_settings_lock:
            return jsonify({
                "success": True,
                "cockpit_id": cockpit_id,
                "settings": dict(cockpit_settings[cockpit_id])
            })

    data = request.get_json() or {}

    duration_value = data.get("session_duration_minutes")
    sensitivity_value = data.get("steering_sensitivity")
    throttle_value = data.get("throttle_sensitivity") if "throttle_sensitivity" in data else data.get("throttle_limit")

    with cockpit_sessions_lock:
        session_active_for_cockpit = cockpit_sessions[cockpit_id]["active"]

    if duration_value is not None:
        try:
            duration_value = int(duration_value)
        except (TypeError, ValueError):
            return jsonify({"success": False, "error": "Session duration must be an integer"}), 400

        with cockpit_settings_lock:
            current_duration = cockpit_settings[cockpit_id]["session_duration_minutes"]

        if session_active_for_cockpit and duration_value != current_duration:
            return jsonify({
                "success": False,
                "error": "Cannot change session duration while session is active"
            }), 400

        if not 1 <= duration_value <= 120:
            return jsonify({
                "success": False,
                "error": "Session duration must be between 1 and 120 minutes"
            }), 400

    if sensitivity_value is not None:
        try:
            sensitivity_value = int(sensitivity_value)
        except (TypeError, ValueError):
            return jsonify({
                "success": False,
                "error": "Steering sensitivity must be an integer"
            }), 400

        if not 10 <= sensitivity_value <= 200:
            return jsonify({
                "success": False,
                "error": "Steering sensitivity must be between 10 and 200"
            }), 400

    if throttle_value is not None:
        try:
            throttle_value = int(throttle_value)
        except (TypeError, ValueError):
            return jsonify({
                "success": False,
                "error": "Throttle sensitivity must be an integer"
            }), 400

        if not 10 <= throttle_value <= 100:
            return jsonify({
                "success": False,
                "error": "Throttle sensitivity must be between 10 and 100"
            }), 400

    with cockpit_settings_lock:
        current = cockpit_settings[cockpit_id]

        if duration_value is not None:
            current["session_duration_minutes"] = duration_value

        if sensitivity_value is not None:
            current["steering_sensitivity"] = sensitivity_value
        if throttle_value is not None:
            current["throttle_sensitivity"] = throttle_value
            
        driver_name_val = data.get("driver_name")
        if driver_name_val is not None:
            current["driver_name"] = str(driver_name_val).strip()
            
        autocenter_val = data.get("autocenter_enabled")

        if autocenter_val is not None:
            current["autocenter_enabled"] = bool(autocenter_val)
            set_autocenter_hardware(bool(autocenter_val), cockpit_id=cockpit_id)

        saved = dict(current)

    _save_cockpit_settings()

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id,
        "settings": saved
    })


@app.route('/api/cockpits/<int:cockpit_id>/session/start', methods=['POST'])
def cockpit_session_start_api(cockpit_id):
    if not validate_cockpit_id(cockpit_id):
        return jsonify({
            "success": False,
            "error": "Invalid cockpit ID"
        }), 400

    success, error = start_cockpit_session(cockpit_id)

    if not success:
        return jsonify({
            "success": False,
            "error": error
        }), 400

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id,
        "session": get_cockpit_session_state(cockpit_id)
    })


@app.route('/api/cockpits/<int:cockpit_id>/session/stop', methods=['POST'])
def cockpit_session_stop_api(cockpit_id):
    if not validate_cockpit_id(cockpit_id):
        return jsonify({
            "success": False,
            "error": "Invalid cockpit ID"
        }), 400

    success, error = stop_cockpit_session(cockpit_id)

    if not success:
        return jsonify({
            "success": False,
            "error": error
        }), 400

    return jsonify({
        "success": True,
        "cockpit_id": cockpit_id,
        "session": get_cockpit_session_state(cockpit_id)
    })


@app.route('/api/status')
def status():
    elapsed = int(time.time() - start_time) if session_active else 0
    return jsonify({
        "active": session_active,
        "autocenter": autocenter_enabled,
        "steering_sensitivity": steering_sensitivity,
        "throttle_limit": throttle_limit,
        "haptics": haptic_settings,
        "elapsed": f"{elapsed // 60:02d}:{elapsed % 60:02d}"
    })


def _reboot_pi():
    """Reboot the host Pi. Runs on a delay so the HTTP response can be sent."""
    time.sleep(1.0)
    try:
        subprocess.run(["sudo", "reboot"], check=False)
    except Exception as exc:
        print(f"[System] Reboot command failed: {exc}")


@app.route('/api/system/restart', methods=['POST'])
def system_restart_api():
    """Reboot the Raspberry Pi from the UI restart button."""
    try:
        threading.Thread(target=_reboot_pi, daemon=True).start()
        return jsonify({
            "success": True,
            "message": "Pi is restarting..."
        })
    except Exception as e:
        print(f"[System] Restart request error: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500


@app.route('/vehicle/settings', methods=['GET', 'POST'])
def vehicle_settings():
    global steering_sensitivity, throttle_limit

    if request.method == 'POST':
        data = request.get_json() or {}
        if "steering_sensitivity" in data:
            steering_sensitivity = int(data["steering_sensitivity"])
            with cockpit_settings_lock:
                for c_id in cockpit_settings:
                    cockpit_settings[c_id]["steering_sensitivity"] = steering_sensitivity
        if "throttle_limit" in data:
            throttle_limit = int(data["throttle_limit"])
            with cockpit_settings_lock:
                for c_id in cockpit_settings:
                    cockpit_settings[c_id]["throttle_sensitivity"] = throttle_limit
        _save_cockpit_settings()
        return jsonify({"success": True})

    return jsonify({
        "success": True,
        "steering_sensitivity": steering_sensitivity,
        "throttle_limit": throttle_limit
    })


@app.route('/api/haptics/tune', methods=['POST'])
def tune_haptics():
    global haptic_settings
    data = request.get_json() or {}

    if "grip_threshold" in data:
        haptic_settings["grip_threshold"] = float(data["grip_threshold"])
    if "bump_sensitivity" in data:
        haptic_settings["bump_sensitivity"] = float(data["bump_sensitivity"])
    if "centering_gain" in data:
        haptic_settings["centering_gain"] = float(data["centering_gain"])
    if "alpha" in data:
        haptic_settings["alpha"] = float(data["alpha"])

    return jsonify({"success": True, "haptics": haptic_settings})


@app.route('/api/autocenter/toggle', methods=['POST'])
def toggle_autocenter():
    global autocenter_enabled
    data = request.get_json() or {}
    autocenter_enabled = data.get("enabled", not autocenter_enabled)
    set_autocenter_hardware(autocenter_enabled)
    return jsonify({"success": True, "autocenter": autocenter_enabled})


@app.route('/api/start', methods=['POST'])
def start_session():
    global session_active, start_time
    if not session_active:
        start_time = time.time()
        session_active = True
    return jsonify({"success": True, "active": True})


@app.route('/api/stop', methods=['POST'])
def stop_session():
    global session_active
    session_active = False
    return jsonify({"success": True, "active": False})



@app.route('/api/ffb_test', methods=['POST'])
def ffb_test_api():
    data = request.json or {}
    cockpit_id = data.get('cockpit_id', 'cockpit1')
    effect = data.get('effect', 'rumble')
    
    cockpit = cockpit_manager.get_cockpit(cockpit_id)
    if not cockpit or not getattr(cockpit, 'wheel', None):
        return jsonify({'error': 'Cockpit or wheel not found'}), 400
        
    ffb = get_ffb_device(cockpit_id)
    if not ffb:
        return jsonify({'error': 'FFB not initialized'}), 400
        
    if effect == 'rumble':
        if hasattr(ffb, 'play_rumble'):
            ffb.play_rumble(1000)
    elif effect == 'terrain':
        if hasattr(ffb, 'play_terrain'):
            ffb.play_terrain(1000)
    elif effect == 'kick':
        ffb.set_hardware_autocenter(100.0)
        import threading
        threading.Timer(0.25, _release_cockpit_impact_ffb, args=(cockpit_id,)).start()
        
    return jsonify({'success': True, 'effect': effect})

if __name__ == '__main__':
    initialize_cockpit_system()

    reconcile_esp_devices()

    update_esp_ap_presence()

#    start_vehicle_discovery()

    start_lap_timer()

    for cockpit_id in range(1, ESP_COCKPIT_COUNT + 1):
        start_esp_cockpit_control_worker(cockpit_id)

    # Telemetry reception, impact detection and validated G29 impact FFB.
#    telemetry_receiver.start()

    # Pi-owned cockpit session timer.
    # This does not touch Nano serial connections or the RF control loop.
    session_timer_thread = threading.Thread(
        target=cockpit_session_worker,
        daemon=True
    )
    session_timer_thread.start()

    # Multi-cockpit control. Start the same generic worker for all logical
    # cockpits. Unassigned cockpits exit safely without affecting the
    # cockpits that have valid wheel/radio pairs. RadioManager owns all Nano
    # serial connections; these workers never open Nano serial ports.
    for cockpit_id in range(1, cockpit_manager.max_cockpits + 1):
        start_cockpit_control_worker(cockpit_id)

    # Keep Flask alive while G29s are plugged/unplugged. The monitor only
    # rescans wheels when the detected wheel set changes.
    start_cockpit_device_monitor()

    try:
        app.run(host='0.0.0.0', port=5001)
    finally:
        telemetry_receiver.stop()
        stop_lap_timer()
        stop_vehicle_discovery()

        stop_cockpit_device_monitor()
        stop_cockpit_control_workers()
        stop_esp_cockpit_control_workers()
        cockpit_manager.close()
