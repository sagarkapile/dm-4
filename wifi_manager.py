import time
import subprocess

AP_SSID = "DriveMatrix-AP"
AP_PASSWORD = "drivematrix"

def scan_networks():
    try:
        result = subprocess.check_output([
            "nmcli",
            "-t",
            "-f",
            "SSID,SIGNAL",
            "device",
            "wifi",
            "list"
        ]).decode()

        networks = {}

        for line in result.splitlines():
            try:
                parts = line.split(":")
                if len(parts) >= 2:
                    ssid = parts[0].strip()
                    signal = parts[1].strip()
                    if ssid:
                        sig_int = int(signal) if signal.isdigit() else 0
                        if ssid not in networks:
                            networks[ssid] = sig_int
                        else:
                            networks[ssid] = max(networks[ssid], sig_int)
            except Exception:
                pass

        return sorted(
            networks.items(),
            key=lambda x: x[1],
            reverse=True
        )
    except Exception as e:
        print(f"[WiFi Manager] Scan error: {e}", flush=True)
        return []

def connect_wifi(ssid, password=""):
    print(
        "[WiFi Manager] External Wi-Fi connection is disabled. "
        "Use nmcli manually.",
        flush=True
    )
    return False

def get_wifi_state():
    try:
        output = subprocess.check_output(
            ["nmcli", "-t", "-f", "TYPE,STATE,CONNECTION", "device"]
        ).decode().splitlines()
        
        for line in output:
            parts = line.split(":")
            if len(parts) >= 2 and parts[0] == "wifi":
                state = parts[1]
                conn_name = parts[2] if len(parts) > 2 else ""
                return state, conn_name
    except Exception:
        pass
    return "disconnected", ""

def start_access_point():
    print("[WiFi Manager] Starting Access Point...", flush=True)
    try:
        connections = subprocess.check_output(
            ["nmcli", "-t", "-f", "NAME", "connection", "show"]
        ).decode().splitlines()

        if AP_SSID in connections:
            subprocess.call(["sudo", "nmcli", "connection", "up", AP_SSID])
        else:
            subprocess.call([
                "sudo", "nmcli", "device", "wifi", "hotspot",
                "ifname", "wlan0",
                "ssid", AP_SSID,
                "password", AP_PASSWORD
            ])
            subprocess.call([
                "sudo", "nmcli", "connection", "modify",
                "Hotspot", "connection.id", AP_SSID
            ])
    except Exception as e:
        print(f"[WiFi Manager] Failed to start AP: {e}", flush=True)

def monitor_wifi():
    time.sleep(10)

    # Start DriveMatrix AP once if it is not already active.
    state, conn_name = get_wifi_state()

    if conn_name != AP_SSID:
        print(
            f"[WiFi Manager] Starting {AP_SSID} "
            f"(current connection: {conn_name or 'NONE'})",
            flush=True
        )
        start_access_point()

    # Keep the process alive.
    # Do NOT force DriveMatrix-AP back if the user manually
    # changes Wi-Fi using nmcli.
    while True:
        time.sleep(60)

if __name__ == "__main__":
    monitor_wifi()
