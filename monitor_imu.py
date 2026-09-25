import socket
import struct
import math
import sys

PACKET_FORMAT = "<B6sI6f"

print("Starting live IMU monitor... (Move the ESP32 to see changes, Ctrl+C to stop)")
print(f"{'-'*75}")
print(f"{'Time (ms)':<10} | {'Accel X (G)':<12} | {'Accel Y (G)':<12} | {'Accel Z (G)':<12} | {'Total G':<12}")
print(f"{'-'*75}")

try:
    # Requires root privileges (sudo)
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003))
    s.bind(("wlan0", 0))
    
    while True:
        packet, addr = s.recvfrom(65535)
        # Parse Ethernet (14) + IP (20) + UDP (8) = 42 bytes header roughly
        # Let's just search for the start byte '0x04' in the packet payload
        idx = packet.find(b'\x04')
        if idx != -1 and len(packet) - idx >= 35:
            payload = packet[idx:idx+35]
            try:
                vals = struct.unpack(PACKET_FORMAT, payload)
                if vals[0] == 4:
                    ts = vals[2]
                    ax, ay, az = vals[3], vals[4], vals[5]
                    total_g = math.sqrt(ax*ax + ay*ay + az*az)
                    print(f"{ts:<10} | {ax:>12.3f} | {ay:>12.3f} | {az:>12.3f} | {total_g:>12.3f}")
            except Exception:
                pass

except PermissionError:
    print("Please run this script with sudo: sudo python3 monitor_imu.py")
except KeyboardInterrupt:
    print("\nStopped.")
except Exception as e:
    print(f"Error: {e}")

