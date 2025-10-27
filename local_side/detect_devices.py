import os
import subprocess

def find_device_by_manufacturer(target):
    for dev in os.listdir("/dev"):
        if dev.startswith("ttyACM") or dev.startswith("ttyUSB"):
            full_path = os.path.join("/dev", dev)
            try:
                out = subprocess.check_output(["udevadm", "info", "-a", full_path], universal_newlines=True)
                if target.lower() in out.lower():
                    return full_path
            except Exception:
                continue
    return None

def is_oakd_connected():
    try:
        output = subprocess.check_output(["lsusb"], universal_newlines=True)
        for line in output.splitlines():
            if "03e7:2485" in line:  # Intel Movidius MyriadX
                return True
    except Exception:
        pass
    return False

esp32_port = find_device_by_manufacturer("Espressif")
rtk_port = find_device_by_manufacturer("Emlid")
oakd_found = is_oakd_connected()

print(f"ESP32: {esp32_port or 'Not found'}")
print(f"RTK  : {rtk_port or 'Not found'}")
print(f"OAK-D: {'Connected' if oakd_found else 'Not found'}")

# Save as env file (for Docker Compose usage)
with open("device.env", "w") as f:
    if esp32_port:
        f.write(f"ESP32_PORT={esp32_port}\n")
    if rtk_port:
        f.write(f"RTK_PORT={rtk_port}\n")
