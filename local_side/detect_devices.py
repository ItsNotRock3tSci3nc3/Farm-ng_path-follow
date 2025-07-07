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

esp32_port = find_device_by_manufacturer("Espressif")
rtk_port = find_device_by_manufacturer("Emlid")

print(f"ESP32: {esp32_port or 'Not found'}")
print(f"RTK  : {rtk_port or 'Not found'}")

# 保存为 env 文件（供 Docker Compose 使用）
with open("device.env", "w") as f:
    if esp32_port:
        f.write(f"ESP32_PORT={esp32_port}\n")
    if rtk_port:
        f.write(f"RTK_PORT={rtk_port}\n")
