def get_yaw():
    # ⚠️ 实际使用中请替换为 IMU 实时 yaw 角度（0~360°）
    return 180


import json, socket

_ROBOT_IP = "100.87.161.11"  # ← 修改为 robot 实际 IP
# _ROBOT_IP = "localhost"  # 本地测试时使用
_IMU_PORT = 6000             # 与 robot 端 imu_sender.py 对应

sock = None

def _connect():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect((_ROBOT_IP, _IMU_PORT))
    sock_file = sock.makefile("r")  # text mode iterator
    return sock_file

_file = None

def get_yaw():
    """Read one JSON line and return yaw (deg). Re‑connect on failure."""
    global _file
    if _file is None:
        try:
            _file = _connect()
        except Exception as e:
            print(f"[IMU] connect error: {e}")
            return None
    try:
        line = _file.readline()
        if not line:
            raise ValueError("EOF")
        data = json.loads(line)
        # Check for error in the received data
        if "error" in data:
            print(f"[IMU] Error received: {data['error']}")
            return None
        print(f"[IMU] Received data: {data}")
        return float(data.get("yaw", 0)) % 360
    except Exception as e:
        print(f"[IMU] read error: {e}; reconnecting…")
        try:
            _file.close()
        except:
            pass
        _file = None
        return None