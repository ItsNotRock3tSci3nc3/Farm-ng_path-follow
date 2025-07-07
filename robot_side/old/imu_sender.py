# imu_sender.py

"""
持续发送 IMU Yaw 数据到 TCP 客户端
--------------------------------
- 监听 0.0.0.0:6000
- 客户端连接后，按 20 Hz 发送一行 JSON（示例为整数 125）
- 若 IMU 初始化失败，15 秒后退出程序并发送错误信息
"""

import json
import socket
import time
from imu import IMUStream
import os

_HOST = "0.0.0.0"
_PORT = 6000
SEND_HZ = 20
MAX_IMU_INIT_RETRY_SEC = 5
USE_FAKE_IMU = os.getenv("FAKE_IMU", "False") == "True"

def try_initialize_imu():
    start_time = time.time()
    while time.time() - start_time < MAX_IMU_INIT_RETRY_SEC:
        try:
            imu = IMUStream()
            print("[IMU-SENDER] ✅ IMU initialized successfully.")
            return imu
        except Exception as e:
            print(f"[IMU-SENDER] ⚠️ IMU init error: {e}; retrying in 2 s…")
            time.sleep(2)
    return None

def start_error_broadcast_loop():
    """持续广播错误信息，直到成功发出或达到最大重试次数"""
    max_retry = 10
    for attempt in range(max_retry):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as error_socket:
                error_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                error_socket.bind((_HOST, _PORT))
                error_socket.listen(1)
                error_socket.settimeout(10)  # 设置超时时间为10秒
                print(f"[IMU-SENDER] 🛑 Waiting for client to send error (attempt {attempt + 1})…")
                conn, addr = error_socket.accept()
                with conn, conn.makefile("w") as fp:
                    fp.write(json.dumps({"error": "imu_init_failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"}) + "\n")
                    fp.flush()
                    print("[IMU-SENDER] ⚠️ Error message sent to client.")
                    # return
        except Exception as e:
            print(f"[IMU-SENDER] ❌ Attempt {attempt + 1} failed: {e}")
        time.sleep(2)

    print("[IMU-SENDER] ⏹️ All error broadcast attempts failed. Exiting.")


def start_imu_server():
    """主入口"""
    print(f"[IMU-SENDER] 🌐 Listening on {_PORT} …")

    imu = try_initialize_imu()

    if imu is None and not USE_FAKE_IMU:
        print("[IMU-SENDER] ❌ IMU failed to initialize within time limit. Shutting down.")
        start_error_broadcast_loop()
        return

    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind((_HOST, _PORT))
                srv.listen(1)

                conn, addr = srv.accept()
                print(f"[IMU-SENDER] ✅ Client connected: {addr}")
                with conn, conn.makefile("w") as fp:
                    while True:
                        try:
                            if USE_FAKE_IMU:
                                # 模拟 IMU 数据
                                yaw = 125  # 假设的 yaw 角度
                            else:
                                sample = imu.get_sample()
                                yaw = sample["euler_deg"]["yaw"] if sample else None
                            fp.write(json.dumps({"yaw": yaw, "sample": sample}) + "\n")
                            fp.flush()
                            time.sleep(1 / SEND_HZ)
                        except (BrokenPipeError, ConnectionResetError):
                            print("[IMU-SENDER] 🔌 Client disconnected; waiting again…")
                            break
        except Exception as e:
            print(f"[IMU-SENDER] ⚠️ Server error: {e}; retrying in 2 s…")
            time.sleep(2)

if __name__ == "__main__":
    start_imu_server()
