# robot/main.py
# from imu_sender import start_imu_server
from controller_receiver import start_ctrl_server
import threading

def main():
    print("[Robot] 启动中…")

    # imu_thread = threading.Thread(target=start_imu_server, daemon=True)
    ctrl_thread = threading.Thread(target=start_ctrl_server, daemon=True)

    # imu_thread.start()
    ctrl_thread.start()

    print("[Robot] 服务已启动（IMU + 控制）")
    # imu_thread.join()
    ctrl_thread.join()

if __name__ == "__main__":
    main()
