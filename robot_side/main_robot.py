# robot/main.py
# from imu_sender import start_imu_server
from controller_receiver import start_ctrl_server
import threading

def main():
    print("[Robot] Starting...")

    # imu_thread = threading.Thread(target=start_imu_server, daemon=True)
    ctrl_thread = threading.Thread(target=start_ctrl_server, daemon=True)

    # imu_thread.start()
    ctrl_thread.start()

    print("[Robot] Services started (IMU + Control)")
    # imu_thread.join()
    ctrl_thread.join()

if __name__ == "__main__":
    main()
