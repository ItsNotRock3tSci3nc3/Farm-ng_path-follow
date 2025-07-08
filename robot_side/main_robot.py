1| # robot/main.py
2| # from imu_sender import start_imu_server
3| from controller_receiver import start_ctrl_server
4| import threading
5| 
6| def main():
7|     print("[Robot] Starting…")
8| 
9|     # imu_thread = threading.Thread(target=start_imu_server, daemon=True)
10|     ctrl_thread = threading.Thread(target=start_ctrl_server, daemon=True)
11| 
12|     # imu_thread.start()
13|     ctrl_thread.start()
14| 
15|     print("[Robot] Services started (IMU + Control)")
16|     # imu_thread.join()
17|     ctrl_thread.join()
18| 
19| if __name__ == "__main__":
20|     main()
21| 
