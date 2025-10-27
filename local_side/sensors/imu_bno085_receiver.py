import serial
import threading
import time
import os

class IMUReader:
    def __init__(self, baud=115200):
        port = os.getenv("ESP32_PORT")
        self.ser = serial.Serial(port, baud, timeout=1)
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.accuracy = None
        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print(f"[IMUReader] Listening on {port}...")

    def _read_loop(self):
        debug = False
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue
                
                if debug: 
                    print(f"[IMU READER] Received: {line}")
                
                parts = [x.strip() for x in line.split(",") if x.strip()]
                #if len(parts) != 4:
                #    print(f"[IMU READER] Invalid line: {line}")
                #    return None
                r, p, y, acc = map(float, line.split(","))

                if debug: 
                    print(f"[IMU READER] Parsed: Roll={r}, Pitch={p}, Yaw={y}, Accuracy={acc}")
                
                with self.lock:
                    self.roll = r
                    self.pitch = p
                    self.yaw = y
                    self.accuracy = acc
            except Exception as e:
                print("[IMU READER] Parse error:", e)

    def get_yaw(self):
        with self.lock:
            return self.yaw
        
    def get_accuracy(self):
        with self.lock:
            return self.accuracy

    def get_all(self):
        with self.lock:
            return self.roll, self.pitch, self.yaw, self.accuracy

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()

# 示例用法：
if __name__ == "__main__":
    imu = IMUReader()

    try:
        while True:
            yaw = imu.get_yaw()
            if yaw is not None:
                print(f"Current Yaw: {yaw:.2f}°")
            else:
                print("[IMU READER] Yaw data not available.")
            #time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopping IMU reader...")
        imu.stop()
