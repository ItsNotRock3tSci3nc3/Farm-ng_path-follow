import depthai as dai
import math
import threading
import time

class oakDReceiver:
    def __init__(self):
        self.pipeline = dai.Pipeline()

        imu = self.pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)  # 100 Hz
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        xout = self.pipeline.create(dai.node.IMU)
        xout.setStreamName("imu")
        imu.out.link(xout.input)

        self.roll = None
        self.pitch = None
        self.yaw = None
        self.accuracy = None
        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("[OAK-D Receiver] IMU pipeline created.")

    def _read_loop(self, debug = False):
        with dai.Device(self.pipeline) as device:
            imuQueue = device.getOutputQueue("imu", maxSize=8, blocking=False)
            while self.running:
                try:
                    imuData = imuQueue.get().packets
                    for imuPacket in imuData:
                        if imuPacket.rotationVector is not None:
                            quat = imuPacket.rotationVector

                            x = quat.i
                            y = quat.j
                            z = quat.k
                            w = quat.real

                            self.accuracy = quat.accuracy

                            self.roll, self.pitch, self.yaw = self.quat_to_euler(w, x, y, z)
                            if debug:
                                print(f"Roll: {math.degrees(self.roll):.2f}, Pitch: {math.degrees(self.pitch):.2f}, Yaw: {math.degrees(self.yaw):.2f}")

                except Exception as e:
                    print("[OAK-D Receiver] Error reading IMU data:", e)

    def quat_to_euler(self, w, x, y, z):
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def get_yaw(self):
        with self.lock:
            return self.yaw
    
    def get_roll(self):
        with self.lock:
            return self.roll
    
    def get_pitch(self):
        with self.lock:
            return self.pitch
        
    def get_accuracy(self):
        with self.lock:
            return self.accuracy
    
    def get_all(self): 
        with self.lock:
            return self.roll, self.pitch, self.yaw, self.accuracy

    def stop(self):
        self.running = False

if __name__ == "__main__":
    oakD = oakDReceiver()

    try:
        while True:
            roll, pitch, yaw, accuracy= oakD.get_all()
            if yaw is not None:
                print(f"[OAK-D Receiver] Current Yaw: {math.degrees(yaw):.2f}° | Current Accuracy: {accuracy}")
            else:
                print("[OAK-D Receiver] Yaw data not available.")
            #time.sleep(1)
    except KeyboardInterrupt:
       
        print("\n[OAK-D Receiver] Stopping IMU reader...")
        oakD.stop()
