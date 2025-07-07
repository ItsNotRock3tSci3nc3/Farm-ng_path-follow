import depthai as dai
import math


def quaternion_to_euler_deg(i, j, k, real):
    """四元数 -> 欧拉角（度）"""
    x, y, z, w = i, j, k, real

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2 * (w * y - z * x)
    pitch = math.degrees(math.copysign(math.pi / 2, sinp)) if abs(sinp) >= 1 else math.degrees(math.asin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return {"roll": roll, "pitch": pitch, "yaw": yaw}


class IMUStream:
    def __init__(self):
        self.device = dai.Device()
        self.imu_type = self.device.getConnectedIMU()
        self.firmware_version = str(self.device.getIMUFirmwareVersion())

        if self.imu_type != "BNO086":
            raise RuntimeError("Only BNO086 IMU supports rotation vector")

        pipeline = dai.Pipeline()
        imu = pipeline.create(dai.node.IMU)
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("imu")

        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        imu.out.link(xout.input)

        self.device.startPipeline(pipeline)
        self.queue = self.device.getOutputQueue(name="imu", maxSize=10, blocking=True)
        self.base_ts = None

    def get_sample(self):
        imu_data = self.queue.get()
        for packet in imu_data.packets:
            rV = packet.rotationVector

            if self.base_ts is None:
                self.base_ts = rV.getTimestampDevice()
            delta_t = rV.getTimestampDevice() - self.base_ts

            quat = {
                "i": rV.i,
                "j": rV.j,
                "k": rV.k,
                "real": rV.real,
            }
            euler = quaternion_to_euler_deg(**quat)

            return {
                "timestamp_ms": delta_t.total_seconds() * 1000,
                "quaternion": quat,
                "euler_deg": euler,
                "accuracy_rad": rV.rotationVectorAccuracy,
                "imu_type": self.imu_type,
                "firmware_version": self.firmware_version,
            }
