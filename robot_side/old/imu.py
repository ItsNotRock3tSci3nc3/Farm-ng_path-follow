import depthai as dai
import math
from typing import Optional


def quaternion_to_euler_deg(i: float, j: float, k: float, real: float):
    """Convert quaternion (i, j, k, real) to Euler angles (deg)."""
    x, y, z, w = i, j, k, real
    # Roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    # Pitch
    sinp = 2 * (w * y - z * x)
    pitch = (
        math.degrees(math.copysign(math.pi / 2, sinp))
        if abs(sinp) >= 1
        else math.degrees(math.asin(sinp))
    )
    # Yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp)) % 360
    return {"roll": roll, "pitch": pitch, "yaw": yaw}


class IMUStream:
    """Continuous IMU rotation‑vector stream.

    Args:
        device_id:  可选。指定要连接的设备 MXID 或 IP（USB/OAK‑PoE）。
                        - None  : 自动连接首个检测到的设备。
                        - "list": 只列出所有可用设备并抛异常（方便查看）。
    """

    def __init__(self, device_id: Optional[str] = None):
        # ------- 选择设备 ----------
        available = dai.Device.getAllAvailableDevices()
        if device_id == "list":
            raise RuntimeError(f"Available devices: {[d.getMxId() for d in available]}")
        selected_info = None
        if device_id is None:
            if not available:
                raise RuntimeError("No OAK devices found")
            selected_info = available[0]
        else:
            for info in available:
                if device_id in (info.getMxId(), info.name, info.ip):
                    selected_info = info
                    break
            if selected_info is None:
                raise RuntimeError(
                    f"Device '{device_id}' not found. Available MXIDs: {[d.getMxId() for d in available]}"
                )

        # ------- 连接设备 ----------
        self.device = dai.Device(selected_info)
        self.imu_type = self.device.getConnectedIMU()
        self.firmware_version = str(self.device.getIMUFirmwareVersion())
        if self.imu_type != "BNO086":
            raise RuntimeError("Only BNO086 IMU supports rotation vector")

        # ------- 构建 pipeline ----------
        pipe = dai.Pipeline()
        imu = pipe.create(dai.node.IMU)
        xout = pipe.create(dai.node.XLinkOut)
        xout.setStreamName("imu")
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        imu.out.link(xout.input)

        self.device.startPipeline(pipe)
        self.queue = self.device.getOutputQueue("imu", maxSize=10, blocking=True)
        self.base_ts = None
        self.mxid = selected_info.getMxId()

    # ---------- public API ----------
    def get_sample(self):
        """Return a single rotation‑vector sample as dict."""
        imu_data = self.queue.get()
        for pkt in imu_data.packets:
            rv = pkt.rotationVector
            if self.base_ts is None:
                self.base_ts = rv.getTimestampDevice()
            dt_ms = (rv.getTimestampDevice() - self.base_ts).total_seconds() * 1000
            quat = {"i": rv.i, "j": rv.j, "k": rv.k, "real": rv.real}
            euler = quaternion_to_euler_deg(**quat)
            return {
                "timestamp_ms": dt_ms,
                "quaternion": quat,
                "euler_deg": euler,
                "accuracy_rad": rv.rotationVectorAccuracy,
                "imu_type": self.imu_type,
                "firmware_version": self.firmware_version,
                "mxid": self.mxid,
            }
