import depthai as dai
import numpy as np
import cv2
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler
import time

# 初始化 Madgwick
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])
prev_time = None

# 创建 DepthAI Pipeline
pipeline = dai.Pipeline()
imu = pipeline.create(dai.node.IMU)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("imu")

imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
imu.out.link(xout.input)

# 可视化函数
def draw_axes(img, R, origin, length=100):
    """Draw 3D axes on a 2D image."""
    x_axis = R @ np.array([[1], [0], [0]]) * length
    y_axis = R @ np.array([[0], [1], [0]]) * length
    z_axis = R @ np.array([[0], [0], [1]]) * length

    origin = tuple(np.int32(origin))
    x_end = tuple(np.int32(origin + x_axis[:2].flatten()))
    y_end = tuple(np.int32(origin + y_axis[:2].flatten()))
    z_end = tuple(np.int32(origin + z_axis[:2].flatten()))

    cv2.line(img, origin, x_end, (0, 0, 255), 2)  # X - red
    cv2.line(img, origin, y_end, (0, 255, 0), 2)  # Y - green
    cv2.line(img, origin, z_end, (255, 0, 0), 2)  # Z - blue

    return img

# 主程序
with dai.Device(pipeline) as device:
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=True)

    while True:
        imuData = imuQueue.get()
        for packet in imuData.packets:
            acc = packet.acceleroMeter
            gyr = packet.gyroscope

            # 时间间隔
            t = acc.getTimestampDevice().total_seconds()
            if prev_time is None:
                prev_time = t
                continue
            dt = t - prev_time
            prev_time = t

            a = np.array([acc.x, acc.y, acc.z])
            g = np.array([gyr.x, gyr.y, gyr.z])

            q = madgwick.updateIMU(q, g, a)
            roll, pitch, yaw = q2euler(q)

            # 旋转矩阵（ZXY）
            R = cv2.Rodrigues(np.array([roll, pitch, yaw]))[0]

            # 创建画布
            canvas = np.ones((480, 640, 3), dtype=np.uint8) * 255
            draw_axes(canvas, R, origin=(320, 240), length=80)

            # 显示角度
            text = f"Roll: {np.degrees(roll):+.1f}°, Pitch: {np.degrees(pitch):+.1f}°, Yaw: {np.degrees(yaw):+.1f}°"
            cv2.putText(canvas, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

            # 显示窗口
            cv2.imshow("IMU Orientation", canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
