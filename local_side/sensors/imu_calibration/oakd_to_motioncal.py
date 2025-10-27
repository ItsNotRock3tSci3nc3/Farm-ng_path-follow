import depthai as dai
import serial
import time
import subprocess
import os

# === Step 1: Create virtual serial port using socat ===
pty = subprocess.Popen(
    ['socat', '-d', '-d', 'PTY,raw,echo=0', 'PTY,raw,echo=0'],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

# Wait for PTYs to be printed
time.sleep(2)
lines = pty.stderr.read().decode().split('\n')
pty_lines = [line for line in lines if 'PTY is' in line]
dev1 = pty_lines[0].split()[-1]
dev2 = pty_lines[1].split()[-1]

print(f"\nVirtual serial port created:")
print(f"Connect MotionCal to: {dev2}")
print(f"Python writing to:     {dev1}\n")

# === Step 2: Open the writer serial port ===
ser = serial.Serial(dev1, baudrate=115200)

# === Step 3: Start DepthAI pipeline ===
pipeline = dai.Pipeline()
imu = pipeline.createIMU()
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
imu.enableIMUSensor(dai.IMUSensor.MAGNETOMETER, 100)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)

xout = pipeline.createXLinkOut()
xout.setStreamName("imu")
imu.out.link(xout.input)

with dai.Device(pipeline) as device:
    imu_queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    
    while True:
        imuData = imu_queue.get()
        packets = imuData.packets

        for packet in packets:
            accel = packet.acceleroMeter
            gyro = packet.gyroscope
            mag = packet.magneticField

            ax, ay, az = int(accel.x * 1000), int(accel.y * 1000), int(accel.z * 1000)
            gx, gy, gz = int(gyro.x * 1000), int(gyro.y * 1000), int(gyro.z * 1000)
            mx, my, mz = int(mag.x * 1000), int(mag.y * 1000), int(mag.z * 1000)

            # Format as MotionCal expects
            ser.write(f"RawMag: {mx} {my} {mz}\n".encode())
            ser.write(f"RawAccel: {ax} {ay} {az}\n".encode())
            ser.write(f"RawGyro: {gx} {gy} {gz}\n".encode())

        time.sleep(0.01)
