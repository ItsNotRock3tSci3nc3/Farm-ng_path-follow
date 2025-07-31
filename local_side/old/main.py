import time
from local_side.sensors.gps_reader import get_latest_fix
from local_side.old.imu_receiver import get_yaw
from local_side.old.nav_utils import haversine_distance, bearing_deg, angle_diff_deg
from local_side.old.controller import send_key

# 设置目标点（你可以动态读取）
TARGET_LAT = 38.9199
TARGET_LON = -92.4662

def main():
    while True:
        pos = get_latest_fix()
        lat, lon = pos["lat"], pos["lon"]
        yaw = get_yaw()

        dist = haversine_distance(lat, lon, TARGET_LAT, TARGET_LON)
        bearing = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)
        heading_diff = angle_diff_deg(bearing, yaw)

        print("\n🌐 当前定位")
        print(f"  纬度: {lat:.6f}, 经度: {lon:.6f}, 精度: {pos['precision']}")
        print(f"  朝向: {yaw:.2f}°")
        print(f"  距离目标: {dist:.2f} m")
        print(f"  目标方位: {bearing:.2f}°, 偏差: {heading_diff:.2f}°")

        # 简单控制逻辑
        if dist < 0.5:
            send_key("S")  # 到达目标，停止
        elif abs(heading_diff) > 15:
            send_key("D" if heading_diff > 0 else "A")
        else:
            send_key("W")

        time.sleep(0.5)

if __name__ == "__main__":
    main()
