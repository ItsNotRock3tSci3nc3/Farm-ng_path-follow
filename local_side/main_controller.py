import asyncio
import cv2
import math
import websockets
import pygame
import time
# from imu_receiver import get_yaw
from gps_reader import get_latest_fix 
from imu_bno085_receiver import IMUReader

# === Navigation target point ===
TARGET_LAT = 38.94123854
TARGET_LON = -92.31853863600745

# === Mode and speed levels ===
mode = "manual" # "manual" or "auto"
SPEED_LEVELS = [0.2, 0.4, 0.6, 0.8, 0.9, 1.0]  # Speed percentage
speed_index = 2
last_record_time = 0

imu_reader = IMUReader()

# === List of navigation targets (latitude, longitude) ===
# come way
#TARGETS = [(38.90764031, -92.26851491),(38.90768574, -92.26833880),(38.90775425, -92.26808159),(38.90780484, -92.26789371)]
# out
TARGETS = [
(38.90763367,-92.26851091),
(38.90763546,-92.26850523),
(38.90763791,-92.26849781),
(38.90763924,-92.26849308),
(38.90763925,-92.26849311),
(38.90763929,-92.26849298),
(38.90764068,-92.26848867),
(38.90764370,-92.26847903),
(38.90764633,-92.26846686),
(38.90764823,-92.26845158),
(38.90765200,-92.26843730),
(38.90765592,-92.26842193),
(38.90766055,-92.26840502),
(38.90766492,-92.26838928),
(38.90766871,-92.26837267),
(38.90767347,-92.26835599),
(38.90767810,-92.26833882),
(38.90768209,-92.26832337),
(38.90768691,-92.26830648),
(38.90769060,-92.26829082),
(38.90769497,-92.26827411),
(38.90769944,-92.26825842),
(38.90770364,-92.26824268),
(38.90770816,-92.26822651),
(38.90771302,-92.26820915),
(38.90771661,-92.26819415),
(38.90772092,-92.26817839),
(38.90772532,-92.26816184),
(38.90772889,-92.26814607),
(38.90773290,-92.26813014),
(38.90773762,-92.26811333),
(38.90774202,-92.26809619),
(38.90774646,-92.26808006),
(38.90775080,-92.26806321),
(38.90775572,-92.26804566),
(38.90776001,-92.26803034),
(38.90776452,-92.26801363),
(38.90776891,-92.26799773),
(38.90777291,-92.26798168),
(38.90777731,-92.26796545)
]
current_target_idx = 0

def get_current_target():
    if current_target_idx < len(TARGETS):
        return TARGETS[current_target_idx]
    return None, None

# ======================
# Simulated GPS and IMU interfaces
# ======================
def _get_latest_fix():
    # TODO: Replace with real GPS module

    # return {"lat": 38.94209340545889, "lon": -92.31854182078087, "precision": 0.8} # North
    # return {"lat": 38.940795965552134, "lon": -92.3185432362358, "precision": 0.5}  # South
    # return {"lat": 38.941245696031814, "lon": -92.31752127781246, "precision": 0.1}  # East
    # return {"lat": 38.94124074, "lon": -92.3189133776869, "precision": 0.05}  # West
    # return {"lat": 38.94208239631051, "lon": -92.31732877594878, "precision": 0.05}  # Northeast
    # return {"lat": 38.940759084223465, "lon": -92.31987659473259, "precision": 0.05}  # Southwest

    return get_latest_fix(debug=False)



def _get_yaw():
    # TODO: Replace with real IMU module
    # return 45.0  # Assume heading 45°
    return imu_reader.get_yaw()  # Get heading angle from IMU interface

# ======================
# Heading/distance calculation utilities
# ======================
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius (meters)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(d_lambda/2)**2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

def bearing_deg(lat1, lon1, lat2, lon2):
    d_lon = math.radians(lon2 - lon1)
    y = math.sin(d_lon) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def angle_diff_deg(bearing, yaw):
    return (bearing + yaw)

# ======================
# Autonomous navigation task
# ======================
def map_angle_to_speed(diff_deg):
    abs_diff = min(abs(diff_deg), 90.0)
    speed = 0.1 + 0.8 * (abs_diff / 90.0) ** 1.5
    return round(min(speed, 0.9), 2)

async def nav_task(ws):
    global mode, current_target_index
    while True:
        if mode != "auto":
            await asyncio.sleep(0.1)
            continue

        if current_target_index >= len(WAYPOINTS):
            print("[AUTO] ✅ 所有目标点已完成，自动切换为手动模式。")
            mode = "manual"
            continue

        pos = _get_latest_fix()
        yaw = _get_yaw()

        print("[AUTO] 🛰️ 获取最新 GPS 和 IMU 数据...")
        lat, lon, precision = pos["lat"], pos["lon"], pos["precision"]

        if precision > 2 or math.isnan(precision):
            print(f"[AUTO] ⚠️ GPS 精度较差: {precision:.2f}m")
            await asyncio.sleep(1)
            continue
        if yaw is None:
            print("[AUTO] ⚠️ IMU yaw 获取失败，重试中...")
            await asyncio.sleep(1)
            continue

        target_lat, target_lon = WAYPOINTS[current_target_index]
        dist = haversine_distance(lat, lon, target_lat, target_lon)
        bearing = bearing_deg(lat, lon, target_lat, target_lon)
        if bearing > 180:
            bearing -= 360
        diff = angle_diff_deg(bearing, yaw)

        print(f"[AUTO] 📍 Waypoint {current_target_index+1}/{len(WAYPOINTS)} | "
              f"Lat={lat:.8f}, Lon={lon:.8f}, Dist={dist:.2f}m, Yaw={yaw:.2f}, "
              f"bearing={bearing:.2f}, Δ={diff:.2f}, Precision={precision:.2f}")

        if dist < 0.5:
            print(f"[AUTO] 🎯 已到达目标点 {current_target_index + 1}")
            current_target_index += 1
            await asyncio.sleep(1)
            continue

        # === 智能转向（动态转向速度） ===
        if abs(diff) > 5:  # 夹角大于5度才转向
            cmd = "d" if diff > 0 else "a"

            # 将角度误差转为转向速度，范围 [0.1, 0.9]
            abs_diff = abs(diff)
            turn_speed = min(max(abs_diff / 90.0, 0.1), 0.9)

            # turn_speed = map_angle_to_speed(diff)

            print(f"[AUTO] 🔄 角度误差 {abs_diff:.2f}° → 转向速度 {turn_speed:.2f} + 指令 {cmd}")
            await ws.send(f"{turn_speed:.2f}")
        else:
            cmd = "w"

        await ws.send(cmd)
        await asyncio.sleep(0.5)

# ======================
# Keyboard control task
# ======================
async def keyboard_task(ws):
    global mode, speed_index

    pygame.init()
    screen = pygame.display.set_mode((300, 100))  # Must create a window
    pygame.display.set_caption("Control Window")

    pressed_keys = set()

    clock = pygame.time.Clock()



    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

            if event.type == pygame.KEYDOWN:
                key = event.key
                pressed_keys.add(key)

                # Mode switch
                if key == pygame.K_m:
                    mode = "manual" if mode == "auto" else "auto"
                    print(f"🌀 Switched to {mode.upper()} mode.")
                    await ws.send(" ")  # Notify robot to clear state
                    continue

                # Speed level switch (number keys 1~6)
                if pygame.K_1 <= key <= pygame.K_6:
                    speed_index = key - pygame.K_1
                    print(f"⚙️ Speed level set to {speed_index + 1} ({SPEED_LEVELS[speed_index] * 100:.2f} %)")
                    await ws.send(str(speed_index + 1))

            elif event.type == pygame.KEYUP:
                pressed_keys.discard(event.key)

        if mode != "manual":
            await asyncio.sleep(0.01)
            continue


        pos = _get_latest_fix()
        yaw = _get_yaw()
        lat, lon, precision = pos["lat"], pos["lon"], pos["precision"]
        dist = haversine_distance(lat, lon, TARGET_LAT, TARGET_LON)
        bearing = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)
        if bearing > 180:
            bearing -= 360
        diff = angle_diff_deg(bearing, yaw)
        print(f"[MANUAL] 📍 Lat={lat:.8f}, Lon={lon:.8f}, Dist={dist:.2f}m, Yaw={yaw:.2f}, bearing= {bearing:.2f}, Δ={diff:.2f}, Precision={precision:.2f}")
        
        
	
        # === Handle direction combinations ===
        if pressed_keys:
            commands = []
            if pygame.K_w in pressed_keys or pygame.K_UP in pressed_keys:
                commands.append("w")
            if pygame.K_s in pressed_keys or pygame.K_DOWN in pressed_keys:
                commands.append("s")
            if pygame.K_a in pressed_keys or pygame.K_LEFT in pressed_keys:
                commands.append("a")
            if pygame.K_d in pressed_keys or pygame.K_RIGHT in pressed_keys:
                commands.append("d")

            for cmd in commands:
                await ws.send(cmd)
                print(f"🎮 Sending: {cmd}")

        await asyncio.sleep(0.1)  # Control sending frequency (10Hz)
        clock.tick(60)


# ======================
# Main entry
# ======================
async def main():
    uri = "ws://100.87.161.11:8555"  # Replace with your robot IP
    print(f"🚀 Connecting to robot at {uri}")
    async with websockets.connect(uri) as ws:
        await asyncio.gather(
            nav_task(ws),
            keyboard_task(ws)
        )

if __name__ == "__main__":
    asyncio.run(main())
