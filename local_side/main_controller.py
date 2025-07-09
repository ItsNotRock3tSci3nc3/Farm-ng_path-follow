import asyncio
import cv2
import math
import websockets
import pygame
import time
# from imu_receiver import get_yaw
from gps_reader import get_latest_fix 
from imu_bno085_receiver import IMUReader

from kalman import KalmanFilter2D

kf = KalmanFilter2D()

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
(38.9425477, -92.3197040),
(38.9425531, -92.3199857),
(38.9425558, -92.3204111),
(38.9425632, -92.3210972)
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

def get_filtered_gps():
    pos = _get_latest_fix()
    if pos is None:
        return None
    lat, lon = pos["lat"], pos["lon"]
    filtered_lat, filtered_lon = kf.update(lat, lon)
    pos["lat"] = filtered_lat
    pos["lon"] = filtered_lon
    return pos

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
            print("[AUTO] ✅ All waypoints completed, automatically switching to manual mode.")
            mode = "manual"
            continue

        #pos = _get_latest_fix()
        pos = get_filtered_gps() #filtered GPS data
        yaw = _get_yaw()

        print("[AUTO] 🛰️ Getting latest GPS and IMU data...")
        lat, lon, precision = pos["lat"], pos["lon"], pos["precision"]

        if precision > 2 or math.isnan(precision):
            print(f"[AUTO] ⚠️ Poor GPS precision: {precision:.2f}m")
            await asyncio.sleep(1)
            continue
        if yaw is None:
            print("[AUTO] ⚠️ IMU yaw acquisition failed, retrying...")
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
            print(f"[AUTO] 🎯 Reached waypoint {current_target_index + 1}")
            current_target_index += 1
            await asyncio.sleep(1)
            continue

        # === Smart turning (dynamic turning speed) ===
        if abs(diff) > 5:  # Only turn if the angle difference is greater than 5 degrees
            cmd = "d" if diff > 0 else "a"

            # Convert angle error to turning speed, range [0.1, 0.9]
            abs_diff = abs(diff)
            turn_speed = min(max(abs_diff / 90.0, 0.1), 0.9)

            # turn_speed = map_angle_to_speed(diff)

            print(f"[AUTO] 🔄 Angle error {abs_diff:.2f}° → Turning speed {turn_speed:.2f} + Command {cmd}")
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
# KML Logging task
# ======================
async def kml_logging_task(get_pos_func, kml_logger):
    try:
        while True:
            pos = get_pos_func()
            if pos is not None:
                lat, lon = pos["lat"], pos["lon"]
                kml_logger.add_point(lon, lat)
            await asyncio.sleep(3)
    finally:
        kml_logger.close()

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
    try:
        asyncio.run(main(), kml_logging_task())
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Running without kml logging")
        asyncio.run(main())
