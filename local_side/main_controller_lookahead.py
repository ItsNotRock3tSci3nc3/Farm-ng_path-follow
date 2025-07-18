"""
THIS PROGRAM IS A VERSION OF THE MAIN CONTROLLER
This verion of main_controller.py is designed to work with the lookahead controller for smoother navigation and path correction
"""

import asyncio
import cv2
import os
import math
import websockets
import pygame
import time
import datetime

from gps_reader import get_latest_fix 
from imu_bno085_receiver import IMUReader

from CSVLogger import CSVLogger

test_name = "robot_track_lookahead" #CHANGE BEFORE TESTING
logging = input("Enable logging(y/n)? Press Enter to continue\n")
if logging.lower() == "y" or logging.lower() == "yes":
    csv_logger = CSVLogger(f"{test_name}.csv", True)
else:
    csv_logger = CSVLogger(f"{test_name}.csv", False)

from kalman_filter import KalmanFilter2D
kf = KalmanFilter2D()

from yaw_filter import YawFilter
yaw_filter = YawFilter(alpha=0.7)  # Adjust alpha as needed

# === Navigation target point ===
TARGET_LAT = 38.94123854
TARGET_LON = -92.31853863600745

# === Coordinate buffer radius ===
WAYPOINT_RADIUS = 2.0  # meters (adjust as needed)


# === Mode and speed levels ===
mode = "manual" # "manual" or "auto"
SPEED_LEVELS = [0.2, 0.4, 0.6, 0.8, 0.9, 1.0]  # Speed percentage
speed_index = 2
last_record_time = 0

# === lookahead distance ===
LOOKAHEAD_DISTANCE = max(1.0, speed_index * 3.0)  # meters (adjust as needed)

# === IMU reader instance ===
imu_reader = IMUReader()

# === List of navigation targets (latitude, longitude) ===
# come way
#TARGETS = [(38.90764031, -92.26851491),(38.90768574, -92.26833880),(38.90775425, -92.26808159),(38.90780484, -92.26789371)]
# out

"""
Sanborn Field
TARGETS = [
(38.9425311, -92.31954402),
(38.9425488, -92.31965974),
(38.9425507, -92.31999613),
(38.9425491, -92.32057431)
]
"""

"""
Yard
"""
TARGETS = [
(38.941226778933206, -92.31878180426872),
(38.94113774185417, -92.31877807500175)
]

"""
TARGETS = [
(38.94253063, -92.31954402),
(38.94253776, -92.32057431)
]
"""

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
    raw_yaw = imu_reader.get_yaw()
    yaw_accuracy = imu_reader.get_accuracy()

    if raw_yaw is None or yaw_accuracy is None:
        return None

    if yaw_accuracy > 5.0:
        print(f"[IMU] ⚠️ Yaw accuracy too low ({fmt(yaw_accuracy)}), discarding yaw")
        return yaw_filter.filtered_yaw

    filtered_yaw = yaw_filter.update(raw_yaw, yaw_accuracy)
    # ✅ Invert yaw to correct flipped IMU direction
    corrected_yaw = -filtered_yaw

    # Optional: normalize to [-180, 180] just in case
    if corrected_yaw > 180:
        corrected_yaw -= 360
    elif corrected_yaw < -180:
        corrected_yaw += 360

    return corrected_yaw
    #return yaw_filter.update(raw_yaw, yaw_accuracy)

def _get_yaw_accuracy():
    return imu_reader.get_accuracy()  # Get accuracy from IMU interface

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
    
    bearing = math.degrees(math.atan2(y, x))
    
    # Normalize to [-180, 180]
    if bearing > 180:
        bearing -= 360
    elif bearing < -180:
        bearing += 360

    return bearing

def angle_diff_deg(bearing, yaw):
    diff = bearing - yaw
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff

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
    global mode, current_target_idx
    while True:
        if mode != "auto":
            await asyncio.sleep(0.1)
            continue

        if current_target_idx >= len(TARGETS):
            print("[AUTO] ✅ All waypoints completed, automatically switching to manual mode.")
            mode = "manual"
            current_target_idx = 0
            await ws.send(" ")  # Notify robot to clear state
            continue

        pos = _get_latest_fix() #get_filtered_gps() #kalman
        yaw = _get_yaw()
        yaw_acc = _get_yaw_accuracy()

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
        if yaw_acc > 5.0:
            print(f"[AUTO] ⚠️ Yaw accuracy too low ({yaw_acc:.2f})")
            await asyncio.sleep(1)
            continue

        # === Pure Pursuit look-ahead point selection ===
        LOOKAHEAD_DISTANCE = max(1.0, speed_index * 3.0)  # meters (adjust as needed)
        lookahead_point = None
        for i in range(current_target_idx, len(TARGETS)):
            wp_lat, wp_lon = TARGETS[i]
            dist_to_wp = haversine_distance(lat, lon, wp_lat, wp_lon)
            if dist_to_wp >= LOOKAHEAD_DISTANCE:
                lookahead_point = (wp_lat, wp_lon)
                break

        # If no lookahead point found, use last waypoint
        if not lookahead_point:
            lookahead_point = TARGETS[-1]

        target_lat, target_lon = lookahead_point
        dist = haversine_distance(lat, lon, target_lat, target_lon)
        bearing = bearing_deg(lat, lon, target_lat, target_lon)

        if bearing > 180:
            bearing -= 360
        diff = angle_diff_deg(bearing, yaw)

        print_data("auto", f"Waypoint {current_target_idx+1}/{len(TARGETS)} | ", diff)


        if dist < WAYPOINT_RADIUS: #WAYPOINT_RADIUS: #0.5 was original, prior to WAYPOINT_RADIUS. Using for baseline. Buffer radius
            print(f"[AUTO] 🎯 Reached waypoint {current_target_idx + 1} within {WAYPOINT_RADIUS:.2f}m buffer")
            current_target_idx += 1
            continue

        # === Smart turning (dynamic turning speed) ===
        base_speed = SPEED_LEVELS[speed_index]

        if abs(diff) > 10:
            cmd = "d" if diff > 0 else "a"
            # Slow turning — reduce speed while turning
            turn_speed = max(0.1, base_speed * 0.4)
            print(f"[AUTO] 🔄 Turning | Δ={abs(diff):.2f}° | Turn Speed={turn_speed:.2f} | Command={cmd}")
            await ws.send(f"{turn_speed:.2f}")  # Send speed
            await ws.send(cmd)                  # Send turn direction
        else:
            cmd = "w"
            print(f"[AUTO] 🚀 Moving forward at speed {base_speed:.2f}")
            await ws.send(f"{base_speed:.2f}")  # Send speed
            await ws.send(cmd)                  # Send move command

        await asyncio.sleep(0.5)

# ======================
# Keyboard control task
# ======================
async def keyboard_task(ws):
    global mode, speed_index

    pygame.init()
    if pygame.get_init: print("pygame initialized") 
    else: print("pygame not initialized")
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

        print_data("manual")
        
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
# CSV Logging task
# ======================
async def csv_logging_task(get_pos_func, get_yaw, csv_logger):
    try:
        while True:
            pos = get_pos_func()
            yaw = get_yaw()
            if pos is not None and yaw is not None:
                lat, lon = pos["lat"], pos["lon"]
                yaw = yaw
                yaw_acc = _get_yaw_accuracy()
                dist = haversine_distance(lat, lon, TARGET_LAT, TARGET_LON)
                bearing = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)
                if yaw is None:
                    yaw = 0
                    print(f"[CSV] ⚠️ Yaw is None. Using 0 as placeholder. Yaw accuracy: {yaw_acc:.2f}")
                    continue
                diff = angle_diff_deg(bearing, yaw)
                precision = pos.get("precision", 0.0)
                bearing = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)
                if bearing > 180:
                    bearing -= 360

                UNIX_TIMESTAMP = time.time()  # Use a consistent timestamp for all points
                timestamp_converted = datetime.datetime.fromtimestamp(UNIX_TIMESTAMP)
                csv_logger.add_point(timestamp_converted, dist, lon, lat, yaw, yaw_acc, bearing, diff, precision)
            await asyncio.sleep(1)
    finally:
        csv_logger.close()

# ======================
# Print data to console
# ======================
def print_data(mode, ext_data=None, diff = None):
    mode = mode.upper()
    pos = get_filtered_gps()
    yaw = _get_yaw()
    yaw_acc = _get_yaw_accuracy()

    if pos is None:
        print(f"[{mode}] ⚠️ GPS data unavailable.")
        return
    if yaw is None:
        print(f"[{mode}] ⚠️ IMU yaw is None. Yaw accuracy: {fmt(yaw_acc)}")
        yaw = 0

    lat, lon, precision = pos.get("lat"), pos.get("lon"), pos.get("precision")
    dist = haversine_distance(lat, lon, TARGET_LAT, TARGET_LON)
    bearing = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)
    if bearing > 180:
        bearing -= 360
    if yaw is None:
        print(f"[{mode}] ⚠️ Yaw is None. Yaw accuracy: {yaw_acc:.2f}\n setting yaw to 0")
        yaw = 0  # Fallback to 0 if yaw is None
    try:
        if ext_data is not None:
            print(f"[{mode}] 📍 {ext_data} Lat={fmt(lat, 8)}°, Lon={fmt(lon, 8)}°, Dist={fmt(dist, 2)}m, Yaw={fmt(yaw, 2)}, Yaw precision = {fmt(yaw_acc)}, bearing= {fmt(bearing, 2)}, Δ={fmt(diff, 2)}, Precision={fmt(precision, 2)}")
        else:
            print(f"[{mode}] 📍 Lat={fmt(lat, 8)}°, Lon={fmt(lon, 8)}°, Dist={fmt(dist, 2)}m, Yaw={fmt(yaw, 2)}, Yaw precision = {fmt(yaw_acc)}, bearing= {fmt(bearing, 2)}, Δ={fmt(diff, 2)}, Precision={fmt(precision, 2)}")
    except Exception as e:
        print(f"[{mode}] Error printing data: {e}")   

# data formatter
def fmt(val, precision=None):
    try:
        if precision is None:
            return str(val) if isinstance(val, (float, int)) else "None"
        return f"{float(val):.{precision}f}"
    except (ValueError, TypeError):
        return "None" 

# ======================
# Main entry
# ======================
async def main():
    uri = "ws://100.87.161.11:8555"  # Replace with your robot IP
    print(f"🚀 Connecting to robot at {uri}")
    async with websockets.connect(uri) as ws:
        await asyncio.gather(
            nav_task(ws),
            keyboard_task(ws),
            csv_logging_task(get_filtered_gps, _get_yaw, csv_logger)
        )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"[CONTROLLER] Error: {e}\n{e.__annotations__}\n{e.__traceback__}")

