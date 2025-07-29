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
import traceback

from gps_reader import get_latest_fix 
from imu_bno085_receiver import IMUReader

from CSVLogger import CSVLogger

test_name = "robot_track_lookahead_10mSeg_RS2Test" #CHANGE BEFORE TESTING
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
# NOTE: The values are are initialized to are meaningless, they will be updated by the navigation task
TARGET_LAT = 38.90732879
TARGET_LON = -92.26825423


# === Coordinate buffer radius ===
WAYPOINT_RADIUS = 2.0  # meters (adjust as needed)


# === Mode and speed levels ===
mode = "manual" # "manual" or "auto"
SPEED_LEVELS = [0.2, 0.4, 0.6, 0.8, 0.9, 1.0]  # Speed percentage
speed_index = 2
last_record_time = 0

# === lookahead distance ===
LOOKAHEAD_DISTANCE = SPEED_LEVELS[speed_index] * 3.0  # e.g. 0.6 → 1.8m
LOOKAHEAD_DISTANCE = max(1.2, min(4.0, LOOKAHEAD_DISTANCE))
#LOOKAHEAD_DISTANCE = 1  # meters (adjust as needed)

omega_history = []

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
# TARGETS = [
# (38.941226778933206, -92.31878180426872),
# (38.94113774185417, -92.31877807500175)
# ]

TARGETS = [

(38.94126891,-92.31891656),
(38.9412354, -92.31887348),
(38.94121944,-92.31863472)
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

def nonlinear_omega(curvature, base_speed):
    #Gain is used to adjust the reaction speed of the robot to curvature. More gain means more reaction speed, and the opposite is true.
    # Increase gain to keep robot on track for tighter turns, decrease if there is too much oscillation
    gain = 1.6 

    if math.isnan(curvature):
        print("[AUTO WARN] NaN curvature in omega")
        return 0.0
    adjusted = math.copysign(abs(curvature) ** 0.5, curvature)
    return adjusted * base_speed * gain

def get_robot_frame(dx, dy, yaw_rad):
    # Transform coordinates to robot frame
    Xr = math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy
    Yr = -math.cos(yaw_rad) * dx + math.sin(yaw_rad) * dy
    return Xr, Yr

def check_lookahead_point(Xr, Yr, Lf, lookahead_point, wp_lat, wp_lon):
    if Xr < 0.2:
        print(f"[LOOKAHEAD SKIP] Lookahead point too lateral or behind: Xr={Xr:.2f}, Yr={Yr:.2f}")
        return False
    if abs(Yr) > 5.0:
        print(f"[LOOKAHEAD SKIP] Lookahead point too far off track: Xr={Xr:.2f}, Yr={Yr:.2f}")
        lookahead_point = (wp_lat, wp_lon)
    if Xr > 0.2 and abs(Yr) < 5.0 and Lf >= LOOKAHEAD_DISTANCE:
        lookahead_point = (wp_lat, wp_lon)
        
    return True

GET_CURVATURE = None
GET_OMEGA = None
GET_SPEED = None

async def nav_task(ws):
    global current_target_idx
    while True:
        if mode != "auto":
            await asyncio.sleep(0.1)
            continue
        pos = _get_latest_fix()
        yaw = _get_yaw()
        yaw_acc = _get_yaw_accuracy()
        if pos is None or yaw is None or yaw_acc > 5.0:
            await asyncio.sleep(1)
            continue

        lat, lon, precision = pos["lat"], pos["lon"], pos["precision"]
        if precision > 2 or math.isnan(precision):
            await asyncio.sleep(1)
            continue

        LAT_TO_M = 111000
        LON_TO_M = 111000 * math.cos(math.radians(lat))
        yaw_rad = math.radians(yaw)

        lookahead_point = None
        for i in range(current_target_idx, len(TARGETS)):
            wp_lat, wp_lon = TARGETS[i]
            dx = (wp_lon - lon) * LON_TO_M
            dy = (wp_lat - lat) * LAT_TO_M

            # ✅ Transform to robot frame
            Xr, Yr = get_robot_frame(dx, dy, yaw_rad)


            if not check_lookahead_point(Xr, Yr, Lf, lookahead_point, wp_lat, wp_lon): #check if lookahead point is valid, if not it will skip
                continue

            """
            if Xr < 0.2:
                print(f"[LOOKAHEAD SKIP] Lookahead point too lateral or behind: Xr={Xr:.2f}, Yr={Yr:.2f}")
                await asyncio.sleep(0.1)
                continue
            """

            if i == len(TARGETS) - 1:
                lookahead_point = (wp_lat, wp_lon)
                break

            Lf = math.hypot(Xr, Yr)
            #Lf = min(max(distance_to_waypoint * 0.4, 2.0), 8.0)
            """
            if Xr > 0.2 and abs(Yr) < 5.0 and Lf >= LOOKAHEAD_DISTANCE:
                lookahead_point = (wp_lat, wp_lon)
                break
            """
            

        if not lookahead_point:
            lookahead_point = TARGETS[-1]
            print("[LOOKAHEAD WARN] No forward-looking waypoint found. Using last target.")

        wp_lat, wp_lon = lookahead_point
        dx = (wp_lon - lon) * LON_TO_M
        dy = (wp_lat - lat) * LAT_TO_M

        # ✅ Transform again using correct yaw
        #Xr = math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy
        #Yr = -math.cos(yaw_rad) * dx + math.sin(yaw_rad) * dy
        Xr, Yr = get_robot_frame(dx, dy, yaw_rad)

        alpha = math.atan2(Yr, Xr)
        Lf = math.hypot(Xr, Yr)

        if Lf < 1e-3 or math.isnan(Lf):
            print("[LOOKAHEAD WARN] Very small or invalid Lf, skipping this cycle")
            await asyncio.sleep(0.1)
            continue

        # Curavature corresponds to the radius of the circle that the robot is following
        # Curvature is defined as 2 * sin(alpha) / Lf
        # negative curvature means the robot needs to turn left, positive curvature means right
        curvature = 2 * math.sin(alpha) / max(Lf, 1e-3)
        GET_CURVATURE = curvature

        base_speed = SPEED_LEVELS[speed_index]
        GET_SPEED = base_speed
        #omega = curvature * base_speed * 1.6

        #Calculate omega using nonlinear function based on curvature
        omega = nonlinear_omega(curvature, base_speed)
        GET_OMEGA = omega

        #Omega smoothing
        #omega_history.append(omega)
        #if len(omega_history) > 3:
        #    omega_history.pop(0)
        #omega = sum(omega_history) / len(omega_history)

        MAX_OMEGA = 1.0
        #if abs(omega) < 0.05:
        #    omega = 0.0
        omega = max(min(omega, MAX_OMEGA), -MAX_OMEGA)

        # Command format: v<speed>w<omega>
        # Sends command to robot as velocity and angular velocity
        command = f"v{base_speed:.2f}w{omega:.2f}"

        # Check distance to waypoint and scale speed if close to waypoint
        # This is to prevent overshooting the waypoint and increased accuracy
        dist_to_wp = haversine_distance(lat, lon, *TARGETS[current_target_idx])
        if dist_to_wp < 1.5 * WAYPOINT_RADIUS:
            scale = max(dist_to_wp / (1.5 * WAYPOINT_RADIUS), 0.1)
            base_speed *= scale
            print(f"[BRAKE] Scaling speed near waypoint: scale={scale:.2f}, speed={base_speed:.2f}")
        else:
            scale = 1.0

        auto_data = f"Xr = {Xr:.2f}, Yr = {Yr:.2f}, α = {math.degrees(alpha):.2f}, Lf = {Lf:.2f}, curvature = {curvature:.4f}, ω = {omega:.3f}"
        
        #debug 
        #print(f"Robot GPS: ({lat:.6f}, {lon:.6f}) | Yaw = {yaw:.2f}° (accuracy={yaw_acc:.2f})")
        #print(f"Target WP : ({wp_lat:.6f}, {wp_lon:.6f})")
        #print(f"Xr = {Xr:.2f}, Yr = {Yr:.2f}, alpha = {math.degrees(alpha):.2f}, curvature = {curvature:.3f}, ω = {omega:.2f}")

        print_data("auto", f"Waypoint {current_target_idx+1}/{len(TARGETS)} | {auto_data}\n")
        await ws.send(command)

        if dist_to_wp < WAYPOINT_RADIUS:
            current_target_idx += 1
            if current_target_idx >= len(TARGETS):
                print("✅ Reached final waypoint. Stopping.")
                await ws.send("v0.00w0.00")
                break

        await asyncio.sleep(0.2)



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
async def csv_logging_task(get_pos_func, get_yaw, robot_frame, curvature, omega, speed, csv_logger):
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

                Xr, Yr = robot_frame(lat, lon, yaw)
                curvature = GET_CURVATURE
                omega = GET_OMEGA
                speed = GET_SPEED

                UNIX_TIMESTAMP = time.time()  # Use a consistent timestamp for all points
                timestamp_converted = datetime.datetime.fromtimestamp(UNIX_TIMESTAMP)

                # time difference not setup yet
                time_diff = None
                csv_logger.add_point(timestamp_converted, mode, time_diff, dist, lon, lat, precision, yaw, yaw_acc, speed, bearing, diff, Xr, Yr, curvature, omega)
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
    while True:
        start_time = time.time()
        try:
            # Replace with your actual robot's WebSocket URI
            ws_uri = "ws://100.87.161.11:8555"
            print(f"[MAIN] Connecting to {ws_uri}...")
            async with websockets.connect(ws_uri) as ws:
                print("[MAIN] Connected. Starting tasks...")
                await asyncio.gather(
                    nav_task(ws),
                    keyboard_task(ws),
                    csv_logging_task(_get_latest_fix, _get_yaw, csv_logger)
                )

        except websockets.exceptions.ConnectionClosedError as e:
            print(f"[WEBSOCKET ERROR] Connection lost: {e}. Reconnecting in 3s...")
            traceback.print_exc()
            await asyncio.sleep(3)

        except Exception as e:
            print(f"[CONTROLLER] Unexpected error: {e}")
            traceback.print_exc()
            await asyncio.sleep(3)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"[CONTROLLER] Error: {e}\n{traceback.format_exc()}")

#ws://100.87.161.11:8555