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

from sensors.gps_reader import get_latest_fix 
from sensors.imu_bno085_receiver import IMUReader
from sensors.oakD_bno085_receiver import oakDReceiver

import target_data.target_parser as TP

from CSV.CSVLogger import CSVLogger

test_name = "robot_track_lookahead_oakD_IMU_test_5m_yard" #CHANGE BEFORE TESTING
logging = input("Enable logging(y/n)? Press Enter to continue\n")
if logging.lower() == "y" or logging.lower() == "yes":
    csv_logger = CSVLogger(f"{test_name}.csv", True)
else:
    csv_logger = CSVLogger(f"{test_name}.csv", False)

from filters.kalman_filter import KalmanFilter2D
kf = KalmanFilter2D()

from filters.yaw_filter import YawFilter
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
#LOOKAHEAD_DISTANCE = 2  # meters (adjust as needed)
#LOOKAHEAD_DISTANCE = max(2.5, min(5.0, SPEED_LEVELS[speed_index] * 4.0))


origin_lat = None
origin_lon = None
origin_initialized = False
prev_lat = None
prev_lon = None

omega_history = []

# === IMU reader instance ===
imu_reader = IMUReader()
oakD_reader = oakDReceiver()

TARGETS = TP.get_targets()
print(TARGETS)

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

def _get_oakD_yaw():
    yaw = oakD_reader.get_yaw()
    if yaw is None:
        print("[OAK-D Receiver] Yaw data not available.")
        return None
    yaw_deg = math.degrees(yaw)
    
    # Normalize to [-180, 180]
    if yaw_deg > 180:
        yaw_deg -= 360
    elif yaw_deg < -180:
        yaw_deg += 360

    #correct yaw
    yaw_deg = -yaw_deg

    return yaw_deg

def _get_oakD_yaw_accuracy():
    return oakD_reader.get_accuracy()  # Get accuracy from OAK-D IMU interface

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

def _get_yaw_accuracy(imu):
    acc = imu.get_accuracy()
    if(imu == oakD_reader) and acc is None or acc == 0.0:
        #print(f"[OAK-D ACCURACY] ⚠️ OAK-D IMU accuracy is {acc}")
        return float('inf')  # treat 0.0 as unusably bad
    elif(imu == oakD_reader):
        return math.degrees(acc)  # Convert rad → degrees for consistency
    
    return acc  # return accuracy if not oakD imu

# ======================
# Heading/distance calculation utilities
# ======================
def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Returns the great-circle distance in meters between two lat/lon points.
    """
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = math.sin(d_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

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

    # Add strict validation here
    if not (-90 <= filtered_lat <= 90) or not (-180 <= filtered_lon <= 180):
        print(f"[GPS ERROR] Filtered GPS invalid: lat={filtered_lat}, lon={filtered_lon}")
        return None

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

last_omega = 0.0  # global or closure variable
def smooth_omega(new_omega, alpha=0.6):
    global last_omega
    smoothed = alpha * new_omega + (1 - alpha) * last_omega
    last_omega = smoothed
    return smoothed

def nonlinear_omega(curvature, base_speed):
    #Gain is used to adjust the reaction speed of the robot to curvature. More gain means more reaction speed, and the opposite is true.
    # Increase gain to keep robot on track for tighter turns, decrease if there is too much oscillation
    gain = 1.0 #1.6: former default 1.0 Optimal

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

def check_lookahead_point(Xr, Yr, Lf, wp_lat, wp_lon):
    if Xr < 0.2:
        print(f"[LOOKAHEAD SKIP] Too lateral or behind: Xr={Xr:.2f}, Yr={Yr:.2f}")
        return None
    if abs(Yr) > 5.0:
        print(f"[LOOKAHEAD SKIP] Too far off-track: Xr={Xr:.2f}, Yr={Yr:.2f}")
        return None
    if Lf >= LOOKAHEAD_DISTANCE:
        return (wp_lat, wp_lon)
    return None





async def nav_task(ws):
    global current_target_idx
    try:
        while True:
            if mode != "auto":
                await asyncio.sleep(0.1)
                continue

            pos = get_filtered_gps()
            yaw = _get_yaw()
            oakD_yaw = _get_oakD_yaw()
            oakD_yaw_acc = _get_yaw_accuracy(oakD_reader)
            yaw_acc = _get_yaw_accuracy(imu_reader)

            if pos is None or (oakD_yaw and yaw is None) or yaw_acc > 5.0: #or (oakD_yaw_acc < 0.01 and oakD_yaw_acc > 0.05):
                print(f"[AUTO] Sensor(s) input error. Pos: {pos}, Yaw: {yaw}, Yaw Acc: {fmt(yaw_acc)}, OAK-D Yaw: {oakD_yaw}")
                await asyncio.sleep(1)
                continue

            yaw = oakD_yaw #FOR TESTING ONLY
            yaw_acc = oakD_yaw_acc #FOR TESTING ONLY

            lat, lon, precision = float(f"{pos['lat']:.10f}"), float(f"{pos['lon']:.10f}"), pos["precision"]
            #lat, lon, precision = pos['lat'], pos['lon'], pos["precision"]

            #print(f"[AUTO] Lat: {lat}, Lon: {lon}")
            if precision > 2 or math.isnan(precision):
                await asyncio.sleep(1)
                continue
            if prev_lat is not None and prev_lon is not None:
                delta_meters = haversine_distance(prev_lat, prev_lon, lat, lon)
                if delta_meters > 20:  # Tune this threshold if needed
                    print(f"[GPS JUMP DETECTED] Δ={delta_meters:.2f} m — skipping")
                    continue

            # === Position transform
            LAT_TO_M = 111000
            LON_TO_M = 111000 * math.cos(math.radians(lat))
            if not (-90 <= lat <= 90):
                print(f"[LON_TO_M ERROR] Invalid lat={lat}")
                await asyncio.sleep(0.2)
                continue
            
            yaw_rad = math.radians(yaw)

            lookahead_point = None
            for i in range(current_target_idx, len(TARGETS)):
                TARGET_LAT, TARGET_LON = TARGETS[i]
                wp_lat, wp_lon = TARGETS[i]
                dx = (wp_lon - lon) * LON_TO_M
                dy = (wp_lat - lat) * LAT_TO_M
                Xr, Yr = get_robot_frame(dx, dy, yaw_rad)
                Lf = math.hypot(Xr, Yr)

                candidate = check_lookahead_point(Xr, Yr, Lf, wp_lat, wp_lon)
                if candidate:
                    lookahead_point = candidate
                    break

                if i == len(TARGETS) - 1:
                    lookahead_point = (wp_lat, wp_lon)
                    break

            if not lookahead_point:
                lookahead_point = TARGETS[-1]
                print("[LOOKAHEAD WARN] No forward-looking waypoint found. Using last target.")

            wp_lat, wp_lon = lookahead_point
            dx = (wp_lon - lon) * LON_TO_M
            dy = (wp_lat - lat) * LAT_TO_M
            Xr, Yr = get_robot_frame(dx, dy, yaw_rad)
            print(f"[AUTO] Xr = {Xr} Yr = {Yr}")
            if abs(dx) > 500 or abs(dy) > 500:
                print(f"[NAV ERROR] Unreasonable dx/dy: dx={dx:.2f}, dy={dy:.2f}")
                await asyncio.sleep(0.2)
                continue

            alpha = math.atan2(Yr, Xr)
            Lf = math.hypot(Xr, Yr)
            Lf = max(0.5, min(Lf, 10.0))
            if Lf < 1e-3 or math.isnan(Lf):
                print("[LOOKAHEAD WARN] Very small or invalid Lf, skipping this cycle")
                await asyncio.sleep(0.1)
                continue

            curvature = 2 * math.sin(alpha) / max(Lf, 1e-3)
            base_speed = SPEED_LEVELS[speed_index]
            omega = nonlinear_omega(curvature, base_speed)
            #omega = smooth_omega(nonlinear_omega(curvature, base_speed))
            omega = max(min(omega, 2.5), -2.5)

            command = f"v{base_speed:.2f}w{omega:.2f}"

            dist_to_wp = haversine_distance(lat, lon, *TARGETS[current_target_idx])
            if dist_to_wp < 1.5 * WAYPOINT_RADIUS:
                scale = max(dist_to_wp / (1.5 * WAYPOINT_RADIUS), 0.1)
                base_speed *= scale
                print(f"[BRAKE] Scaling speed near waypoint: scale={scale:.2f}, speed={base_speed:.2f}")

            print_data("auto", f"Waypoint {current_target_idx+1}/{len(TARGETS)} | Xr={Xr:.2f}, Yr={Yr:.2f}, α={math.degrees(alpha):.2f}, Lf={Lf:.2f}, curvature={curvature:.4f}, ω={omega:.3f}")
            
            try:
                
                await ws.send(command)
                csv_logging_task(get_filtered_gps, yaw, yaw_acc, oakD_yaw, oakD_yaw_acc, TARGET_LAT, TARGET_LON, Xr, Yr, curvature, omega, base_speed, csv_logger)
            except Exception as e:
                print(f"[WEBSOCKET ERROR] Failed to send command: {e}")
                break  # or continue/reconnect logic

            if dist_to_wp < WAYPOINT_RADIUS:
                current_target_idx += 1
                if current_target_idx >= len(TARGETS):
                    print("✅ Reached final waypoint. Stopping.")
                    try:
                        
                        await ws.send("v0.00w0.00")
                    except:
                        pass
                    break

            await asyncio.sleep(0.2)

    except Exception as e:
        print(f"[NAV TASK ERROR] {e}")


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
        yaw = _get_yaw()
        oakD_yaw = _get_oakD_yaw()
        csv_logging_task(get_filtered_gps, yaw, _get_yaw_accuracy(imu_reader), oakD_yaw, _get_yaw_accuracy(oakD_reader), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,csv_logger)

        
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
def csv_logging_task(get_pos_func, yaw_in, yaw_accuracy, oakD_yaw, oakD_yaw_accuracy, targ_lat, targ_lon, Xr, Yr, curvature, omega, speed, csv_logger):
    try:
        last_timestamp = None
        pos = get_pos_func()
        yaw = yaw_in
        oakD_yaw = oakD_yaw

        if pos is not None and yaw is not None:
            lat, lon = pos["lat"], pos["lon"]
            yaw_acc = yaw_accuracy
            oakD_yaw_acc = oakD_yaw_accuracy
            dist = haversine_distance(lat, lon, TARGET_LAT, TARGET_LON)
            bearing = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)

        if yaw is None:
            yaw = 0
            print(f"[CSV] ⚠️ Yaw is None. Using 0 as placeholder. Yaw accuracy: {yaw_acc:.2f}")
            

        diff = angle_diff_deg(bearing, yaw)
        precision = pos.get("precision", 0.0)

        UNIX_TIMESTAMP = time.time()
        timestamp_converted = datetime.datetime.fromtimestamp(UNIX_TIMESTAMP)

        # ➕ Compute time_diff
        if last_timestamp is None:
            time_diff = 0.0
        else:
            time_diff = UNIX_TIMESTAMP - last_timestamp
            last_timestamp = UNIX_TIMESTAMP

        csv_logger.add_point_main(
            timestamp_converted, mode, time_diff, targ_lat, targ_lon, dist, lon, lat, precision, yaw, yaw_acc, oakD_yaw, oakD_yaw_acc, speed, bearing, diff, Xr, Yr, curvature, omega
        )

    except Exception as e:
        print(f"[CSV LOGGING ERROR] {e}")
        traceback.print_exc()
        


# ======================
# Print data to console
# ======================
def print_data(mode, ext_data=None, diff = None):
    mode = mode.upper()
    pos = get_filtered_gps()
    yaw = _get_yaw()
    yaw_acc = _get_yaw_accuracy(imu_reader)
    oakD_yaw = _get_oakD_yaw()
    oakD_yaw_acc = _get_yaw_accuracy(oakD_reader)
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
            print(f"[{mode}] 📍 {ext_data} Lat={fmt(lat, 8)}°, Lon={fmt(lon, 8)}°, Dist={fmt(dist, 2)}m, Yaw = {fmt(yaw, 2)}, Yaw precision = {fmt(yaw_acc)}, oak-D Yaw = {fmt(oakD_yaw,2)}, oak-D Yaw accuracy= {fmt(oakD_yaw_acc,2)}, bearing= {fmt(bearing, 2)}, Δ={fmt(diff, 2)}, Precision={fmt(precision, 2)}")
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

#ping the webserver to automatically find the correct connection
#necessary due to ip changing (tailscale vs connection via router)
from pythonping import ping
def get_connection():
    uri1 = "100.87.161.11"
    uri2 = "192.168.0.101"
    try:
        response = ping(uri1, count=4, timeout=5)
        if response.success():
            print(f"[CONNECTION CHECK] WebSocket server {uri1} is reachable.")
            return uri1
        else:
            print(f"[CONNECTION CHECK] WebSocket server {uri1} is unreachable. Attempting connection to {uri2}")
            response = ping(uri2, count=4, timeout=5)
            if response.success():
                print(f"[CONNECTION CHECK] WebSocket server {uri2} is reachable.")
                return uri2
            else:
                print(f"[CONNECTION CHECK] WebSocket server {uri2} is unreachable.")
                return None
    except Exception as e:
        print(f"[CONNECTION CHECK ERROR] {e}")
        return None

async def main():
    global origin_initialized, origin_lat, origin_lon
    while True:
        try:
            fix = get_latest_fix()
            if not fix:
                continue

            lat = fix["lat"]
            lon = fix["lon"]
            precision = fix["precision"]

            # Require good fix before origin initialization
            if not origin_initialized and precision is not None and precision < 2.0:
                origin_lat = lat
                origin_lon = lon
                origin_initialized = True
                print(f"[ORIGIN SET] lat={origin_lat}, lon={origin_lon}")
                continue  # Skip first iteration

            port = "8555"
            ws_uri = get_connection()
            if ws_uri == None:
                print("[WS URI] URI is none, connection failed")
                return 0
            
            ws_uri = f"ws://{ws_uri}:{port}"
            print(f"[URI CHECK] {ws_uri}")

            print(f"[MAIN] Connecting to {ws_uri}...")
            async with websockets.connect(ws_uri) as ws:
                print("[MAIN] Connected. Starting tasks...")
                await asyncio.gather(
                    nav_task(ws),
                    keyboard_task(ws),
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

