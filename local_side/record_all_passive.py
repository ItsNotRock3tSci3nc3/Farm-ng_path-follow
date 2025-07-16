import asyncio
import cv2
import math
import websockets
import pygame
import time
import datetime
# from imu_receiver import get_yaw
from gps_reader import get_latest_fix 
from imu_bno085_receiver import IMUReader
imu_reader = IMUReader()

from CSVLogger import CSVLogger
csv_logger = CSVLogger("robot_track_testrun1.csv") #CHANGE BEFORE TESTING

from kalman import KalmanFilter2D
kf = KalmanFilter2D()

from yaw_filter import YawFilter
yaw_filter = YawFilter(alpha=0.7)  # Adjust alpha as needed

TARGET_LAT = 38.94123854
TARGET_LON = -92.31853863600745

def _get_yaw():
    raw_yaw = imu_reader.get_yaw()
    if raw_yaw is None:
        print(f"[IMU] ⚠️ Yaw is None, setting yaw to 0.")
        raw_yaw = 0.0  # Default to 0 if yaw is None
    yaw_accuracy = imu_reader.get_accuracy()
    
    # Discard if yaw accuracy is poor
    if yaw_accuracy < 2:
        print(f"[IMU] ⚠️ Yaw is None due to low accuracy({yaw_accuracy}).")
    #    return yaw_filter.filtered_yaw  # Return last valid filtered yaw

    return yaw_filter.update(raw_yaw, yaw_accuracy)
def _get_yaw_accuracy():
    return imu_reader.get_accuracy()  # Get accuracy from IMU interface

def _get_latest_fix():
    # TODO: Replace with real GPS module

    # return {"lat": 38.94209340545889, "lon": -92.31854182078087, "precision": 0.8} # North
    # return {"lat": 38.940795965552134, "lon": -92.3185432362358, "precision": 0.5}  # South
    # return {"lat": 38.941245696031814, "lon": -92.31752127781246, "precision": 0.1}  # East
    # return {"lat": 38.94124074, "lon": -92.3189133776869, "precision": 0.05}  # West
    # return {"lat": 38.94208239631051, "lon": -92.31732877594878, "precision": 0.05}  # Northeast
    # return {"lat": 38.940759084223465, "lon": -92.31987659473259, "precision": 0.05}  # Southwest

    return get_latest_fix(debug=False)

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
    diff = bearing - yaw
    while diff < -180:
        diff += 360
    while diff > 180:
        diff -= 360
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

async def csv_logging_task(timestamp,get_pos_func, get_yaw, csv_logger):
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

                diff = angle_diff_deg(bearing, yaw)
                precision = pos.get("precision", 0.0)
                bearing = bearing_deg(lat, lon, TARGET_LAT, TARGET_LON)
                if bearing > 180:
                    bearing -= 360

                csv_logger.add_point(timestamp, 0, lon, lat, yaw, yaw_acc, 0, diff, precision)
            await asyncio.sleep(1)
    finally:
        csv_logger.close()

async def main():
    
        await asyncio.gather(

            csv_logging_task(time.time(), get_filtered_gps, _get_yaw, csv_logger)
        )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"Error: {e}")