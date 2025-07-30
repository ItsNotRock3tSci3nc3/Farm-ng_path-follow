import csv
import os
from pathlib import Path
from datetime import datetime

class CSVLogger:
    def __init__(self, filename= None, logging = True):
        self.logging = logging
        if self.logging:
            self.file = open(filename, "w", newline='')
        if not self.logging:
            print("[CSV LOGGER] Logging is not enabled.")
            return
        base_dir = Path("CSV_files")
        base_dir.mkdir(exist_ok=True)
        
        if filename.endswith(".csv"):
            filename = filename[:-4]  # Remove .csv if present
            print(f"[CSVLogger] Removed .csv from filename: {filename}")
        folder = base_dir / filename
        folder.mkdir(parents=True, exist_ok=True)

        # Find unique filename within the folder
        csv_name = f"{filename}.csv"
        file_path = folder / csv_name
        count = 1
        while file_path.exists():
            csv_name = f"{filename[:-4]}_run{count}.csv"
            file_path = folder / csv_name
            count += 1

        self.filename = file_path
        self.file = open(self.filename, "w", newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow(["timestamp", "Robot state", "Time difference", "distance", "latitude", "longitude", "RTK precision", "yaw", "yaw precision", "Speed", "heading", "delta", "Xr", "Yr", "Curvature", "Omega"])
        print(f"[CSVLogger] Logging to {self.filename}")

    def add_point(self, time = "NULL", state = "NULL", time_diff = "NULL", dist = 0, lon = 0, lat = 0, prec = 0, yaw = 0, yaw_acc = -1, speed = -1, bearing = -1, delta = 0, Xr = 0, Yr = 0, curvature = 0, omega = 0):
        if not self.logging:
            return
        self.writer.writerow([time, state, time_diff, dist, lat, lon, prec, yaw, yaw_acc, speed, bearing, delta, Xr, Yr, curvature, omega])

    def close(self):
        if not self.logging:
            return
        self.file.close()
