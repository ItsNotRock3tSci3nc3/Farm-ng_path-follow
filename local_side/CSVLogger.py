import csv
import os
from pathlib import Path
from datetime import datetime

class CSVLogger:
    def __init__(self, filename="robot_track", logging = True):
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

        self.writer.writerow(["timestamp", "distance", "latitude", "longitude", "yaw", "yaw precision", "heading", "delta", "precision"])
        print(f"[CSVLogger] Logging to {self.filename}")

    def add_point(self, time, dist, lon, lat, yaw, yaw_acc, bearing, delta=None, prec=None):
        if not self.logging:
            return
        self.writer.writerow([time, dist, lat, lon, yaw, yaw_acc, bearing, delta, prec])

    def close(self):
        if not self.logging:
            return
        self.file.close()
