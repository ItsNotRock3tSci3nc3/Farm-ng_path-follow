import csv

class CSVLogger:
    def __init__(self, filename="robot_track.csv"):
        self.filename = filename
        self.file = open(self.filename, "w", newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(["latitude", "longitude"])  # Header row

    def add_point(self, lon, lat):
        self.writer.writerow([lat, lon])  # Note: QGIS expects lat, lon order by default

    def close(self):
        self.file.close()
