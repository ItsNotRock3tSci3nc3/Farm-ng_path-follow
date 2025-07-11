import csv

class CSVLogger:
    def __init__(self, filename="robot_track.csv"):
        self.filename = filename
        self.file = open(self.filename, "w", newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(["distance", "latitude", "longitude", "yaw", "yaw precision", "heading", "delta", "precision"])  # Header row

    def add_point(self, dist, lon, lat, yaw, yaw_acc, bearing, delta=None, prec=None):
        self.writer.writerow([dist, lat, lon, yaw, yaw_acc, bearing, delta, prec])  # Note: QGIS expects lat, lon order by default

    def close(self):
        self.file.close()
