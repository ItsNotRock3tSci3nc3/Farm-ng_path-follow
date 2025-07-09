import threading

class KMLLogger:
    def __init__(self, filename="track.kml"):
        self.filename = filename
        self.points = []
        self.lock = threading.Lock()
        # Write KML header
        with open(self.filename, "w") as f:
            f.write("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
<name>Robot Path</name>
<Placemark>
<LineString>
<coordinates>
""")

    def add_point(self, lon, lat):
        with self.lock:
            self.points.append((lon, lat))
            with open(self.filename, "a") as f:
                f.write(f"{lon},{lat},0\n")

    def close(self):
        with open(self.filename, "a") as f:
            f.write("""</coordinates>
</LineString>
</Placemark>
</Document>
</kml>
""")