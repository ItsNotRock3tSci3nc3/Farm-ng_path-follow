import numpy as np
import serial
import pynmea2
import os

SERIAL_PORT = os.getenv("RTK_PORT")
BAUDRATE = 9600

def estimate_precision_cm(hdop, gps_qual):
    try:
        hdop = float(hdop)
    except (ValueError, TypeError):
        return np.nan

    if gps_qual == "5":
        return 1
    elif gps_qual == "4":
        return 2
    elif gps_qual == "2":
        return int(hdop * 200)
    elif gps_qual == "1":
        return int(hdop * 500)
    else:
        return np.nan

def get_latest_fix(debug=False):
    """
    读取一次串口数据，返回经纬度 + 精度 + debug信息:
    {
        'lat': float,
        'lon': float,
        'precision': float,
        'debug': { ... }  # 包含全部字段
    }
    """
    try:
        ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
    except Exception as e:
        print(f"无法打开串口: {e}")
        return None

    while True:
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line.startswith("$"):
                continue

            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue

            if isinstance(msg, pynmea2.types.talker.GGA):
                lat = msg.latitude
                lon = msg.longitude
                hdop = msg.horizontal_dil
                gps_qual = str(msg.gps_qual)
                precision_cm = estimate_precision_cm(hdop, gps_qual)

                result = {
                    "lat": lat,
                    "lon": lon,
                    "precision": precision_cm,
                    "debug": {
                        "raw": line,
                        "timestamp": str(msg.timestamp) if msg.timestamp else None,
                        "hdop": hdop,
                        "gps_qual": gps_qual,
                        "num_sats": msg.num_sats,
                        "altitude": msg.altitude,
                        "altitude_units": msg.altitude_units,
                        "ref_station_id": msg.ref_station_id
                    }
                }

                if debug:
                    print("==== GGA 原始调试信息 ====")
                    for k, v in result["debug"].items():
                        print(f"{k:<15}: {v}")

                return result

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"读取错误: {e}")
            return None
