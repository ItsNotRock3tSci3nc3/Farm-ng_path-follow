# 🧭 Autonomous Navigation System - Farm Robot

This project is divided into two modules:

* 📍 `local_side`: Runs on the laptop, responsible for position and orientation sensing + navigation command calculation
* 🤖 `robot_side`: Runs on the robot, responsible for receiving remote commands and executing movement actions

---

## 🧩 Project Structure Overview

```
.
├── local_side
│   ├── gps_reader.py                # Reads GPS coordinates from the RTK module
│   ├── imu_bno085_receiver.py       # Reads BNO085 orientation data from ESP32
│   ├── main_controller.py           # Main navigation controller: fuses GPS+IMU and sends control commands
│   ├── service_config.json          # FarmNG CAN bus configuration file
│   ├── docker-compose.yml / Dockerfile
│   └── start.sh / run.sh            # Startup scripts
│
├── robot_side
│   ├── controller_receiver.py       # WebSocket service: receives remote control commands
│   ├── main_robot.py                # Starts robot-side functions (e.g., motor control)
│   ├── service_config.json          # FarmNG configuration
│   ├── docker-compose.yml / Dockerfile
│   └── start.sh / run.sh            # Startup scripts

```

---

## 🚀 System Flow Overview

### 🧠 Local Side (Navigation Calculation)

1. **Connect Devices**

   * ✅ Connect RTK: Get current GPS coordinates
   * ✅ Connect ESP32: Get BNO085 yaw (heading)
2. **Navigation Fusion**

   * Determine current position and heading
   * Calculate target direction and movement commands (e.g., turn first, then move forward)
3. **Send Control**

   * Send control commands (such as `w`, `a`, `s`, `d`) to Robot Side via WebSocket

### 🤖 Robot Side (Execution)

1. **Receive Control**

   * Start `controller_receiver.py` to listen for control commands
2. **Execute Actions**

   * Control the robot to move forward, backward, turn, etc.
   * Use FarmNG's CAN bus to send `Twist2d` commands

---

## 🛠️ Startup Instructions

### Robot Side

```bash
cd robot_side
bash start.sh
bash run.sh
```

This will run:

* `controller_receiver.py`: Starts the WebSocket service and waits for control commands

---

### Local Side

```bash
cd local_side
bash start.sh
```

This will automatically run:

* `gps_reader.py`: Listens to RTK
* `imu_bno085_receiver.py`: Listens to ESP32 IMU
* `main_controller.py`: Fuses and calculates navigation commands and sends controls



## 📡 Network Connection Recommendations

* You can use a mobile hotspot as a temporary LAN, allowing RTK, Farm-NG, and the laptop to join
* If the RTK needs to connect to a base station service (such as NTRIP), it is recommended to share the network from your phone to the RTK

---

## 📎 Other Notes

* All serial devices are automatically recognized via `udev` scripts, with RTK and ESP32 mapped to `/dev/rtk` and `/dev/esp32` respectively
* Control speed can be adjusted with keys 1~6, corresponding to 50% ~ 100% speed