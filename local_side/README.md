# 🧭 Autonomous Navigation System - Farm Robot

本项目分为两个模块：

* 📍 `local_side`：运行在笔记本电脑上，负责位置与姿态感知 + 导航指令计算
* 🤖 `robot_side`：运行在机器人上，负责接收远程指令并执行移动行为

---

## 🧩 项目结构说明

```
.
├── local_side
│   ├── gps_reader.py                # 从 RTK 模块读取 GPS 坐标
│   ├── imu_bno085_receiver.py       # 从 ESP32 读取 BNO085 姿态数据
│   ├── main_controller.py           # 导航主控模块：融合 GPS+IMU 计算并发送控制命令
│   ├── service_config.json          # FarmNG CAN 总线配置文件
│   ├── docker-compose.yml / Dockerfile
│   └── start.sh / run.sh            # 启动入口脚本
│
├── robot_side
│   ├── controller_receiver.py       # WebSocket 服务：接收远程控制指令
│   ├── main_robot.py                # 启动 robot-side 功能（如控制电机）
│   ├── service_config.json          # FarmNG 配置
│   ├── docker-compose.yml / Dockerfile
│   └── start.sh / run.sh            # 启动入口脚本

```

---

## 🚀 系统流程概览

### 🧠 Local Side（导航计算端）

1. **连接设备**

   * ✅ 连接 RTK：获取当前 GPS 坐标
   * ✅ 连接 ESP32：获取 BNO085 方向角（Yaw）
2. **融合导航**

   * 确定当前位置和朝向
   * 计算目标点方向与移动指令（如先转向、再前进）
3. **发送控制**

   * 通过 WebSocket 将控制命令（如 `w`, `a`, `s`, `d`）发送至 Robot Side

### 🤖 Robot Side（执行端）

1. **接收控制**

   * 启动 `controller_receiver.py`，监听控制指令
2. **执行动作**

   * 控制机器人前进、后退、转向等行为
   * 使用 FarmNG 的 CAN 总线发送 `Twist2d` 指令

---

## 🛠️ 启动方法

### 机器人端（Robot Side）

```bash
cd robot_side
bash start.sh
bash run.sh
```

将运行：

* `controller_receiver.py`：开启 WebSocket 服务，等待控制指令

---

### 本地端（Local Side）

```bash
cd local_side
bash start.sh
```

将自动运行：

* `gps_reader.py`：监听 RTK
* `imu_bno085_receiver.py`：监听 ESP32 IMU
* `main_controller.py`：融合计算导航命令并发送控制



## 📡 网络连接建议

* 可以使用手机提供的热点作为临时局域网，让 RTK、Farm-NG和笔记本加入
* 若 RTK 需要连接基站服务（如 NTRIP），建议手机共享网络给 RTK

---

## 📎 其他说明

* 所有串口设备通过 `udev` 脚本自动识别，RTK 和 ESP32 分别映射为 `/dev/rtk`, `/dev/esp32`
* 控制速度可通过按键 1\~6 调整， 50% ~ 100% 速度