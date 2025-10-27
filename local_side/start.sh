#!/bin/bash

#Ensure pop up permissions are enabled
xhost +local:root

# 生成设备环境变量文件
python3 detect_devices.py

# 启动容器
docker compose run --rm local_side
