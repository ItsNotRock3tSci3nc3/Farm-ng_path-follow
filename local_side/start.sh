#!/bin/bash

# 生成设备环境变量文件
python detect_devices.py

# 启动容器
docker compose run --rm local_side
