FROM python:3.10

# 系统依赖：包括 libusb, udev, OpenCV GUI 等
RUN apt-get update && apt-get install -y \
    python3-pip cmake git libusb-1.0-0-dev libudev-dev \
    libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 包
COPY requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY . /app
WORKDIR /app

# 启动脚本
CMD ["bash"]
