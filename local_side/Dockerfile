FROM python:3.10

# 安装系统依赖（OpenCV GUI 支持）
RUN apt-get update && apt-get install -y \
    libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 \
    && rm -rf /var/lib/apt/lists/*

# 单独复制依赖文件（避免每次代码改动就重装）
COPY requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# 再复制项目代码
COPY . /app
WORKDIR /app

# 默认启动脚本
CMD ["python", "main_controller.py"]
