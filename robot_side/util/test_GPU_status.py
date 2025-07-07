import torch
import time

print("Torch version:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
print("Device name:", torch.cuda.get_device_name(0))

# 创建一个简单的张量和模型
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

x = torch.rand(1000, 1000).to(device)
w = torch.rand(1000, 1000).to(device)

# 计时一次矩阵乘法，测试是否在GPU上
start = time.time()
for _ in range(100):
    y = torch.matmul(x, w)
torch.cuda.synchronize()
end = time.time()

print(f"GPU Matrix multiply done. Time: {end - start:.4f} sec")
