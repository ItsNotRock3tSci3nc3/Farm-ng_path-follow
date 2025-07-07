import asyncio
import cv2
import websockets

async def send_commands(uri: str):
    print(f"🚀 Connecting to robot at {uri}")
    async with websockets.connect(uri) as ws:
        print("✅ Connected. Press keys (w/a/s/d, 1~6, space, q to quit)...")

        cv2.namedWindow("Remote Keyboard")  # 创建虚拟窗口监听键盘
        while True:
            key = cv2.waitKey(10)  # 每 10ms 轮询一次键盘输入
            if key == -1:
                await asyncio.sleep(0.01)
                continue

            char = chr(key).lower() if 32 <= key <= 126 else ''
            if not char:
                continue

            await ws.send(char)
            print(f"→ Sent: {char}")

            if char == 'q':
                print("👋 Exiting sender.")
                break

if __name__ == "__main__":
    uri = "ws://localhost:8765"
    asyncio.run(send_commands(uri))
