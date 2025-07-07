# local_side/simple_ws_sender.py
import asyncio
import websockets

async def send_messages():
    uri = "ws://localhost:8765"  # 替换为你 robot 端的 IP，例如 ws://192.168.1.123:8765

    while True:
        try:
            # 尝试连接到 WebSocket 服务器
            async with websockets.connect(uri) as websocket:
                print("🛰️ Connected to robot.")
                while True:
                    msg = input("Enter message to send: ")
                    await websocket.send(msg)
                    print(f"[Local] Sent: {msg}")
        except (websockets.ConnectionClosedError, websockets.ConnectionClosedOK):
            print("❗ Connection closed, retrying in 2 seconds...")
            await asyncio.sleep(2)
        except Exception as e:
            print(f"⚠️ Error: {e}, retrying in 2 seconds...")
            await asyncio.sleep(2)

if __name__ == "__main__":
    asyncio.run(send_messages())
