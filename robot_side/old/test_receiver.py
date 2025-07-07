# robot_side/simple_ws_receiver.py
import asyncio
import websockets

async def echo_server(websocket):
    async for message in websocket:
        print(f"[Robot] Received: {message}")

async def main():
    async with websockets.serve(echo_server, "0.0.0.0", 8765):
        print("🚀 Robot WebSocket Receiver started on ws://0.0.0.0:8765")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
