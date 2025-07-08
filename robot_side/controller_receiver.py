import asyncio
import argparse
from pathlib import Path

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file

import websockets

# -------------------------------
# Speed level settings
# -------------------------------
SPEED_LEVELS = [0.2, 0.4, 0.6, 0.8, 0.9, 1.0]  # Speed percentage
DEFAULT_SPEED_INDEX = 2
current_speed_index = DEFAULT_SPEED_INDEX


def update_twist_with_key(twist: Twist2d, key: str) -> Twist2d:
    """
    Receives character commands and updates the Twist2d control signal.
    Supports w/a/s/d for direction control, 1~6 to switch speed, and spacebar for emergency stop.
    """
    global current_speed_index

    key = key.lower()
    if not key:
        return twist

    if key in "123456":
        current_speed_index = int(key) - 1
        print(f"⚙️  Speed level set to {current_speed_index + 1} ({SPEED_LEVELS[current_speed_index] * 100:.2f} %)")
        return twist

    speed = SPEED_LEVELS[current_speed_index]
    twist.linear_velocity_x = 0.0
    twist.angular_velocity = 0.0

    if key == "w":
        twist.linear_velocity_x = speed
        print(f"🚀 Moving forward at {speed*100:.2f} %")
    elif key == "s":
        twist.linear_velocity_x = -speed
        print(f"🚀 Moving backward at {speed*100:.2f} %")
    elif key == "a":
        twist.angular_velocity = speed
        print(f"🚀 Turning left at {speed*100:.2f} %")
    elif key == "d":
        twist.angular_velocity = -speed
        print(f"🚀 Turning right at {speed*100:.2f} %")
    elif key == "1":
        current_speed_index = 0
        print(f"⚙️  Speed level set to 1 ({SPEED_LEVELS[0]:.2f} %)")
    elif key == "2":
        current_speed_index = 1
        print(f"⚙️  Speed level set to 2 ({SPEED_LEVELS[1]:.2f} %)")
    elif key == "3":
        current_speed_index = 2
        print(f"⚙️  Speed level set to 3 ({SPEED_LEVELS[2]:.2f} %)")
    elif key == "4":
        current_speed_index = 3
        print(f"⚙️  Speed level set to 4 ({SPEED_LEVELS[3]:.2f} %)")
    elif key == "5":
        current_speed_index = 4
        print(f"⚙️  Speed level set to 5 ({SPEED_LEVELS[4]:.2f} %)")
    elif key == "6":
        current_speed_index = 5
        print(f"⚙️  Speed level set to 6 ({SPEED_LEVELS[5]:.2f} %)")
    elif key == " ":
        twist.linear_velocity_x = 0.0
        twist.angular_velocity = 0.0
        print("🛑 Emergency stop triggered.")
    elif key == "q":
        print("👋 Exiting receiver.")
        twist.linear_velocity_x = 0.0
        twist.angular_velocity = 0.0
        return twist
    else:
        print(f"❓ Unknown command: '{key}'")

    return twist


async def start_server(service_config_path: Path, port: int):
    config = proto_from_json_file(service_config_path, EventServiceConfig())
    print(f"📡 Using CANBus config: {config}")
    client = EventClient(config)

    async def handle_connection(websocket):
        """
        Inner function, accesses outer client.
        Receives control characters and sends Twist2d to the CAN bus.
        """
        print(f"✅ Connected from {websocket.remote_address}")
        twist = Twist2d()

        try:
            async for message in websocket:
                twist = update_twist_with_key(twist, message)
                print(f"📨 Received '{message}' → linear={twist.linear_velocity_x:.2f}, angular={twist.angular_velocity:.2f}")
                await client.request_reply("/twist", twist)  # Send to CAN bus !! Comment out for testing, uncomment for running
        except websockets.exceptions.ConnectionClosedError:
            print(f"❌ Connection closed from {websocket.remote_address}")
        except Exception as e:
            print(f"[!] Unexpected error: {e}")

    print(f"🛰️  Starting WebSocket server on 0.0.0.0:{port}...")
    async with websockets.serve(handle_connection, "0.0.0.0", port):
        await asyncio.Future()  # run forever

def start_ctrl_server():
    service_config = Path("service_config.json")
    port = 8555
    asyncio.run(start_server(service_config, port))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Farm-ng Controller Receiver (WebSocket Server)")
    parser.add_argument("--service-config", type=Path, default="service_config.json", help="Path to CANBus service config.")
    parser.add_argument("--port", type=int, default=8555, help="WebSocket port to listen on")

    args = parser.parse_args()
    asyncio.run(start_server(args.service_config, args.port))
