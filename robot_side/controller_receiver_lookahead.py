import asyncio
import argparse
import re
from pathlib import Path

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file

import websockets

# -------------------------------
# 控制速度档位设置
# -------------------------------
SPEED_LEVELS = [0.2, 0.4, 0.6, 0.8, 0.9, 1.0]  # 速度百分比
DEFAULT_SPEED_INDEX = 2
current_speed_index = DEFAULT_SPEED_INDEX

# 动态速度（0.1~0.9）：由客户端浮点消息设置，优先生效
dynamic_speed = None   # None 表示未启用

def get_active_speed() -> float:
    """返回当前应使用的速度（优先 dynamic_speed，否则档位速度）"""
    return dynamic_speed if dynamic_speed is not None else SPEED_LEVELS[current_speed_index]


def update_twist_with_key(twist: Twist2d, key: str) -> Twist2d:
    """
    解析来自客户端的消息：
    1) 若是 0.1~0.9 浮点 → 仅更新 dynamic_speed
    2) 若是 w/a/s/d/空格/1~6 → 生成 Twist2d
    """
    global current_speed_index, dynamic_speed

    key = key.strip().lower()
    if not key:
        return twist

    # Check for lookahead command format: v{linear}w{angular}, e.g. v0.60w0.15
    m = re.match(r"v([-+]?\d*\.?\d+)w([-+]?\d*\.?\d+)", key)
    if m:
        try:
            linear = float(m.group(1))
            angular = float(m.group(2))
            twist.linear_velocity_x = linear
            twist.angular_velocity = angular
            print(f"🛰️  Lookahead command: lin={linear:.2f}, ang={angular:.2f}")
            return twist
        except Exception as e:
            print(f"❌ Failed to parse lookahead command: {e}")
            return twist

    # ① 尝试把消息解析为浮点速度
    try:
        val = float(key)
        if 0.05 <= val <= 1.0:
            dynamic_speed = max(0.1, min(val, 0.9))  # 限定 0.1~0.9
            print(f"⚡  Dynamic speed set to {dynamic_speed:.2f}")
            return twist          # 只更新速度，不发运动指令
    except ValueError:
        pass  # 不是浮点数，继续按字符指令解析

    # ② 如果是数字 1~6：切档位 & 清除 dynamic_speed
    if key in "123456":
        current_speed_index = int(key) - 1
        dynamic_speed = None
        print(f"⚙️  Speed level set to {current_speed_index + 1} "
              f"({SPEED_LEVELS[current_speed_index] * 100:.0f} %)")
        return twist

    # ③ 方向/急停指令
    speed = get_active_speed()
    twist.linear_velocity_x = 0.0
    twist.angular_velocity = 0.0

    if key == "w":
        twist.linear_velocity_x = speed
        print(f"🚀 Forward @ {speed:.2f}")
    elif key == "s":
        twist.linear_velocity_x = -speed
        print(f"🚀 Backward @ {speed:.2f}")
    elif key == "a":
        twist.angular_velocity = speed
        print(f"🔄 Left @ {speed:.2f}")
    elif key == "d":
        twist.angular_velocity = -speed
        print(f"🔄 Right @ {speed:.2f}")
    elif key == " ":
        dynamic_speed = None      # 急停也清掉 dynamic_speed
        twist.linear_velocity_x = 0.0
        twist.angular_velocity = 0.0
        print("🛑 Emergency stop")
    elif key == "q":
        print("👋 Exit requested")
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
        print(f"✅ Connected from {websocket.remote_address}")
        twist = Twist2d()

        try:
            async for message in websocket:
                twist = update_twist_with_key(twist, message)
                print(f"📨 '{message}' → lin={twist.linear_velocity_x:.2f}, ang={twist.angular_velocity:.2f}")
                await client.request_reply("/twist", twist)  # ← 和底层总线交互
        except websockets.exceptions.ConnectionClosedError:
            print(f"❌ Connection closed from {websocket.remote_address}")
        except Exception as e:
            print(f"[!] Unexpected error: {e}")

    print(f"🛰️  WebSocket server on 0.0.0.0:{port}")
    async with websockets.serve(handle_connection, "0.0.0.0", port):
        await asyncio.Future()  # run forever


def start_ctrl_server():
    service_config = Path("service_config.json")
    port = 8555
    asyncio.run(start_server(service_config, port))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Farm-ng Controller Receiver (WebSocket Server)")
    parser.add_argument("--service-config", type=Path, default="service_config.json",
                        help="Path to CANBus service config.")
    parser.add_argument("--port", type=int, default=8555, help="WebSocket port")
    args = parser.parse_args()
    asyncio.run(start_server(args.service_config, args.port))
