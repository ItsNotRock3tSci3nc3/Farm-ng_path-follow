import depthai as dai

def list_all_devices():
    infos = dai.XLinkConnection.getAllConnectedDevices()
    if not infos:
        print("❌ 没有检测到任何 OAK 设备")
        return
    print("✅ 发现的 OAK 设备：")
    for info in infos:
        print(f"  MXID: {info.getMxId():<22}  Name: {info.name:<10}  State: {info.state.name}")

list_all_devices()