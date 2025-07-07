import depthai as dai

devices = dai.Device.getAllAvailableDevices()
print(f"Total devices: {len(devices)}")
for d in devices:
    print(f"- {d.name} | State: {d.state.name} | Protocol: {d.protocol.name} | MXID: {d.mxid}")
