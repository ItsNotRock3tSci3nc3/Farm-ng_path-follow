import socket
from farm_ctrl import send_key  # your existing WASD control API

_HOST="0.0.0.0"; _CTRL_PORT=7000
srv=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
srv.bind((_HOST,_CTRL_PORT))
srv.listen(1)
print(f"[CTRL‑RECV] listening on {_CTRL_PORT} …")
conn,_=srv.accept()
print("Control client connected")
while True:
    cmd=conn.recv(1)
    if not cmd:
        break
    send_key(cmd.decode())