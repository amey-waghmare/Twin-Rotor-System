import time
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "0.0.0.0"
UDP_PORT = 5051
sock.setblocking(False)
sock.bind((UDP_IP,UDP_PORT))
## Enter above UDP_PORT in mobile app
time.sleep(1)

data = b'0'

while True:
    try:
        data = sock.recv(1024, socket.MSG_DONTWAIT)
    except BlockingIOError as e:
        pass
    if not isinstance(data, str):
        data = data.decode("utf-8")

    print(data, type(data))
