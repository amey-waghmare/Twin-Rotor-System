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
count = 0

pitch = 0.3

while True:
    try:
        data = sock.recv(1024, socket.MSG_DONTWAIT)
    except BlockingIOError as e:
        data = "0"
        
    if not isinstance(data, str):
        data = data.decode("utf-8")
    
    if data == "pitrada":
        pitch = pitch + 0.01
    elif data == "pitradz":
        pitch = pitch - 0.01
    
    count = count + 1
    
    
    print(count, data, pitch, type(data))
