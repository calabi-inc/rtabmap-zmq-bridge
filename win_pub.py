# win_pub.py
import time
import zmq

ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)

# IMPORTANT: bind to 0.0.0.0 so other machines/WSL can reach it
sock.bind("tcp://0.0.0.0:5555")
print("PUB bound on tcp://0.0.0.0:5555")

i = 0
while True:
    msg = f"hello {i}".encode()
    print("sending:", msg)
    sock.send(msg)
    i += 1
    time.sleep(1)
