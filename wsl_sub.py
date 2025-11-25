import zmq

HOST_IP = "172.27.240.1"   # from `default via` route

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect(f"tcp://{HOST_IP}:5555")
sock.setsockopt_string(zmq.SUBSCRIBE, "")  # subscribe to all

print(f"SUB connected to tcp://{HOST_IP}:5555, waiting...")
while True:
    msg = sock.recv()
    print("received:", msg)