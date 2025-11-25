import argparse
import json
import zmq


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--ep", default="tcp://host.docker.internal:5001", help="Endpoint to connect, e.g. tcp://<ip>:5001")
	ap.add_argument("--topic", default="test", help="Topic string to subscribe")
	args = ap.parse_args()

	ctx = zmq.Context.instance()
	sock = ctx.socket(zmq.SUB)
	sock.connect(args.ep)
	sock.setsockopt(zmq.SUBSCRIBE, args.topic.encode("utf-8"))
	print(f"SUB connected to {args.ep} topic={args.topic}")

	while True:
		parts = sock.recv_multipart()
		topic = parts[0].decode("utf-8") if parts else ""
		payload = parts[1].decode("utf-8") if len(parts) > 1 else ""
		try:
			j = json.loads(payload) if payload else {}
		except Exception:
			j = {"raw": payload[:256]}
		print(topic, j)


if __name__ == "__main__":
	main()


