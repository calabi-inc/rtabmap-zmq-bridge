import argparse
import json
import time
import zmq


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--ep", default="tcp://0.0.0.0:5001", help="Endpoint to bind, e.g. tcp://0.0.0.0:5001")
	ap.add_argument("--topic", default="test", help="Topic string")
	ap.add_argument("--hz", type=float, default=2.0, help="Publish rate (Hz)")
	args = ap.parse_args()

	ctx = zmq.Context.instance()
	sock = ctx.socket(zmq.PUB)
	sock.bind(args.ep)
	topic = args.topic.encode("utf-8")
	print(f"PUB bound on {args.ep} topic={args.topic} rate={args.hz}Hz")

	i = 0
	period = 1.0 / max(0.001, args.hz)
	while True:
		now_ns = time.time_ns()
		msg = {"seq": i, "ts_ns": now_ns}
		sock.send_multipart([topic, json.dumps(msg).encode("utf-8")])
		if i % 20 == 0:
			print(f"sent {i}: {msg}")
		i += 1
		time.sleep(period)


if __name__ == "__main__":
	main()


