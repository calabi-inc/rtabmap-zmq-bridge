import argparse
import json
import zmq


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--ep", default="tcp://127.0.0.1:6000", help="ZMQ endpoint to connect (bridge PUB)")
	ap.add_argument(
		"--topics",
		nargs="*",
		default=[
			"rtabmap.tracking_pose",
			"rtabmap.kf_pose",
			"rtabmap.kf_pose_update",
			"rtabmap.kf_packet",
			"rtabmap.map_correction",
		],
		help="Topic filters to subscribe",
	)
	args = ap.parse_args()

	ctx = zmq.Context.instance()
	sock = ctx.socket(zmq.SUB)
	sock.connect(args.ep)
	for t in args.topics:
		sock.setsockopt(zmq.SUBSCRIBE, t.encode("utf-8"))

	print(f"Subscribed to {args.topics} on {args.ep}")
	while True:
		parts = sock.recv_multipart()
		topic = parts[0].decode("utf-8") if parts else ""
		msg = {}
		if len(parts) >= 2:
			try:
				msg = json.loads(parts[1].decode("utf-8"))
			except Exception:
				msg = {"raw": parts[1][:64].hex() if parts[1] else ""}
		extra = f" + {len(parts)-2} bin part(s)" if len(parts) > 2 else ""
		print(topic, msg, extra, flush=True)


if __name__ == "__main__":
	main()


