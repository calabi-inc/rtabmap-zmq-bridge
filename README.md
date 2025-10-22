# rtabmap-zmq-bridge

Windows host publishes RealSense RGB-D over ZMQ; Docker/WSL container runs RTAB-Map and republishes SLAM topics.

## Layout
- `Dockerfile`: builds RTAB-Map and the bridge binary
- `CMakeLists.txt`: bridge build config
- `src/`: CameraZmq and bridge main
- `scripts/rs2_pub_zmq.py`: Windows-side RealSense publisher (runs outside WSL)
- `scripts/run_container.sh`: helper to run the container

## 1) Windows host → ZMQ publisher

Install dependencies: Python 3.10+, `pyrealsense2`, `opencv-python`, `pyzmq`.

Run:

```bash
python scripts/rs2_pub_zmq.py --pub tcp://0.0.0.0:5555 --w 640 --h 480 --fps 30
```

## 2) Build and run the bridge (Docker)

```bash
docker build -t rtabmap_zmq_bridge:latest .
./scripts/run_container.sh
```

By default the container subscribes to `tcp://host.docker.internal:5555` and publishes to `tcp://*:6000`.

## 3) Consuming SLAM topics

Subscribe to `tcp://127.0.0.1:6000` topics:
- `slam.tracking_pose`: current camera pose (T_wc) per odometry tick
- `slam.kf_pose`: pose of newly created keyframe node
- `slam.kf_pose_update`: pose updates to existing nodes
- `slam.kf_packet`: keyframe packet JSON + optional RGB thumbnail

## Notes
- Depth stays uint16 in PNG on the wire and is converted to meters inside the bridge.
- RTAB-Map runs in standalone C++ mode, no ROS required.
- Change `SUB` to point at a remote Windows machine if needed.
