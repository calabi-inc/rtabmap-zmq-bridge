# rtabmap-zmq-bridge

Windows host publishes RealSense RGB-D over ZMQ; Docker/WSL container runs RTAB-Map and republishes rtabmap.* topics.

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

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `SUB` | `tcp://host.docker.internal:5555` | ZMQ endpoint to subscribe for RGB-D input |
| `PUB` | `tcp://*:6000` | ZMQ endpoint to publish RTAB-Map output topics |
| `FULL_RGBD` | `false` | Set to `"true"` for full-resolution RGB-D in `kf_packet` |

Example with full RGB-D enabled:
```bash
FULL_RGBD=true ./scripts/run_container.sh
```

## 3) Consuming topics

Subscribe to `tcp://127.0.0.1:6000` topics:
- `rtabmap.tracking_pose`: current camera pose (T_wc) per odometry tick
- `rtabmap.kf_pose`: pose of newly created keyframe node
- `rtabmap.kf_pose_update`: pose updates to existing nodes
- `rtabmap.kf_packet`: keyframe packet with metadata, pose, and image data
  - **Default mode** (`FULL_RGBD=false`): JSON metadata + RGB thumbnail (320px, JPEG 70% quality)
  - **Full RGB-D mode** (`FULL_RGBD=true`): JSON metadata + full RGB (JPEG 85% quality) + full depth (PNG uint16)
- `rtabmap.map_correction`: map-to-odometry transform for drift correction

## Notes
- Depth stays uint16 in PNG on the wire and is converted to meters inside the bridge.
- RTAB-Map runs in standalone C++ mode, no ROS required.
- Change `SUB` to point at a remote Windows machine if needed.
