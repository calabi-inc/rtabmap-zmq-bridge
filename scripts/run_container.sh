#!/usr/bin/env bash
set -euo pipefail

docker rm -f rtab-zmq >/dev/null 2>&1 || true

docker run --rm --name rtab-zmq \
  -p 6000:6000 \
  -e SUB="${SUB:-tcp://172.27.240.1:5555}" \
  -e PUB="${PUB:-tcp://*:6000}" \
  -e FULL_RGBD="${FULL_RGBD:-true}" \
  rtabmap_zmq_bridge:latest


