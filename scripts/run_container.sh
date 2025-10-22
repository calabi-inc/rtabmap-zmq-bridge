#!/usr/bin/env bash
set -euo pipefail

docker rm -f rtab-zmq >/dev/null 2>&1 || true

docker run --rm --name rtab-zmq \
  --network host \
  -e SUB="${SUB:-tcp://host.docker.internal:5555}" \
  -e PUB="${PUB:-tcp://*:6000}" \
  rtabmap_zmq_bridge:latest


