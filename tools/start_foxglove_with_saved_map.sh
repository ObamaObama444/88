#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true

mkdir -p /src/log

echo "[INFO] Killing old visual map publishers..."
pkill -f publish_map_display.py || true
pkill -f publish_saved_map_force.py || true
pkill -f force_map_republish.py || true

echo "[INFO] Starting /map_display publisher..."
nohup python3 /src/publish_map_display.py > /src/log/map_display.log 2>&1 &

sleep 2

echo "[INFO] Checking /map_display..."
ros2 topic echo /map_display --once | head -25 || {
  echo "[ERROR] /map_display not publishing"
  tail -50 /src/log/map_display.log || true
  exit 1
}

echo "[INFO] Restarting foxglove_bridge..."
pkill -f foxglove_bridge || true
sleep 1

nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0 \
  > /src/log/foxglove_bridge_manual.log 2>&1 &

sleep 3

echo "[INFO] Bridge status:"
ros2 node list | grep foxglove || true
ss -lntp | grep 8765 || true

echo "[INFO] /map_display info:"
ros2 topic info /map_display -v | head -80

echo "[INFO] DONE"
echo "[INFO] In Foxglove use OccupancyGrid topic: /map_display"
