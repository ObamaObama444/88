#!/usr/bin/env bash
set -eo pipefail

cd /src
source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true

echo "[1/2] Load map for robot navigation and set start pose..."
/src/load_map_and_start.sh

echo "[2/2] Start Foxglove display map..."
/src/start_foxglove_with_saved_map.sh

echo "[DONE]"
echo "Robot navigation map: /map"
echo "Foxglove display map: /map_display"
echo "Foxglove URL: ws://ROBOT_IP:8765"
