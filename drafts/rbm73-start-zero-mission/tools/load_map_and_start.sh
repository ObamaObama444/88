#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true

MAPS_DIR="/src/maps"
POSEGRAPH="${MAPS_DIR}/final_posegraph"
START_FILE="${MAPS_DIR}/start_pose.yaml"

echo "[INFO] Loading saved map and setting start pose..."

if [[ ! -f "$START_FILE" ]]; then
  echo "[ERROR] No start pose file: $START_FILE"
  exit 1
fi

if [[ ! -f "${POSEGRAPH}.posegraph" || ! -f "${POSEGRAPH}.data" ]]; then
  echo "[ERROR] No saved posegraph files:"
  echo "  ${POSEGRAPH}.posegraph"
  echo "  ${POSEGRAPH}.data"
  exit 1
fi

read START_X START_Y START_YAW < <(python3 - <<'PY'
import yaml
with open("/src/maps/start_pose.yaml", "r") as f:
    s = yaml.safe_load(f)["start"]
print(float(s["x"]), float(s["y"]), float(s["yaw"]))
PY
)

echo "[INFO] Start: x=${START_X}, y=${START_Y}, yaw=${START_YAW}"

echo "[INFO] Deserializing slam_toolbox posegraph..."
timeout 15 ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{
  filename: '/src/maps/final_posegraph',
  match_type: 2,
  initial_pose: {
    x: ${START_X},
    y: ${START_Y},
    theta: ${START_YAW}
  }
}" || {
  echo "[WARN] match_type=2 failed, trying match_type=1..."
  timeout 15 ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{
    filename: '/src/maps/final_posegraph',
    match_type: 1,
    initial_pose: {
      x: 0.0,
      y: 0.0,
      theta: 0.0
    }
  }" || true
}

sleep 2

echo "[INFO] Publishing /initialpose..."
python3 - <<'PY'
import math
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class StartPosePublisher(Node):
    def __init__(self):
        super().__init__("start_pose_loader")
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        with open("/src/maps/start_pose.yaml", "r") as f:
            self.start = yaml.safe_load(f)["start"]

    def run(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = float(self.start["x"])
        msg.pose.pose.position.y = float(self.start["y"])
        msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quat(float(self.start["yaw"]))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance = [
            0.05, 0, 0, 0, 0, 0,
            0, 0.05, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.03,
        ]

        time.sleep(0.5)
        for _ in range(25):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.1)

        self.get_logger().info(
            f"Published /initialpose x={self.start['x']} y={self.start['y']} yaw={self.start['yaw']}"
        )

rclpy.init()
node = StartPosePublisher()
node.run()
node.destroy_node()
rclpy.shutdown()
PY

sleep 2

echo "[INFO] Checking /map..."
ros2 topic echo /map --once >/tmp/map_check.txt 2>/dev/null || {
  echo "[ERROR] /map is not available"
  exit 1
}

grep -E "resolution:|width:|height:|frame_id:" /tmp/map_check.txt | head -20 || true

echo "[INFO] Checking TF map -> base_link..."
timeout 5 ros2 run tf2_ros tf2_echo map base_link || true

echo "[INFO] Restarting foxglove_bridge so it republishes the latest latched /map..."
pkill -f foxglove_bridge || true
sleep 1

mkdir -p /src/log

nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0 \
  > /src/log/foxglove_bridge_manual.log 2>&1 &

sleep 3

echo "[INFO] Foxglove bridge:"
ros2 node list | grep foxglove || true
ss -lntp | grep 8765 || true

echo "[INFO] DONE"
echo "[INFO] Now refresh Foxglove with Ctrl+F5 and reconnect to ws://ROBOT_IP:8765"
