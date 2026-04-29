#!/usr/bin/env python3
import base64
import json
import math
import os
import shutil
import subprocess
import time
import urllib.error
import urllib.request
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


TARGET_CLASSES = {"apple", "car", "donut", "motorcycle"}

DEFAULT_POINTS_FILE = "/src/maps/sticker_points.yaml"
DEFAULT_OUTPUT_ROOT = "/tmp/rover-mission"
DEFAULT_BASE_URL = "https://www.adolanna.ru"
DEFAULT_RTSP_URL = "rtsp://172.18.0.2:8554/cam"
DEFAULT_FRAME_COUNT = 8
DEFAULT_MIN_FRAME_COUNT = 4
DEFAULT_DWELL_SEC = 3.0
DEFAULT_NAV_TIMEOUT_SEC = 80.0
DEFAULT_API_TIMEOUT_SEC = 120.0
DEFAULT_FFMPEG_TIMEOUT_SEC = 30.0
DEFAULT_MAX_FRAME_BYTES = 3_500_000
DEFAULT_FINAL_ALREADY_THERE_DIST = 0.05


def env_int(name, default):
    value = os.getenv(name)
    if not value:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def env_float(name, default):
    value = os.getenv(name)
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def mission_id_now():
    return "mission_" + datetime.now().strftime("%Y%m%d_%H%M%S")


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def post_json(url, token, payload, timeout_sec):
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        headers={
            "Content-Type": "application/json",
            "X-Robot-Token": token,
        },
        method="POST",
    )

    with urllib.request.urlopen(request, timeout=timeout_sec) as response:
        parsed = json.loads(response.read().decode("utf-8"))
    return parsed if isinstance(parsed, dict) else {}


class DriveScanClassifyMission(Node):
    def __init__(self):
        super().__init__("drive_scan_classify_mission")

        self.points_file = os.getenv("MISSION_POINTS_FILE", DEFAULT_POINTS_FILE)
        self.output_root = Path(os.getenv("MISSION_OUTPUT_ROOT", DEFAULT_OUTPUT_ROOT))
        self.mission_id = os.getenv("MISSION_ID", mission_id_now())
        self.rtsp_url = os.getenv("ROBOT_RTSP_URL", DEFAULT_RTSP_URL)
        self.frame_count = max(1, env_int("MISSION_FRAME_COUNT", DEFAULT_FRAME_COUNT))
        self.min_frame_count = max(1, env_int("MISSION_MIN_FRAME_COUNT", DEFAULT_MIN_FRAME_COUNT))
        self.dwell_sec = max(0.1, env_float("MISSION_DWELL_SEC", DEFAULT_DWELL_SEC))
        self.nav_timeout_sec = max(1.0, env_float("MISSION_NAV_TIMEOUT_SEC", DEFAULT_NAV_TIMEOUT_SEC))
        self.api_timeout_sec = max(5.0, env_float("MISSION_API_TIMEOUT_SEC", DEFAULT_API_TIMEOUT_SEC))
        self.ffmpeg_timeout_sec = max(
            self.dwell_sec + 5.0,
            env_float("MISSION_FFMPEG_TIMEOUT_SEC", DEFAULT_FFMPEG_TIMEOUT_SEC),
        )
        self.max_frame_bytes = max(100_000, env_int("ROBOT_MAX_FRAME_BYTES", DEFAULT_MAX_FRAME_BYTES))
        self.final_already_there_dist = max(
            0.0,
            env_float("MISSION_FINAL_ALREADY_THERE_DIST", DEFAULT_FINAL_ALREADY_THERE_DIST),
        )

        base_url = os.getenv("MINIAPP_BASE_URL", DEFAULT_BASE_URL).rstrip("/")
        self.classify_url = os.getenv(
            "MINIAPP_MISSION_CLASSIFY_URL",
            f"{base_url}/api/robot/mission/classify-point",
        )
        self.complete_url = os.getenv(
            "MINIAPP_MISSION_COMPLETE_URL",
            f"{base_url}/api/robot/mission/complete",
        )
        self.robot_token = os.getenv("ROBOT_PUSH_TOKEN")
        if not self.robot_token:
            raise RuntimeError("ROBOT_PUSH_TOKEN is required")

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        with open(self.points_file, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)

        self.points = data["stickers"]
        self.route_index = {p["id"]: idx for idx, p in enumerate(self.points)}
        self.records = []
        self.counts = Counter()
        self.points_by_class = defaultdict(list)
        self.last_successful_point = None
        self.mission_dir = self.output_root / self.mission_id
        self.mission_dir.mkdir(parents=True, exist_ok=True)

        self.status(f"MISSION_LOADED id={self.mission_id} points={len(self.points)}")

    def status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(20):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def make_pose(self, p):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(p["x"])
        pose.pose.position.y = float(p["y"])
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat(float(p["yaw"]))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def go_to(self, p, timeout=None):
        timeout = self.nav_timeout_sec if timeout is None else timeout
        self.status(f"GO_TO {p['id']} x={p['x']} y={p['y']} yaw={p['yaw']}")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.status("ERROR_NAVIGATE_TO_POSE_NOT_AVAILABLE")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(p)

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.status(f"NAV_REJECTED {p['id']}")
            return False

        result_future = goal_handle.get_result_async()
        start_time = time.time()

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.status(f"NAV_TIMEOUT {p['id']}")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                self.stop_robot()
                return False

        self.stop_robot()
        self.last_successful_point = p
        self.status(f"ARRIVED {p['id']}")
        return True

    def go_via_points(self, p):
        via_points = p.get("via") or []
        if not isinstance(via_points, list):
            self.status(f"INVALID_VIA_POINTS {p['id']}")
            return False

        for via in via_points:
            if not isinstance(via, dict):
                self.status(f"INVALID_VIA_POINT {p['id']}")
                return False
            via_id = via.get("id") or f"via_before_{p['id']}"
            via_point = {
                "id": str(via_id),
                "x": float(via["x"]),
                "y": float(via["y"]),
                "yaw": float(via["yaw"]),
            }
            self.status(f"GO_VIA target={p['id']} via={via_point['id']}")
            if not self.go_to(via_point):
                return False
        return True

    def point_dir(self, p):
        return self.mission_dir / str(p["id"])

    def capture_frames(self, p):
        point_dir = self.point_dir(p)
        if point_dir.exists():
            shutil.rmtree(point_dir)
        point_dir.mkdir(parents=True, exist_ok=True)

        output_pattern = str(point_dir / "frame_%03d.jpg")
        fps = self.frame_count / self.dwell_sec
        command = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-rtsp_transport",
            "tcp",
            "-i",
            self.rtsp_url,
            "-t",
            f"{self.dwell_sec:.3f}",
            "-vf",
            f"fps={fps:.6f}",
            "-frames:v",
            str(self.frame_count),
            "-q:v",
            "2",
            "-start_number",
            "0",
            output_pattern,
        ]

        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=self.ffmpeg_timeout_sec,
            check=False,
        )
        if completed.returncode != 0:
            message = completed.stderr.decode("utf-8", errors="ignore").strip()
            raise RuntimeError(f"ffmpeg failed: {message[-500:]}")

        frames = sorted(point_dir.glob("frame_*.jpg"))
        if len(frames) < self.min_frame_count:
            raise RuntimeError(f"Expected at least {self.min_frame_count} frames, got {len(frames)}")
        if len(frames) < self.frame_count:
            self.status(f"CAPTURE_PARTIAL {p['id']} expected={self.frame_count} got={len(frames)}")

        for path in frames:
            if path.stat().st_size <= 0 or path.stat().st_size > self.max_frame_bytes:
                raise RuntimeError(f"Invalid frame size: {path.name}")

        return frames

    def encode_frames(self, frame_paths):
        encoded = []
        for path in frame_paths:
            encoded.append(base64.b64encode(path.read_bytes()).decode("ascii"))
        return encoded

    def classify_point(self, p, frame_paths):
        payload = {
            "mission_id": self.mission_id,
            "point_id": p["id"],
            "point": {
                "x": float(p["x"]),
                "y": float(p["y"]),
                "yaw": float(p["yaw"]),
            },
            "frames": self.encode_frames(frame_paths),
        }
        return post_json(self.classify_url, self.robot_token, payload, self.api_timeout_sec)

    def record_result(self, p, result):
        class_name = result.get("final_class")
        status = result.get("status")
        confidence = result.get("confidence")
        counted = status == "ok" and class_name in TARGET_CLASSES

        record = {
            "point_id": p["id"],
            "x": float(p["x"]),
            "y": float(p["y"]),
            "yaw": float(p["yaw"]),
            "class_name": class_name,
            "confidence": confidence,
            "status": status,
            "gate_status": result.get("gate_status"),
            "gate_frames_passed": result.get("gate_frames_passed"),
            "frames_total": result.get("frames_total"),
            "frames_valid": result.get("frames_valid"),
            "capture_dir": result.get("capture_dir"),
            "report_path": result.get("report_path"),
            "gate_report_path": result.get("gate_report_path"),
            "counted": counted,
        }
        self.records.append(record)

        if counted:
            self.counts[class_name] += 1
            self.points_by_class[class_name].append({"point": p, "record": record})

        write_json(self.point_dir(p) / "result.json", record)
        self.write_summary()
        self.status(
            f"POINT_RESULT {p['id']} class={class_name} status={status} "
            f"confidence={confidence} counted={counted} counts={dict(self.counts)}"
        )

    def record_failure(self, p, status, error):
        record = {
            "point_id": p["id"],
            "x": float(p["x"]),
            "y": float(p["y"]),
            "yaw": float(p["yaw"]),
            "class_name": None,
            "confidence": None,
            "status": status,
            "error": str(error),
            "counted": False,
        }
        self.records.append(record)
        write_json(self.point_dir(p) / "result.json", record)
        self.write_summary()
        self.status(f"POINT_FAILED {p['id']} status={status} error={error}")

    def choose_final_target(self):
        if not self.counts:
            return None, None

        min_count = min(self.counts.values())
        rare_classes = {cls for cls, count in self.counts.items() if count == min_count}
        current = self.last_successful_point
        if current is None:
            return None, None

        candidates = []
        for cls in sorted(rare_classes):
            for item in self.points_by_class[cls]:
                point = item["point"]
                distance = math.hypot(float(point["x"]) - float(current["x"]), float(point["y"]) - float(current["y"]))
                candidates.append((distance, self.route_index.get(point["id"], 999), cls, point))

        if not candidates:
            return None, None

        distance, _, cls, point = min(candidates, key=lambda item: (item[0], item[1], item[2]))
        return cls, {"point": point, "distance": distance, "count": min_count}

    def write_summary(self, target=None):
        payload = {
            "mission_id": self.mission_id,
            "points": self.records,
            "counts": dict(self.counts),
            "target": target,
        }
        write_json(self.mission_dir / "mission_summary.json", payload)

    def notify_mission_complete(self, status, target=None, error=None):
        payload = {
            "mission_id": self.mission_id,
            "status": status,
            "points": self.records,
            "counts": dict(self.counts),
            "target": target,
            "error": str(error) if error else None,
        }
        write_json(self.mission_dir / "mission_summary.json", payload)
        try:
            response = post_json(self.complete_url, self.robot_token, payload, self.api_timeout_sec)
            if not response.get("ok"):
                self.status(f"MISSION_COMPLETE_NOTIFY_FAILED response={response}")
            else:
                self.status(f"MISSION_COMPLETE_NOTIFIED status={status}")
        except Exception as notify_error:
            self.status(f"MISSION_COMPLETE_NOTIFY_ERROR {notify_error}")

    def run(self):
        self.status("MISSION_START_SCAN_CLASSIFY")

        for p in self.points:
            if not self.go_via_points(p):
                self.record_failure(p, "nav_failed", "via navigation failed")
                continue

            if not self.go_to(p):
                self.record_failure(p, "nav_failed", "navigation failed")
                continue

            self.status(f"CAPTURE_START {p['id']} dwell={self.dwell_sec} frames={self.frame_count}")
            try:
                frame_paths = self.capture_frames(p)
            except Exception as error:
                self.record_failure(p, "capture_failed", error)
                continue

            self.status(f"CLASSIFY_START {p['id']} frames={len(frame_paths)}")
            try:
                result = self.classify_point(p, frame_paths)
            except (urllib.error.URLError, TimeoutError, OSError) as error:
                self.record_failure(p, "api_failed", error)
                continue
            except Exception as error:
                self.record_failure(p, "api_failed", error)
                continue

            if not result.get("ok"):
                self.record_failure(p, "api_failed", result.get("error") or result)
                continue

            self.record_result(p, result)

        self.status(f"MISSION_SCAN_FINISHED counts={dict(self.counts)}")
        target_class, target = self.choose_final_target()
        target_summary = {"class_name": target_class, **target} if target else None
        self.write_summary(target=target_summary)

        if not target:
            self.status("MISSION_NO_COUNTED_CLASSES_STOP")
            self.stop_robot()
            self.notify_mission_complete(
                "failed",
                target=None,
                error="no counted classes were found during mission",
            )
            return

        target_point = target["point"]
        self.status(
            f"MISSION_TARGET class={target_class} count={target['count']} "
            f"point={target_point['id']} distance={target['distance']:.3f}"
        )

        if target["distance"] <= self.final_already_there_dist:
            self.status(f"MISSION_TARGET_ALREADY_HERE point={target_point['id']}")
            self.stop_robot()
            self.notify_mission_complete("completed", target=target_summary)
            return

        if self.go_to(target_point):
            self.status("MISSION_FINISHED_AT_RAREST_CLASS_POINT")
            self.notify_mission_complete("completed", target=target_summary)
        else:
            self.status("MISSION_FINAL_NAV_FAILED")
            self.notify_mission_complete(
                "failed",
                target=target_summary,
                error="final navigation to rarest class point failed",
            )

        self.stop_robot()


def main():
    rclpy.init()
    node = None
    try:
        node = DriveScanClassifyMission()
        node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.status("MISSION_INTERRUPTED")
            node.stop_robot()
            node.notify_mission_complete("failed", error="mission interrupted")
    except Exception as error:
        if node is not None:
            node.status(f"MISSION_FATAL {error}")
            node.stop_robot()
            node.notify_mission_complete("failed", error=error)
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
