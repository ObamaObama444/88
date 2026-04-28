#!/usr/bin/env python3
import json
import time
import threading
from collections import Counter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


HOST = "0.0.0.0"
PORT = 8989

TARGET_CLASSES = ["apple", "donut", "car", "motorcycle"]
COUNT_COOLDOWN_SEC = 2.5


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame_jpeg = None
        self.current = Counter()
        self.total = Counter()
        self.last_count_time = {}
        self.last_msg_time = 0.0
        self.last_frame_time = 0.0
        self.error = ""
        self.running = True

    def reset(self):
        with self.lock:
            self.total = Counter()
            self.last_count_time = {}


state = SharedState()


class RosWebBridge(Node):
    def __init__(self):
        super().__init__("yolo_ros_web_bridge")

        self.create_subscription(
            CompressedImage,
            "/camera/image_annotated_compressed",
            self.image_cb,
            10,
        )

        self.create_subscription(
            String,
            "/sticker_detections",
            self.det_cb,
            10,
        )

        self.get_logger().info("Subscribed to /camera/image_annotated_compressed")
        self.get_logger().info("Subscribed to /sticker_detections")

    def image_cb(self, msg):
        with state.lock:
            state.frame_jpeg = bytes(msg.data)
            state.last_frame_time = time.time()

    def det_cb(self, msg):
        now = time.time()

        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
        except Exception as e:
            with state.lock:
                state.error = f"JSON error: {e}"
            return

        current = Counter()

        for d in detections:
            cls = d.get("class_name")
            conf = float(d.get("confidence", 0.0))

            if cls not in TARGET_CLASSES:
                continue

            current[cls] += 1

            key = cls
            last = state.last_count_time.get(key, 0.0)
            if now - last >= COUNT_COOLDOWN_SEC:
                state.total[cls] += 1
                state.last_count_time[key] = now

        with state.lock:
            state.current = current
            state.last_msg_time = now
            state.error = ""


def ros_spin_thread():
    rclpy.init()
    node = RosWebBridge()

    try:
        while state.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def html_page():
    return """<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <title>YOLO ROS Web</title>
  <style>
    body {
      margin: 0;
      background: #111;
      color: #eee;
      font-family: Arial, sans-serif;
    }
    header {
      padding: 14px 22px;
      background: #1d1d1d;
      border-bottom: 1px solid #333;
    }
    h1 {
      margin: 0;
      font-size: 22px;
    }
    .wrap {
      display: grid;
      grid-template-columns: 1fr 340px;
      gap: 14px;
      padding: 14px;
    }
    .video {
      background: #000;
      border: 1px solid #333;
      border-radius: 12px;
      overflow: hidden;
    }
    img {
      width: 100%;
      display: block;
    }
    .card {
      background: #1d1d1d;
      border: 1px solid #333;
      border-radius: 12px;
      padding: 14px;
      margin-bottom: 14px;
    }
    .row {
      display: flex;
      justify-content: space-between;
      border-bottom: 1px solid #333;
      padding: 7px 0;
      font-size: 18px;
    }
    .row:last-child {
      border-bottom: 0;
    }
    .num {
      font-size: 22px;
      font-weight: bold;
    }
    button {
      width: 100%;
      padding: 11px;
      border: 0;
      border-radius: 10px;
      background: #e33;
      color: white;
      font-size: 16px;
      cursor: pointer;
    }
    .small {
      color: #aaa;
      font-size: 14px;
      line-height: 1.5;
    }
    .err {
      color: #ff7777;
      white-space: pre-wrap;
    }
  </style>
</head>
<body>
<header>
  <h1>YOLO: apple / donut / car / motorcycle</h1>
</header>

<div class="wrap">
  <div class="video">
    <img id="video" src="/snapshot">
  </div>

  <div>
    <div class="card">
      <h2>Всего найдено</h2>
      <div class="row"><span>🍎 apple</span><span class="num" id="total_apple">0</span></div>
      <div class="row"><span>🍩 donut</span><span class="num" id="total_donut">0</span></div>
      <div class="row"><span>🚗 car</span><span class="num" id="total_car">0</span></div>
      <div class="row"><span>🏍 motorcycle</span><span class="num" id="total_motorcycle">0</span></div>
    </div>

    <div class="card">
      <h2>Сейчас в кадре</h2>
      <div class="row"><span>apple</span><span class="num" id="cur_apple">0</span></div>
      <div class="row"><span>donut</span><span class="num" id="cur_donut">0</span></div>
      <div class="row"><span>car</span><span class="num" id="cur_car">0</span></div>
      <div class="row"><span>motorcycle</span><span class="num" id="cur_motorcycle">0</span></div>
    </div>

    <div class="card">
      <button onclick="resetCounts()">Reset counts</button>
    </div>

    <div class="card small">
      <div>Последний кадр: <span id="frame_age">?</span> сек назад</div>
      <div>Последняя детекция: <span id="det_age">?</span> сек назад</div>
      <div class="err" id="error"></div>
    </div>
  </div>
</div>

<script>
function refreshImage() {
  const img = document.getElementById("video");
  img.src = "/snapshot?t=" + Date.now();
}

async function updateStats() {
  const r = await fetch("/stats?t=" + Date.now());
  const s = await r.json();

  for (const cls of ["apple", "donut", "car", "motorcycle"]) {
    document.getElementById("total_" + cls).textContent = s.total[cls] || 0;
    document.getElementById("cur_" + cls).textContent = s.current[cls] || 0;
  }

  document.getElementById("frame_age").textContent = Number(s.frame_age || 0).toFixed(2);
  document.getElementById("det_age").textContent = Number(s.det_age || 0).toFixed(2);
  document.getElementById("error").textContent = s.error || "";
}

async function resetCounts() {
  await fetch("/reset", {method: "POST"});
  await updateStats();
}

setInterval(refreshImage, 120);
setInterval(updateStats, 300);
refreshImage();
updateStats();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = html_page().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/snapshot"):
            with state.lock:
                frame = state.frame_jpeg

            if frame is None:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write("No frame yet".encode("utf-8"))
                return

            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            self.wfile.write(frame)
            return

        if self.path.startswith("/stats"):
            now = time.time()

            with state.lock:
                data = {
                    "current": {k: state.current.get(k, 0) for k in TARGET_CLASSES},
                    "total": {k: state.total.get(k, 0) for k in TARGET_CLASSES},
                    "frame_age": now - state.last_frame_time if state.last_frame_time else 999,
                    "det_age": now - state.last_msg_time if state.last_msg_time else 999,
                    "error": state.error,
                }

            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        if self.path.startswith("/reset"):
            state.reset()
            body = b'{"ok": true}'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_response(404)
        self.end_headers()


def main():
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    server = ThreadingHTTPServer((HOST, PORT), Handler)

    print(f"[INFO] Web: http://0.0.0.0:{PORT}", flush=True)
    print(f"[INFO] Open: http://ROBOT_IP:{PORT}", flush=True)
    print("[INFO] This web server DOES NOT read camera. It uses ROS topics.", flush=True)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        server.shutdown()


if __name__ == "__main__":
    main()
