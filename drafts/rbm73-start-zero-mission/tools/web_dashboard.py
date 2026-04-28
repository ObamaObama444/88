#!/usr/bin/env python3
import json
import threading
import time
from collections import Counter, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


TARGET_CLASSES = ["apple", "car", "donut", "motorcycle"]
HOST = "0.0.0.0"
PORT = 8989

latest_jpeg = None
latest_seen_text = "Нет данных"
latest_detections = []
counts = Counter()
events = deque(maxlen=30)
lock = threading.Lock()


class DashboardRosNode(Node):
    def __init__(self):
        super().__init__("web_detection_dashboard")

        self.create_subscription(
            CompressedImage,
            "/camera/image_annotated_compressed",
            self.on_image,
            10
        )

        self.create_subscription(
            String,
            "/sticker_detections",
            self.on_detections,
            10
        )

        self.last_count_time = {}
        self.count_cooldown_sec = 3.0

    def on_image(self, msg):
        global latest_jpeg
        with lock:
            latest_jpeg = bytes(msg.data)

    def on_detections(self, msg):
        global latest_seen_text, latest_detections

        try:
            data = json.loads(msg.data)
        except Exception:
            return

        detections = []

        for d in data.get("detections", []):
            name = d.get("class_name")
            conf = float(d.get("confidence", 0.0))
            area = float(d.get("area", 0.0))

            if name not in TARGET_CLASSES:
                continue
            if conf < 0.35:
                continue
            if area < 800:
                continue

            detections.append({
                "class_name": name,
                "confidence": round(conf, 2),
                "area": round(area, 1),
            })

        now = time.time()

        with lock:
            latest_detections = detections

            if detections:
                names = [d["class_name"] for d in detections]
                latest_seen_text = ", ".join(names)

                for name in set(names):
                    last = self.last_count_time.get(name, 0.0)

                    if now - last >= self.count_cooldown_sec:
                        counts[name] += 1
                        self.last_count_time[name] = now
                        events.appendleft({
                            "time": time.strftime("%H:%M:%S"),
                            "class": name,
                            "count": counts[name],
                        })
            else:
                latest_seen_text = "Ничего не найдено"


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_GET(self):
        if self.path == "/":
            self.send_html()
        elif self.path == "/stream":
            self.send_stream()
        elif self.path == "/api":
            self.send_api()
        elif self.path == "/reset":
            self.reset_counts()
        else:
            self.send_response(404)
            self.end_headers()

    def send_html(self):
        html = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<title>Rover Detection Dashboard</title>
<style>
body { margin:0; background:#111; color:#eee; font-family:Arial,sans-serif; }
.wrap { display:grid; grid-template-columns:2fr 1fr; gap:16px; padding:16px; }
.card { background:#1d1d1d; border-radius:12px; padding:16px; }
img { width:100%; max-height:75vh; object-fit:contain; background:#000; border-radius:8px; }
table { width:100%; border-collapse:collapse; font-size:20px; }
td, th { border-bottom:1px solid #444; padding:10px; text-align:left; }
.big { font-size:22px; margin:14px 0; }
button { background:#333; color:white; border:1px solid #777; border-radius:8px; padding:10px 14px; cursor:pointer; }
.small { color:#aaa; font-size:14px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>Камера с детекцией</h1>
    <img src="/stream">
  </div>
  <div class="card">
    <h2>Найдено объектов</h2>
    <table>
      <thead><tr><th>Класс</th><th>Кол-во</th></tr></thead>
      <tbody id="counts"></tbody>
    </table>
    <div class="big">Сейчас в кадре: <b id="latest">...</b></div>
    <button onclick="resetCounts()">Сбросить счётчик</button>
    <h2>Последние события</h2>
    <div id="events"></div>
    <p class="small">Один класс засчитывается не чаще одного раза в 3 секунды.</p>
  </div>
</div>

<script>
async function update() {
  const res = await fetch('/api');
  const data = await res.json();

  const tbody = document.getElementById('counts');
  tbody.innerHTML = '';

  for (const cls of data.target_classes) {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${cls}</td><td>${data.counts[cls] || 0}</td>`;
    tbody.appendChild(tr);
  }

  document.getElementById('latest').innerText = data.latest_seen_text;

  const events = document.getElementById('events');
  events.innerHTML = '';

  for (const ev of data.events) {
    const div = document.createElement('div');
    div.innerText = `${ev.time} - ${ev.class}: ${ev.count}`;
    events.appendChild(div);
  }
}

async function resetCounts() {
  await fetch('/reset');
  await update();
}

setInterval(update, 500);
update();
</script>
</body>
</html>"""

        data = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_api(self):
        with lock:
            payload = {
                "target_classes": TARGET_CLASSES,
                "counts": {cls: counts.get(cls, 0) for cls in TARGET_CLASSES},
                "latest_seen_text": latest_seen_text,
                "latest_detections": latest_detections,
                "events": list(events),
            }

        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def reset_counts(self):
        global counts, events
        with lock:
            counts = Counter()
            events.clear()

        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"ok")

    def send_stream(self):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        while True:
            with lock:
                frame = latest_jpeg

            if frame is not None:
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                except BrokenPipeError:
                    break

            time.sleep(0.1)


def ros_thread():
    rclpy.init()
    node = DashboardRosNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    threading.Thread(target=ros_thread, daemon=True).start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Dashboard started: http://0.0.0.0:{PORT}")
    server.serve_forever()


if __name__ == "__main__":
    main()
