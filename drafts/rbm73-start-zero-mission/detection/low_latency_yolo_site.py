#!/usr/bin/env python3
import os
import cv2
import json
import time
import threading
import subprocess
from collections import Counter, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from ultralytics import YOLO


os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;tcp|"
    "fflags;nobuffer|"
    "flags;low_delay|"
    "max_delay;0|"
    "reorder_queue_size;0"
)

HOST = "0.0.0.0"
PORT = 8989

MODEL_PATH = "/src/yolo11n.pt"

VIDEO_SOURCES = [
    "rtsp://localhost:8554/cam",
    "http://localhost:8889/cam/",
]

DEFAULT_SELECTED_CLASSES = ["apple", "donut", "car", "motorcycle"]

CONF = 0.15
IMGSZ = 640
JPEG_QUALITY = 80
COUNT_COOLDOWN_SEC = 9.0

# Послабления для apple
APPLE_CONF_MIN = 0.08
APPLE_MIN_AREA = 120
DEFAULT_MIN_AREA = 500


class State:
    def __init__(self):
        self.lock = threading.Lock()

        self.raw_frame = None
        self.raw_time = 0.0

        self.annotated_jpeg = None
        self.annotated_time = 0.0

        self.current = Counter()
        self.total = Counter()
        self.last_count_time = {}
        self.counting_enabled = False

        self.selected_classes = list(DEFAULT_SELECTED_CLASSES)
        self.all_classes = []

        self.camera_source = None
        self.camera_ok = False
        self.error = ""

        self.read_fps = 0.0
        self.detect_fps = 0.0

        self.mission_process = None
        self.map_process = None

        self.mission_status = "нет данных"
        self.mission_log = deque(maxlen=80)

        self.map_status = "нет данных"
        self.map_log = deque(maxlen=60)

        self.running = True

    def reset_counts(self):
        with self.lock:
            self.current = Counter()
            self.total = Counter()
            self.last_count_time = {}
            self.counting_enabled = True
            self.mission_status = "Счётчик сброшен, подсчёт включён"
            self.mission_log.append(self.mission_status)


state = State()


def open_camera():
    for src in VIDEO_SOURCES:
        print(f"[INFO] Trying camera: {src}", flush=True)
        cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                with state.lock:
                    state.camera_source = src
                    state.camera_ok = True
                    state.error = ""
                print(f"[INFO] Camera opened: {src}, frame={frame.shape}", flush=True)
                return cap

        cap.release()

    with state.lock:
        state.camera_ok = False
        state.error = "Не удалось открыть камеру"
    return None


def camera_reader():
    cap = None
    frames = 0
    last_fps_time = time.time()

    while state.running:
        if cap is None:
            cap = open_camera()
            if cap is None:
                time.sleep(1.0)
                continue

        ok, frame = cap.read()

        if not ok or frame is None:
            with state.lock:
                state.camera_ok = False
                state.error = "Камера не отдаёт кадры, переподключение..."
            print("[WARN] Camera frame failed, reconnecting...", flush=True)
            cap.release()
            cap = None
            time.sleep(0.3)
            continue

        now = time.time()

        with state.lock:
            state.raw_frame = frame
            state.raw_time = now
            state.camera_ok = True
            state.error = ""

        frames += 1
        dt = now - last_fps_time
        if dt >= 1.0:
            with state.lock:
                state.read_fps = frames / dt
            frames = 0
            last_fps_time = now

    if cap is not None:
        cap.release()


def draw_label(img, text, x, y):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.65
    thickness = 2

    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    y = max(y, th + 10)

    cv2.rectangle(img, (x, y - th - 8), (x + tw + 8, y + 4), (0, 255, 0), -1)
    cv2.putText(img, text, (x + 4, y), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)


def detector_loop():
    print(f"[INFO] Loading YOLO: {MODEL_PATH}", flush=True)
    model = YOLO(MODEL_PATH)

    all_classes = [model.names[i] for i in sorted(model.names.keys())]

    with state.lock:
        state.all_classes = all_classes
        # Если вдруг дефолтного класса нет в модели, убираем его
        state.selected_classes = [c for c in state.selected_classes if c in all_classes]

    print("[INFO] YOLO loaded", flush=True)
    print("[INFO] Default selected classes:", state.selected_classes, flush=True)

    frames = 0
    last_fps_time = time.time()
    last_processed_raw_time = 0.0

    while state.running:
        with state.lock:
            if state.raw_frame is None:
                frame = None
                raw_time = 0.0
            else:
                frame = state.raw_frame.copy()
                raw_time = state.raw_time

            selected_classes = set(state.selected_classes)

        if frame is None:
            time.sleep(0.02)
            continue

        if raw_time == last_processed_raw_time:
            time.sleep(0.01)
            continue

        last_processed_raw_time = raw_time

        start = time.time()
        results = model(frame, imgsz=IMGSZ, conf=CONF, verbose=False)

        annotated = frame.copy()
        current = Counter()
        now = time.time()

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = model.names[cls_id]

                if name not in selected_classes:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = max(0, x2 - x1) * max(0, y2 - y1)

                min_area = APPLE_MIN_AREA if name == "apple" else DEFAULT_MIN_AREA
                min_conf = APPLE_CONF_MIN if name == "apple" else CONF

                if area < min_area or conf < min_conf:
                    continue

                current[name] += 1

                with state.lock:
                    if state.counting_enabled:
                        last = state.last_count_time.get(name, 0.0)

                        if now - last >= COUNT_COOLDOWN_SEC:
                            state.total[name] += 1
                            state.last_count_time[name] = now
                            print(f"[COUNT] {name} total={state.total[name]}", flush=True)

                label = f"{name} {conf:.2f}"
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                draw_label(annotated, label, x1, y1 - 5)

        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 42), (0, 0, 0), -1)

        selected_text = " ".join([f"{c}={current[c]}" for c in sorted(selected_classes)])
        if len(selected_text) > 80:
            selected_text = selected_text[:77] + "..."

        cv2.putText(
            annotated,
            selected_text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        ok, jpg = cv2.imencode(
            ".jpg",
            annotated,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )

        if ok:
            with state.lock:
                state.annotated_jpeg = jpg.tobytes()
                state.annotated_time = time.time()
                state.current = current

        frames += 1
        dt = now - last_fps_time
        if dt >= 1.0:
            with state.lock:
                state.detect_fps = frames / dt
            frames = 0
            last_fps_time = now

        elapsed = time.time() - start
        if elapsed < 0.01:
            time.sleep(0.005)


def run_shell_async(cmd, log_type="mission"):
    def worker():
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        for line in proc.stdout:
            line = line.rstrip()
            if not line:
                continue

            with state.lock:
                if log_type == "map":
                    state.map_status = line
                    state.map_log.append(line)
                else:
                    state.mission_status = line
                    state.mission_log.append(line)

        code = proc.wait()

        with state.lock:
            text = f"process finished, exit_code={code}"
            if log_type == "map":
                state.map_status = text
                state.map_log.append(text)
            else:
                state.mission_status = text
                state.mission_log.append(text)

    threading.Thread(target=worker, daemon=True).start()


def api_start_mission():
    with state.lock:
        if state.mission_process is not None and state.mission_process.poll() is None:
            return False, "mission already running"

        state.current = Counter()
        state.total = Counter()
        state.last_count_time = {}
        state.counting_enabled = False

        # Для миссии принудительно ставим нужные классы
        state.selected_classes = list(DEFAULT_SELECTED_CLASSES)

        state.mission_status = "Запуск миссии, классы сброшены на дефолтные"
        state.mission_log.append("Запуск миссии")
        state.mission_log.append("Классы: apple, donut, car, motorcycle")

    cmd = """
cd /src
source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true
python3 /src/drive_sticker_mission.py
"""

    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    with state.lock:
        state.mission_process = proc

    def reader():
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                with state.lock:
                    state.mission_status = line
                    state.mission_log.append(line)

        code = proc.wait()
        with state.lock:
            state.mission_status = f"Миссия завершилась, exit_code={code}"
            state.mission_log.append(state.mission_status)

    threading.Thread(target=reader, daemon=True).start()
    return True, "mission started"


def api_stop_mission():
    with state.lock:
        state.mission_status = "Остановка миссии"
        state.mission_log.append("Остановка миссии")

    cmd = """
source /opt/ros/jazzy/setup.bash
pkill -f drive_sticker_mission.py || true
pkill -f drive_all_points.py || true
pkill -f drive_to_start.py || true
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{goal_info: {stamp: {sec: 0, nanosec: 0}, goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}}" || true
ros2 topic pub --rate 10 --times 30 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" || true
"""

    run_shell_async(cmd, "mission")
    return True, "stop sent"


def api_update_map():
    with state.lock:
        state.map_status = "Обновление карты"
        state.map_log.append("Обновление карты")

    cmd = """
cd /src
source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true
/src/update_all_maps.sh
"""

    run_shell_async(cmd, "map")
    return True, "map update started"


def api_go_start():
    with state.lock:
        state.mission_status = "Возврат на старт"
        state.mission_log.append("Возврат на старт")

    cmd = """
cd /src
source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true
python3 /src/drive_to_start.py
"""

    run_shell_async(cmd, "mission")
    return True, "return to start started"


def html_page():
    return """<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <title>YOLO Mission Site</title>
  <style>
    body { margin: 0; background: #101010; color: #eee; font-family: Arial, sans-serif; }
    header { background: #1d1d1d; padding: 14px 22px; border-bottom: 1px solid #333; }
    h1 { margin: 0; font-size: 23px; }
    .wrap { display: grid; grid-template-columns: 1fr 400px; gap: 14px; padding: 14px; }
    .video { background: #000; border-radius: 12px; border: 1px solid #333; overflow: hidden; }
    img { width: 100%; display: block; }
    .card { background: #1c1c1c; border: 1px solid #333; border-radius: 12px; padding: 14px; margin-bottom: 14px; }
    .row { display: flex; justify-content: space-between; padding: 8px 0; border-bottom: 1px solid #333; font-size: 18px; }
    .row:last-child { border-bottom: none; }
    .num { font-size: 22px; font-weight: bold; }
    button { width: 100%; padding: 12px; border: 0; border-radius: 10px; color: white; font-size: 16px; cursor: pointer; margin-bottom: 8px; }
    .red { background: #c62828; }
    .green { background: #2e7d32; }
    .blue { background: #1565c0; }
    .orange { background: #ef6c00; }
    .gray { background: #555; }
    .small { color: #aaa; font-size: 14px; line-height: 1.6; }
    .err { color: #ff7777; white-space: pre-wrap; }
    canvas { width: 100%; height: 220px; background: #151515; border-radius: 10px; }
    .statusline { padding: 8px; border-radius: 8px; background: #151515; margin-bottom: 10px; color: #7CFC7C; white-space: pre-wrap; }
    .logbox { background: #101010; border: 1px solid #333; border-radius: 10px; padding: 10px; max-height: 220px; overflow-y: auto; white-space: pre-wrap; font-family: monospace; font-size: 12px; color: #ccc; }
    iframe { width: 100%; height: 75px; background:#101010; color:white; border:1px solid #333; border-radius:8px; margin-bottom:10px; }
    .classes-box { max-height: 260px; overflow-y: auto; background:#101010; border:1px solid #333; border-radius:10px; padding:10px; }
    .class-item { display:flex; align-items:center; gap:8px; padding:4px 0; font-size:14px; }
    .class-item input { transform: scale(1.1); }
    .hint { color:#aaa; font-size:13px; margin-top:8px; }
  </style>
</head>
<body>
<header>
  <h1>YOLO camera detection + mission control</h1>
</header>

<div class="wrap">
  <div class="video">
    <img src="/video">
  </div>

  <div>
    <div class="card">
      <h2>Классы детекции</h2>
      <div>
        <button class="green" type="button" onclick="applyClasses()">✅ Применить выбранные</button>
        <button class="gray" type="button" onclick="selectDefaultClasses()">По умолчанию</button>
        <button class="gray" type="button" onclick="selectAllClasses(false)">Снять все</button>
      </div>
      <div id="classesBox" class="classes-box">Загрузка классов...</div>
      <div class="hint">Для миссии автоматически используются: apple, donut, car, motorcycle.</div>
    </div>

    <div class="card">
      <h2>Всего найдено</h2>
      <div id="totalRows"></div>
    </div>

    <div class="card">
      <h2>График</h2>
      <canvas id="countsChart" width="360" height="220"></canvas>
    </div>

    <div class="card">
      <h2>Сейчас в кадре</h2>
      <div id="currentRows"></div>
    </div>

    <div class="card">
      <form action="/reset" method="post" target="control_result">
        <button class="red" type="submit">Reset counts</button>
      </form>
    </div>

    <div class="card">
      <h2>Управление</h2>
      <iframe name="control_result"></iframe>

      <form action="/mission/start" method="get" target="control_result">
        <button class="green" type="submit">▶ Запустить миссию</button>
      </form>

      <form action="/mission/stop" method="get" target="control_result">
        <button class="red" type="submit">⛔ Остановить миссию</button>
      </form>

      <form action="/map/update" method="get" target="control_result">
        <button class="blue" type="submit">🔄 Обновить карту</button>
      </form>

      <form action="/go/start" method="get" target="control_result">
        <button class="orange" type="submit">↩ Вернуться на старт</button>
      </form>
    </div>

    <div class="card">
      <h2>Статус миссии</h2>
      <div class="statusline" id="mission_status">нет данных</div>
      <div class="logbox" id="mission_log"></div>
    </div>

    <div class="card">
      <h2>Статус карты</h2>
      <div class="statusline" id="map_status">нет данных</div>
      <div class="logbox" id="map_log"></div>
    </div>

    <div class="card small">
      <div>Подсчёт total: <span id="counting_enabled">?</span></div>
      <div>Выбрано классов: <span id="selected_count">0</span></div>
      <div>Camera FPS: <span id="read_fps">0</span></div>
      <div>YOLO FPS: <span id="detect_fps">0</span></div>
      <div>Camera age: <span id="camera_age">0</span> s</div>
      <div>Detection age: <span id="detect_age">0</span> s</div>
      <div>Source: <span id="source">?</span></div>
      <div class="err" id="error"></div>
    </div>
  </div>
</div>

<script>
let allClasses = [];
let selectedClasses = ["apple", "donut", "car", "motorcycle"];
const defaultClasses = ["apple", "donut", "car", "motorcycle"];

function renderClassSelector() {
  const box = document.getElementById("classesBox");
  box.innerHTML = "";

  for (const cls of allClasses) {
    const id = "cls_" + cls.replaceAll(" ", "_");

    const label = document.createElement("label");
    label.className = "class-item";

    const input = document.createElement("input");
    input.type = "checkbox";
    input.value = cls;
    input.checked = selectedClasses.includes(cls);

    const span = document.createElement("span");
    span.textContent = cls;

    label.appendChild(input);
    label.appendChild(span);
    box.appendChild(label);
  }
}

async function loadClasses() {
  const r = await fetch("/classes?t=" + Date.now());
  const data = await r.json();

  allClasses = data.all_classes || [];
  selectedClasses = data.selected_classes || defaultClasses;

  renderClassSelector();
}

function getCheckedClasses() {
  const box = document.getElementById("classesBox");
  return Array.from(box.querySelectorAll("input[type=checkbox]:checked")).map(x => x.value);
}

async function applyClasses() {
  const classes = getCheckedClasses();

  const params = new URLSearchParams();
  params.set("classes", classes.join(","));

  await fetch("/classes/set?" + params.toString());
  await loadClasses();
  await updateStats();
}

function selectDefaultClasses() {
  selectedClasses = defaultClasses.slice();
  renderClassSelector();
}

function selectAllClasses(value) {
  selectedClasses = value ? allClasses.slice() : [];
  renderClassSelector();
}

function renderRows(containerId, values, labels) {
  const box = document.getElementById(containerId);
  box.innerHTML = "";

  for (const cls of labels) {
    const row = document.createElement("div");
    row.className = "row";

    const left = document.createElement("span");
    left.textContent = cls;

    const right = document.createElement("span");
    right.className = "num";
    right.textContent = values[cls] || 0;

    row.appendChild(left);
    row.appendChild(right);
    box.appendChild(row);
  }
}

function drawChart(total, labels) {
  const canvas = document.getElementById("countsChart");
  const ctx = canvas.getContext("2d");

  const values = labels.map(cls => total[cls] || 0);
  const maxVal = Math.max(1, ...values);

  const w = canvas.width;
  const h = canvas.height;

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#151515";
  ctx.fillRect(0, 0, w, h);

  if (labels.length === 0) {
    ctx.fillStyle = "#aaa";
    ctx.font = "16px Arial";
    ctx.textAlign = "center";
    ctx.fillText("Нет выбранных классов", w / 2, h / 2);
    return;
  }

  const margin = 30;
  const baseY = h - 45;
  const maxBarH = h - 80;
  const gap = 8;
  const barW = Math.max(12, Math.min(45, (w - margin * 2) / labels.length - gap));

  labels.forEach((cls, i) => {
    const val = values[i];
    const x = margin + i * (barW + gap);
    const barH = (val / maxVal) * maxBarH;
    const y = baseY - barH;

    ctx.fillStyle = "#4caf50";
    ctx.fillRect(x, y, barW, barH);

    ctx.fillStyle = "#eee";
    ctx.font = "13px Arial";
    ctx.textAlign = "center";
    ctx.fillText(String(val), x + barW / 2, y - 6);

    ctx.save();
    ctx.translate(x + barW / 2, baseY + 34);
    ctx.rotate(-Math.PI / 4);
    ctx.fillStyle = "#aaa";
    ctx.font = "11px Arial";
    ctx.fillText(cls, 0, 0);
    ctx.restore();
  });
}

async function updateStats() {
  const r = await fetch("/stats?t=" + Date.now());
  const s = await r.json();

  const labels = s.selected_classes || [];

  renderRows("totalRows", s.total || {}, labels);
  renderRows("currentRows", s.current || {}, labels);
  drawChart(s.total || {}, labels);

  document.getElementById("selected_count").textContent = labels.length;
  document.getElementById("counting_enabled").textContent = s.counting_enabled ? "ВКЛ" : "ВЫКЛ";
  document.getElementById("read_fps").textContent = Number(s.read_fps || 0).toFixed(1);
  document.getElementById("detect_fps").textContent = Number(s.detect_fps || 0).toFixed(1);
  document.getElementById("camera_age").textContent = Number(s.camera_age || 0).toFixed(2);
  document.getElementById("detect_age").textContent = Number(s.detect_age || 0).toFixed(2);
  document.getElementById("source").textContent = s.source || "?";
  document.getElementById("error").textContent = s.error || "";

  document.getElementById("mission_status").textContent = s.mission_status || "нет данных";
  document.getElementById("mission_log").textContent = (s.mission_log || []).join("\\n");
  document.getElementById("map_status").textContent = s.map_status || "нет данных";
  document.getElementById("map_log").textContent = (s.map_log || []).join("\\n");

  document.getElementById("mission_log").scrollTop = document.getElementById("mission_log").scrollHeight;
  document.getElementById("map_log").scrollTop = document.getElementById("map_log").scrollHeight;
}

loadClasses();
setInterval(updateStats, 300);
updateStats();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_json(self, data):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path == "/" or parsed.path == "/index":
            body = html_page().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/classes":
            with state.lock:
                self.send_json({
                    "all_classes": list(state.all_classes),
                    "selected_classes": list(state.selected_classes),
                    "default_classes": list(DEFAULT_SELECTED_CLASSES),
                })
            return

        if parsed.path == "/classes/set":
            qs = parse_qs(parsed.query)
            raw = qs.get("classes", [""])[0]
            requested = [x.strip() for x in raw.split(",") if x.strip()]

            with state.lock:
                allowed = set(state.all_classes)
                selected = [c for c in requested if c in allowed]

                state.selected_classes = selected
                state.current = Counter()
                state.total = Counter()
                state.last_count_time = {}
                state.mission_status = f"Выбраны классы: {', '.join(selected) if selected else 'ничего'}"
                state.mission_log.append(state.mission_status)

            self.send_json({"ok": True, "selected_classes": selected})
            return

        if parsed.path == "/stats":
            now = time.time()

            with state.lock:
                labels = list(state.selected_classes)

                data = {
                    "current": {k: int(state.current.get(k, 0)) for k in labels},
                    "total": {k: int(state.total.get(k, 0)) for k in labels},
                    "selected_classes": labels,
                    "read_fps": float(state.read_fps),
                    "detect_fps": float(state.detect_fps),
                    "camera_age": now - state.raw_time if state.raw_time else 999,
                    "detect_age": now - state.annotated_time if state.annotated_time else 999,
                    "source": state.camera_source,
                    "error": state.error,
                    "camera_ok": state.camera_ok,
                    "counting_enabled": bool(state.counting_enabled),
                    "mission_status": state.mission_status,
                    "mission_log": list(state.mission_log),
                    "map_status": state.map_status,
                    "map_log": list(state.map_log),
                }

            self.send_json(data)
            return

        if parsed.path == "/mission/start":
            ok, msg = api_start_mission()
            self.send_json({"ok": ok, "message": msg})
            return

        if parsed.path == "/mission/stop":
            ok, msg = api_stop_mission()
            self.send_json({"ok": ok, "message": msg})
            return

        if parsed.path == "/map/update":
            ok, msg = api_update_map()
            self.send_json({"ok": ok, "message": msg})
            return

        if parsed.path == "/go/start":
            ok, msg = api_go_start()
            self.send_json({"ok": ok, "message": msg})
            return

        if parsed.path == "/counting/enable":
            with state.lock:
                state.counting_enabled = True
                state.last_count_time = {}
                state.mission_status = "Подсчёт включён: робот стоит и сканирует"
                state.mission_log.append(state.mission_status)
            self.send_json({"ok": True, "counting_enabled": True})
            return

        if parsed.path == "/counting/disable":
            with state.lock:
                state.counting_enabled = False
                state.mission_status = "Подсчёт выключен: робот движется"
                state.mission_log.append(state.mission_status)
            self.send_json({"ok": True, "counting_enabled": False})
            return

        if parsed.path == "/video":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            last_sent = 0.0

            while True:
                with state.lock:
                    frame = state.annotated_jpeg
                    frame_time = state.annotated_time

                if frame is None:
                    time.sleep(0.03)
                    continue

                if frame_time == last_sent:
                    time.sleep(0.01)
                    continue

                last_sent = frame_time

                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)

        if parsed.path == "/reset":
            state.reset_counts()
            self.send_json({"ok": True, "message": "counts reset"})
            return

        self.send_response(404)
        self.end_headers()


def main():
    cam_thread = threading.Thread(target=camera_reader, daemon=True)
    det_thread = threading.Thread(target=detector_loop, daemon=True)

    cam_thread.start()
    det_thread.start()

    server = ThreadingHTTPServer((HOST, PORT), Handler)

    print(f"[INFO] Site started: http://0.0.0.0:{PORT}", flush=True)
    print(f"[INFO] Open from laptop: http://ROBOT_IP:{PORT}", flush=True)
    print("[INFO] Default classes:", ", ".join(DEFAULT_SELECTED_CLASSES), flush=True)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        server.shutdown()


if __name__ == "__main__":
    main()
