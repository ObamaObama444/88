#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true

ID="${1:-}"

if [[ -z "$ID" ]]; then
  echo "Usage: /src/save_sticker_point.sh sticker_1"
  exit 1
fi

OUT="/src/maps/sticker_points.yaml"
TMP="/tmp/tf_${ID}.txt"

echo "[INFO] Reading TF map -> base_link for $ID..."
timeout 3 ros2 run tf2_ros tf2_echo map base_link > "$TMP" || true

if ! grep -q "Translation:" "$TMP"; then
  echo "[ERROR] Could not read TF map -> base_link"
  cat "$TMP"
  exit 1
fi

python3 - "$ID" "$OUT" "$TMP" <<'PY'
import sys
import re
import yaml
from pathlib import Path

point_id = sys.argv[1]
out_path = Path(sys.argv[2])
tf_path = Path(sys.argv[3])

text = tf_path.read_text()

translations = re.findall(r"Translation:\s*\[([^\]]+)\]", text)
rpys = re.findall(r"Rotation: in RPY \(radian\)\s*\[([^\]]+)\]", text)

if not translations or not rpys:
    print("[ERROR] Could not parse TF output")
    print(text)
    sys.exit(1)

# Берём последнее значение, оно самое свежее
tr = [float(v.strip()) for v in translations[-1].split(",")]
rpy = [float(v.strip()) for v in rpys[-1].split(",")]

x = round(tr[0], 3)
y = round(tr[1], 3)
yaw = round(rpy[2], 3)

if out_path.exists():
    data = yaml.safe_load(out_path.read_text()) or {}
else:
    data = {}

stickers = data.get("stickers", [])

found = False
for p in stickers:
    if p.get("id") == point_id:
        p["x"] = x
        p["y"] = y
        p["yaw"] = yaw
        found = True
        break

if not found:
    stickers.append({
        "id": point_id,
        "x": x,
        "y": y,
        "yaw": yaw,
    })

data["stickers"] = stickers

out_path.write_text(
    yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
    encoding="utf-8"
)

print(f"[OK] Saved {point_id}: x={x}, y={y}, yaw={yaw}")
print(f"[INFO] File: {out_path}")
PY
