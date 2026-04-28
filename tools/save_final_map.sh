#!/usr/bin/env bash
set -eo pipefail

# ВАЖНО: не включаем set -u до source ROS, потому что setup.bash может использовать незаданные переменные
source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true

set -u

MAP_DIR="/src/maps"
MAP_NAME="final_map"
MAP_PATH="${MAP_DIR}/${MAP_NAME}"
BACKUP_DIR="${MAP_DIR}/backups"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"

mkdir -p "$MAP_DIR"
mkdir -p "$BACKUP_DIR"

echo "[INFO] Map directory: $MAP_DIR"
echo "[INFO] Target map: ${MAP_PATH}.yaml / ${MAP_PATH}.pgm"

if [[ -f "${MAP_PATH}.yaml" || -f "${MAP_PATH}.pgm" ]]; then
  echo "[INFO] Existing map found. Creating backup: ${BACKUP_DIR}/${MAP_NAME}_${STAMP}"

  if [[ -f "${MAP_PATH}.yaml" ]]; then
    cp "${MAP_PATH}.yaml" "${BACKUP_DIR}/${MAP_NAME}_${STAMP}.yaml"
  fi

  if [[ -f "${MAP_PATH}.pgm" ]]; then
    cp "${MAP_PATH}.pgm" "${BACKUP_DIR}/${MAP_NAME}_${STAMP}.pgm"
  fi

  echo "[INFO] Backup created."
else
  echo "[INFO] No existing final_map found. Backup skipped."
fi

TMP_NAME="${MAP_NAME}_tmp_${STAMP}"
TMP_PATH="${MAP_DIR}/${TMP_NAME}"

echo "[INFO] Saving current SLAM map to temporary files..."
ros2 run nav2_map_server map_saver_cli -f "$TMP_PATH"

if [[ ! -f "${TMP_PATH}.yaml" || ! -f "${TMP_PATH}.pgm" ]]; then
  echo "[ERROR] Temporary map was not created correctly."
  exit 1
fi

mv "${TMP_PATH}.yaml" "${MAP_PATH}.yaml"
mv "${TMP_PATH}.pgm" "${MAP_PATH}.pgm"

python3 - <<PY2
from pathlib import Path
yaml_path = Path("${MAP_PATH}.yaml")
text = yaml_path.read_text()
lines = []
for line in text.splitlines():
    if line.strip().startswith("image:"):
        lines.append("image: ${MAP_NAME}.pgm")
    else:
        lines.append(line)
yaml_path.write_text("\\n".join(lines) + "\\n")
PY2

echo "[INFO] New map saved successfully:"
ls -lh "${MAP_PATH}.yaml" "${MAP_PATH}.pgm"

echo "[INFO] Backups:"
ls -lh "$BACKUP_DIR" | tail -10
