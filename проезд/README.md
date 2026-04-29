# Проезд по точкам

Эта папка содержит минимальный комплект, который использовался для последнего успешного проезда робота `rbm-73` по 8 точкам без подсчета классов.

## Файлы

- `drive_all_points.py` — ROS2/Nav2 скрипт, который последовательно отправляет точки в `/navigate_to_pose`.
- `drive_scan_classify_mission.py` — полный сценарий: проезд по 8 точкам, 8 кадров на точку, отправка на ВМ, подсчёт классов и финальный приезд к ближайшей точке самого редкого класса.
- `sticker_points.yaml` — нормализованные координаты точек относительно нулевого старта робота.

## Куда положить на роботе

В контейнере файлы должны оказаться так:

```bash
/src/scripts/drive_all_points.py
/src/scripts/drive_scan_classify_mission.py
/src/maps/sticker_points.yaml
```

## Команда запуска

С хоста робота:

```bash
docker exec ros bash -ic "python3 /src/scripts/drive_all_points.py"
```

Именно этой командой робот успешно проехал:

```text
sticker_3 -> sticker_2 -> sticker_6 -> sticker_7 -> sticker_5 -> sticker_4 -> sticker_1 -> sticker_8
```

## Команда запуска миссии с классификацией

С хоста робота:

```bash
docker exec \
  -e ROBOT_PUSH_TOKEN="$ROBOT_PUSH_TOKEN" \
  -e MINIAPP_BASE_URL=https://www.adolanna.ru \
  -e ROBOT_RTSP_URL=rtsp://172.18.0.2:8554/cam \
  ros bash -ic "python3 /src/scripts/drive_scan_classify_mission.py"
```

По умолчанию миссия:

- сканирует все 8 точек из `/src/maps/sticker_points.yaml`;
- едет в порядке `sticker_3 -> sticker_2 -> sticker_6 -> sticker_7 -> sticker_5 -> sticker_4 -> sticker_1 -> sticker_8`;
- стоит на каждой точке 3 секунды;
- сохраняет 8 кадров в `/tmp/rover-mission/<mission_id>/<point_id>`;
- отправляет кадры на `/api/robot/mission/classify-point`;
- считает только ответы `status=ok` по классам `apple`, `car`, `donut`, `motorcycle`;
- после обхода едет к ближайшей точке класса, который встретился меньше всего.

Основные переменные:

```text
MISSION_FRAME_COUNT=8
MISSION_DWELL_SEC=3.0
MISSION_OUTPUT_ROOT=/tmp/rover-mission
MISSION_NAV_TIMEOUT_SEC=80
MISSION_API_TIMEOUT_SEC=120
MINIAPP_MISSION_CLASSIFY_URL=https://www.adolanna.ru/api/robot/mission/classify-point
```
