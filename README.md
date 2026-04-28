# Robomarvel 88 Mission Pack

Переносимый комплект со второго робота `rbm-83` для миссии:

1. ехать по заранее сохраненным точкам на карте;
2. на каждой точке включать подсчет детекций;
3. считать классы `apple`, `donut`, `car`, `motorcycle`;
4. выбрать самый редкий найденный класс;
5. вернуться к первой точке, где этот редкий класс был найден.

Код рассчитан на ROS2 Jazzy, Nav2 и контейнер Robomarvel, где проект смонтирован как `/src`.

## Главный алгоритм

Основной файл:

```bash
/src/scripts/drive_sticker_mission.py
```

Он:

- читает точки из `/src/maps/sticker_points.yaml`;
- отправляет цели напрямую в Nav2 action `/navigate_to_pose`;
- после приезда включает подсчет через `http://localhost:8989/counting/enable`;
- читает статистику через `http://localhost:8989/stats`;
- выключает подсчет через `http://localhost:8989/counting/disable`;
- выбирает класс с минимальным количеством;
- едет обратно к точке этого класса.

## Что лежит в репозитории

```text
scripts/drive_sticker_mission.py      основной боевой алгоритм
scripts/drive_all_points.py           просто объехать все точки без финального выбора
scripts/drive_to_start.py             вернуться в стартовую позу из maps/start_pose.yaml
scripts/mission_via_goal_proxy.py     старая версия через /goal и ros/goal_proxy.py
scripts/static_sticker_mission.py     версия через топик /stickers/detection
ros/goal_proxy.py                     мост /goal -> /navigate_to_pose
detection/low_latency_yolo_site.py    HTTP-сервис подсчета на порту 8989
detection/yolo_ros_web.py             альтернативный веб/ROS детектор
maps/sticker_points.yaml              точки объезда стикеров
maps/start_pose.yaml                  стартовая поза, теперь локальный ноль
maps/final_map.*                      сохраненная occupancy grid карта
maps/final_posegraph.*                сохраненный posegraph slam_toolbox
tools/load_map_and_start.sh           загрузка карты и стартовой позы
tools/save_sticker_point.sh           сохранение текущей точки стикера
tools/save_final_map.sh               сохранение карты
```

## Перенос на другого робота

На целевом роботе:

```bash
cd ~/robomarvel
git clone https://github.com/ObamaObama444/88.git algo88
cp -r algo88/scripts algo88/maps algo88/ros algo88/tools algo88/detection .
chmod +x scripts/*.py ros/*.py tools/*.py tools/*.sh detection/*.py
docker compose restart ros
```

Если проект на целевом роботе лежит не в `~/robomarvel`, копируй эти папки в тот каталог, который в `docker-compose.yaml` монтируется как `/src`.

## Проверка внутри контейнера

```bash
docker exec -it ros bash
cd /src
source /opt/ros/jazzy/setup.bash
source /src/install/setup.bash 2>/dev/null || true
```

Проверь Nav2:

```bash
ros2 action list | grep navigate_to_pose
ros2 topic list | grep -E '^/map$|^/tf$|^/odom$|^/scan$'
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

## Запуск карты

Если нужно загрузить сохраненную карту и стартовую позу:

```bash
bash /src/tools/load_map_and_start.sh
```

Скрипт использует:

- `/src/maps/final_posegraph.posegraph`
- `/src/maps/final_posegraph.data`
- `/src/maps/start_pose.yaml`

## Запуск сервиса подсчета

Основной алгоритм ожидает HTTP-сервис на `localhost:8989`:

```bash
python3 /src/detection/low_latency_yolo_site.py
```

Проверка:

```bash
curl http://localhost:8989/stats
curl http://localhost:8989/counting/enable
curl http://localhost:8989/counting/disable
```

## Запуск миссии

В другом терминале контейнера:

```bash
python3 /src/scripts/drive_sticker_mission.py
```

Полезные команды:

```bash
python3 /src/scripts/drive_all_points.py
python3 /src/scripts/drive_to_start.py
python3 /src/tools/stop.py
python3 /src/tools/cancel_nav.py
```

## Как менять маршрут

Редактируй:

```bash
/src/maps/sticker_points.yaml
```

Формат:

```yaml
stickers:
- id: sticker_1
  x: 4.348
  y: 0.512
  yaw: -0.456
```

`yaw` в этом файле задан в радианах.

## Локальный ноль старта

Точки в `maps/sticker_points.yaml` уже пересчитаны из старой карты в локальную систему старта робота.

Старый якорь на карте был:

```text
x = 3.605
y = 0.055
yaw = 1.505 rad
```

Теперь этот якорь считается новым нулем:

```yaml
start:
  x: 0.0
  y: 0.0
  yaw: 0.0
```

Это соответствует сценарию, где при включении робот зануляется в точке старта, а все цели задаются относительно этой якорной точки.

## Важные условия

- Nav2 должен быть активен.
- Action `/navigate_to_pose` должен существовать.
- Точки должны быть в frame `map`.
- Должен быть корректный TF `map -> base_link`.
- Детектор должен публиковать/считать классы `apple`, `donut`, `car`, `motorcycle`.
- `low_latency_yolo_site.py` зависит от окружения камеры/YOLO на роботе; если на другом роботе уже есть свой сервис детекций на `8989`, можно использовать его вместо этого файла.

## Что хорошо в алгоритме

- Он не рулит напрямую через `/cmd_vel` для движения по маршруту, а использует Nav2 `/navigate_to_pose`.
- Подсчет включается только на точке, а не во время движения.
- После каждого navigation timeout цель отменяется и публикуется stop.
- Финальная цель выбирается по фактической статистике найденных классов.

## Что стоит улучшить

- В `drive_sticker_mission.py` результат Nav2 сейчас считается успешным после завершения action без явной проверки status-кода. Для боевого варианта лучше проверять `GoalStatus.STATUS_SUCCEEDED`.
- `sticker_points.yaml` жестко привязан к конкретной карте и стартовой локализации.
- URL сервиса подсчета захардкожен на `localhost:8989`.
- Нет CLI-аргументов для выбора файла точек, таймаутов и длительности скана.
