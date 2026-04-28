# Проезд по точкам

Эта папка содержит минимальный комплект, который использовался для последнего успешного проезда робота `rbm-73` по 8 точкам без подсчета классов.

## Файлы

- `drive_all_points.py` — ROS2/Nav2 скрипт, который последовательно отправляет точки в `/navigate_to_pose`.
- `sticker_points.yaml` — нормализованные координаты точек относительно нулевого старта робота.

## Куда положить на роботе

В контейнере файлы должны оказаться так:

```bash
/src/scripts/drive_all_points.py
/src/maps/sticker_points.yaml
```

## Команда запуска

С хоста робота:

```bash
docker exec ros bash -ic "python3 /src/scripts/drive_all_points.py"
```

Именно этой командой робот успешно проехал:

```text
sticker_1 -> sticker_2 -> sticker_3 -> sticker_4 -> sticker_5 -> sticker_6 -> sticker_7 -> sticker_8
```
