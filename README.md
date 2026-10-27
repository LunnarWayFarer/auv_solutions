# Обработка телеметрии АНПА в ROS2

Программная часть проекта 2600 «LLM-решения для диагностики аномалий и рекомендаций в динамических подводных сетях АНПА на основе анализа логов».

В проекте есть получение телеметрии Stonefish, первый агент мониторинга с файлами JSON и программное воспроизведение нарушений доставки. Команды проверки и поля сообщений описаны в [docs/stonefish-integration.md](docs/stonefish-integration.md).

Сохранена начальная демонстрация на синтетических данных. Она содержит четыре узла:

```text
synthetic_source -> telemetry_adapter -> diagnostic_stub -> report_recorder
```

Источник публикует тестовые показания. Адаптер проверяет поля и преобразует единицы измерения. Узел `diagnostic_stub` возвращает явно помеченный тестовый ответ. Последний узел записывает отчёты в JSONL.

Stonefish и ArduSub SITL соединены двусторонним мостом: автопилот получает состояние симуляции и возвращает PWM движителей. Сводки первого агента публикуются в ROS2 раз в пять секунд. Сборка и совместная проверка описаны в [docs/ardusub-integration.md](docs/ardusub-integration.md).

Диагностическая LLM и RAG пока не подключены. Первый агент уже формирует сводки по показаниям Stonefish, но класс неисправности ещё не определяет. Значения `status=stub`, `anomaly_type=unknown` и `diagnostic_accuracy=null` обозначают тестовый режим. Успешная передача сообщений не является оценкой точности диагностики.

## Локальный запуск

Все команды выполняются из корня репозитория. Для локального запуска достаточно Python 3.10+; ROS2 не требуется:

```sh
PYTHONPATH=ros2_ws/src/auv_diagnostics python3 -m auv_diagnostics.demo --count 20
```

Команда печатает сводку и путь к новой папке в `artifacts/`. В ней находятся исходные сообщения `raw.jsonl`, преобразованные сообщения `normalized.jsonl`, отчёты `reports.jsonl` и сводка `summary.json`. Файлы старых запусков не перезаписываются.

Локальный запуск вызывает функции обработки напрямую. Обмен через ROS2 проверяется отдельно в контейнере.

## ROS2 Humble в Docker

Потребуется запущенный Docker. На Mac с Apple Silicon используется Linux ARM64 без эмуляции AMD64. Все четыре узла работают внутри одного контейнера.

```sh
docker build -f docker/Dockerfile -t auv-week1 .
mkdir -p artifacts
docker run --rm --mount type=bind,source="$PWD/artifacts",target=/artifacts auv-week1 python3 /workspace/scripts/check_ros_demo.py --output-root /artifacts --count 20
```

При успешной проверке появляется `ROS2_SMOKE_PASS`. Скрипт сверяет число отчётов, идентификаторы, последовательность сообщений и преобразованные значения. Результаты сохраняются в `artifacts/ros2-*/`. Допустимо от 1 до 100 сообщений.

После изменения Python-кода образ нужно пересобрать. Для ручного запуска:

```sh
docker run --rm -it auv-week1 bash
ros2 pkg executables auv_diagnostics
ros2 launch auv_diagnostics week1.launch.py --show-args
```

## Проверки

```sh
PYTHONPATH=ros2_ws/src/auv_diagnostics python3 -m unittest discover -s ros2_ws/src/auv_diagnostics/test -v
```

Ожидается `Ran 26 tests` и `OK`. Тесты проверяют преобразование единиц, состав и значения полей, некорректный JSON и сохранение сообщений.

Проверка Python-кода:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install ruff==0.11.13
.venv/bin/ruff check ros2_ws/src scripts
```

Ожидается `All checks passed!`. Проверка пакета в контейнере:

```sh
docker run --rm auv-week1 bash -c 'cd /workspace/ros2_ws && colcon test --return-code-on-test-failure && colcon test-result --all --verbose'
```

После пересборки образа ожидается 26 тестов без ошибок, падений и пропусков.

## Stonefish

Сцена `scenarios/integration.scn` содержит движущийся аппарат, четыре полноценных движителя, давление, IMU, DVL, навигационную оценку с шумом и отдельную истинную одометрию. Проверочный скрипт записывает окна телеметрии и сводки первого агента, проверяет прямое вращение и реверс, а затем воспроизводит сетевые нарушения на записанных данных.

Запуск: [docs/stonefish-integration.md](docs/stonefish-integration.md). Проверка одного неподвижного датчика давления сохранена как отдельная начальная проверка.

Команды сборки и запуска: [docs/stonefish.md](docs/stonefish.md).

## Файлы

- `ros2_ws/src/auv_diagnostics/`: пакет ROS2, обработка данных, узлы, общий launch-файл и тесты.
- `scripts/check_ros_demo.py`: проверка передачи сообщений через четыре ROS2-узла.
- `scripts/check_stonefish.py`: отдельная проверка датчика давления.
- `scripts/check_ardusub_integration.py`: совместная проверка Stonefish, ArduSub, MAVLink и первого агента.
- `scripts/ardusub_bridge.py`: обмен состоянием и PWM с автопилотом.
- `scripts/ardusub_mavlink.py`: телеметрия автопилота в ROS2.
- `scripts/check_stonefish_integration.py`: получение датчиков, RPM и траектории, запись окон JSON и сетевые проверки.
- `auv_diagnostics/monitoring.py`: обработка JSON первым агентом.
- `auv_diagnostics/network.py`: воспроизведение доставки и контроль доступности узлов.
- `reports/integration-stage/`: результаты проверки датчиков, JSON и нарушений доставки.
- `reports/ardusub-stage/`: результаты совместного запуска Stonefish и ArduSub.
- `docker/`: Dockerfile и запуск контейнеров.
- `scenarios/pressure_probe.scn`: сцена для датчика давления.
- `.github/workflows/ci.yml`: проверки Python и ROS2 для GitHub Actions.
- `docs/interfaces.md`: поля сообщений, единицы и ограничения.
- `docs/repository-upload.md`: перенос в общий репозиторий.
- `reports/software-review-2026-10-19/`: сохранённые результаты повторной проверки.

GitHub Actions настроен на Ruff, unittest, сборку контейнера, colcon test и передачу сообщений через ROS2. Результат конкретного запуска проверяется на вкладке Actions.
