# Обработка телеметрии АНПА в ROS2

Начальная программная часть проекта 2600 «LLM-решения для диагностики аномалий и рекомендаций в динамических подводных сетях АНПА на основе анализа логов».

Код проверяет получение, преобразование и сохранение синтетической телеметрии. Пакет ROS2 содержит четыре узла:

```text
synthetic_source -> telemetry_adapter -> diagnostic_stub -> report_recorder
```

Источник публикует тестовые показания. Адаптер проверяет поля и преобразует единицы измерения. Узел `diagnostic_stub` возвращает явно помеченный тестовый ответ. Последний узел записывает отчёты в JSONL.

LLM, RAG и распознавание неисправностей пока не подключены. Значения `status=stub`, `anomaly_type=unknown` и `diagnostic_accuracy=null` обозначают тестовый режим. Успешная передача сообщений не является оценкой точности диагностики.

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

Ожидается `Ran 8 tests` и `OK`. Тесты проверяют преобразование единиц, состав и значения полей, некорректный JSON и сохранение сообщений.

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

Ожидается восемь тестов без ошибок, падений и пропусков.

## Stonefish

Отдельный сценарий получает показания давления в ROS2 от неподвижного объекта на глубине 5 м. Это проверка датчика и передачи данных. Модель движущегося АНПА и подключение показаний к диагностическому пакету пока отсутствуют.

Команды сборки и запуска: [docs/stonefish.md](docs/stonefish.md).

## Файлы

- `ros2_ws/src/auv_diagnostics/`: пакет ROS2, обработка данных, узлы, общий launch-файл и тесты.
- `scripts/check_ros_demo.py`: проверка передачи сообщений через четыре ROS2-узла.
- `scripts/check_stonefish.py`: отдельная проверка датчика Stonefish.
- `docker/`: Dockerfile и запуск контейнеров.
- `scenarios/pressure_probe.scn`: сцена для датчика давления.
- `.github/workflows/ci.yml`: проверки Python и ROS2 для GitHub Actions.
- `docs/interfaces.md`: поля сообщений, единицы и ограничения.
- `docs/repository-upload.md`: перенос в общий репозиторий.
- `reports/software-review-2026-10-19/`: сохранённые результаты повторной проверки.

GitHub Actions настроен на Ruff, unittest, сборку контейнера, colcon test и передачу сообщений через ROS2. Результат конкретного запуска проверяется на вкладке Actions.
