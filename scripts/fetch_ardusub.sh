#!/usr/bin/env bash
set -euo pipefail
revision=fb773567896014da007bca58358704d4a0d8516f
destination=${1:-artifacts/ardusub-source}
if [ ! -d "$destination/.git" ]; then
    git clone --depth 1 --branch Sub-4.7.2 --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git "$destination"
fi
if [ "$(git -C "$destination" rev-parse HEAD)" != "$revision" ]; then
    echo 'Версия исходников не совпадает с закреплённой версией ArduSub.' >&2
    exit 1
fi
# Upstream excludes SITL source from its own Docker context.
printf 'build\n' > "$destination/.dockerignore"
echo "Исходники ArduSub готовы: $destination"
