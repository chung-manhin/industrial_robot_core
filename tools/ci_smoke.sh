#!/usr/bin/env bash
set -euo pipefail

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure

./build/benchmark_ik | tee build/benchmark.txt

PYTHONPATH=build python3 examples/demo.py

test -s examples/robot_pose.png

echo "OK"
