#!/usr/bin/env bash
set -euo pipefail

# Ensure we use the same Python interpreter for building and running the pybind11 module.
# In GitHub Actions, `actions/setup-python` provides the `python` executable.
: "${PYTHON:=python}"
: "${BUILD_DIR:=build}"

PYTHON_BIN="$(command -v "${PYTHON}")"

cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release \
  -DPYBIND11_FINDPYTHON=ON \
  -DPython_EXECUTABLE="${PYTHON_BIN}" \
  -DPython3_EXECUTABLE="${PYTHON_BIN}"
cmake --build "${BUILD_DIR}" -j
ctest --test-dir "${BUILD_DIR}" --output-on-failure

./"${BUILD_DIR}"/benchmark_ik | tee "${BUILD_DIR}"/benchmark.txt

PYTHONPATH="${BUILD_DIR}" "${PYTHON_BIN}" examples/demo.py

test -s examples/robot_pose.png

echo "OK"
