# industrial_robot_core

Industrial robot kinematics core library (C++17 + pybind11), focused on 6-DoF FK/Jacobian/IK workflows, benchmarks, and Python visualization demos.

## Features

- DH-based forward kinematics (FK)
- Geometric Jacobian computation
- DLS inverse kinematics (IK) solver
- C++ benchmark executable for IK performance
- Python bindings via pybind11
- Python demo that renders robot pose image

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run tests

```bash
ctest --test-dir build --output-on-failure
```

## Benchmark

```bash
./build/benchmark_ik
```

## Python demo

```bash
PYTHONPATH=build python3 examples/demo.py
```

Demo output image path:

- `examples/robot_pose.png`

Preview:

![Robot pose preview](examples/robot_pose.png)
