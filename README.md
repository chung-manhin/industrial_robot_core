# industrial_robot_core

Industrial robot kinematics core library (C++17 + pybind11) for 6-DoF manipulators: forward kinematics (FK), Jacobian, and damped least-squares inverse kinematics (DLS IK). Includes C++ tests/benchmark and a Python demo that renders a pose image.

## Requirements

- C++17 compiler (GCC/Clang)
- CMake >= 3.16
- Eigen3
- Python 3 (optional; only required for Python bindings/demo)
- matplotlib (optional; only required to run `examples/demo.py`)

Ubuntu example:

```bash
sudo apt-get update
sudo apt-get install -y cmake g++ libeigen3-dev python3 python3-pip
# Optional for demo:
python3 -m pip install --user matplotlib
```

## Quick Start

From clone to generating `examples/robot_pose.png`:

```bash
git clone <your-repo-url>
cd industrial_robot_core_clean

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

ctest --test-dir build --output-on-failure
./build/benchmark_ik

# (Optional) Python demo: requires matplotlib
PYTHONPATH=build python3 examples/demo.py

# Verify the output image is present and non-empty
test -s examples/robot_pose.png
```

Preview:

![Robot pose preview](examples/robot_pose.png)

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

Output:

- `examples/robot_pose.png`

## Units / Conventions

- The demo DH parameters in `examples/demo.py` are in **millimeters (mm)**.
- `TaskSpaceWeights.pos_unit` is used to scale position units (e.g., `0.001` for mm -> m) so that position and rotation errors are comparable.

## FAQ

**Q: `examples/demo.py` fails with `ImportError: matplotlib`**

A: matplotlib is optional and not needed for the C++ library/tests. Install it to run the demo:

```bash
python3 -m pip install --user matplotlib
```

**Q: Where does the Python module come from?**

A: The build produces a pybind11 module named `industrial_robot_core` into the build directory; run the demo with `PYTHONPATH=build`.
