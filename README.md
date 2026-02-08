# industrial_robot_core (RobotMotionCore)

![build](https://img.shields.io/badge/build-passing-brightgreen)
![license](https://img.shields.io/badge/license-MIT-blue)
![cpp](https://img.shields.io/badge/C%2B%2B-17-blue)
![python](https://img.shields.io/badge/python-pybind11-informational)

Industrial-grade, soft real-time friendly 6-DoF robot motion core library:
- DH-based **FK / geometric Jacobian**
- Industrialized **DLS-IK** (joint limits, adaptive damping, task weights + unit scaling, multi-seed selection)
- Modern CMake: supports both **`add_subdirectory()`** and **`find_package(CONFIG)`**
- Minimal but effective **CTest regression tests**
- **pybind11** Python bindings

---

## Architecture

```
industrial_robot_core/
├─ include/
│  ├─ robot_arm.hpp                  # DH model + FK/Jacobian
│  └─ robot/
│     ├─ ik_types.hpp                # Options/Result/Workspace + status codes
│     └─ ik_solver_dls.hpp           # IKSolverDls API
├─ src/
│  ├─ robot_arm.cpp
│  ├─ ik_solver_dls.cpp
│  └─ python_bindings.cpp            # pybind11 module: industrial_robot_core
├─ examples/
│  ├─ main.cpp                       # C++ demo
│  └─ demo.py                        # Python demo
├─ tests/
│  ├─ test_fk.cpp
│  ├─ test_jacobian_fd.cpp
│  ├─ test_ik_irb120.cpp
│  └─ benchmark_ik.cpp               # 10k IK benchmarking
├─ cmake/
│  └─ robot_motion_coreConfig.cmake.in
├─ CMakeLists.txt
└─ Doxyfile
```

---

## Performance data

> Machine-dependent. Typical output on the developer machine (Release build, 10k trials):

- Avg: **~53.5 us**
- P99: **~104.5 us**
- Max: **~214.7 us**
- Success rate: **~70.7%** (random reachable targets + random seeds; multi-seed can improve)

---

## Build (C++)

```bash
cmake -S industrial_robot_core -B industrial_robot_core/build -DCMAKE_BUILD_TYPE=Release
cmake --build industrial_robot_core/build --parallel

# Run demo
./industrial_robot_core/build/robot_demo

# Run tests
ctest --test-dir industrial_robot_core/build --output-on-failure

# Run benchmark
./industrial_robot_core/build/benchmark_ik
```

---

## Quick start (C++)

```cpp
#include "robot_arm.hpp"
#include "robot/ik_solver_dls.hpp"
#include "robot/ik_types.hpp"

using robot::RobotArm;
using robot::IKSolverDls;
using robot::Vector6d;

int main() {
    std::vector<robot::DHParam> dh = /* 6-DoF DH params */;
    RobotArm arm(dh);

    IKSolverDls solver(arm);
    robot::IkWorkspace ws;  // reuse in your control loop

    robot::TaskSpaceWeights w;
    w.pos_unit = 0.001; // mm->m if DH is mm

    robot::IkOptions opt;
    opt.max_iters = 120;
    opt.tol = 1e-6;

    robot::Matrix4d T_target = arm.computeFK(Vector6d::Zero());
    Vector6d seed = Vector6d::Zero();

    robot::IkResult r = solver.solve(T_target, seed, opt, nullptr, w, &ws);
    return (r.status == robot::IkStatus::kSuccess) ? 0 : 1;
}
```

---

## Quick start (Python)

```bash
cmake -S industrial_robot_core -B industrial_robot_core/build -DCMAKE_BUILD_TYPE=Release
cmake --build industrial_robot_core/build --parallel
PYTHONPATH=industrial_robot_core/build python3 industrial_robot_core/examples/demo.py
```

---

## Install / find_package

```bash
cmake --install industrial_robot_core/build --prefix /tmp/robot_motion_core_install

# In your consumer project:
# find_package(robot_motion_core CONFIG REQUIRED)
# target_link_libraries(app PRIVATE robot_motion_core::robot_motion_core)
```

---

## License

MIT
