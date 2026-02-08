#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "robot/ik_solver_dls.hpp"
#include "robot/ik_types.hpp"
#include "robot_arm.hpp"

#include <array>
#include <optional>
#include <vector>

namespace py = pybind11;

namespace {

using PyVector6 = std::array<double, 6>;
using PyVec3 = std::array<double, 3>;
using PyChain = std::array<PyVec3, 7>;
using PyMatrix4 = std::array<std::array<double, 4>, 4>;
using PyMatrix6 = std::array<std::array<double, 6>, 6>;

static robot::Vector6d toEigen(const PyVector6& v) {
    robot::Vector6d out;
    for (int i = 0; i < 6; ++i) out(i) = v[static_cast<size_t>(i)];
    return out;
}

static PyVector6 fromEigen(const robot::Vector6d& v) {
    PyVector6 out{};
    for (int i = 0; i < 6; ++i) out[static_cast<size_t>(i)] = v(i);
    return out;
}

static robot::Matrix4d toEigen(const PyMatrix4& m) {
    robot::Matrix4d out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out(r, c) = m[static_cast<size_t>(r)][static_cast<size_t>(c)];
        }
    }
    return out;
}

static PyMatrix4 fromEigen(const robot::Matrix4d& m) {
    PyMatrix4 out{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out[static_cast<size_t>(r)][static_cast<size_t>(c)] = m(r, c);
        }
    }
    return out;
}

static PyMatrix6 fromEigen(const robot::Matrix6d& m) {
    PyMatrix6 out{};
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            out[static_cast<size_t>(r)][static_cast<size_t>(c)] = m(r, c);
        }
    }
    return out;
}

static PyChain fromChain(const std::array<Eigen::Vector3d, 7>& chain) {
    PyChain out{};
    for (int i = 0; i < 7; ++i) {
        out[static_cast<size_t>(i)] = {chain[static_cast<size_t>(i)].x(),
                                       chain[static_cast<size_t>(i)].y(),
                                       chain[static_cast<size_t>(i)].z()};
    }
    return out;
}

struct PyJointLimits {
    PyVector6 lower{ -1e9, -1e9, -1e9, -1e9, -1e9, -1e9 };
    PyVector6 upper{ 1e9, 1e9, 1e9, 1e9, 1e9, 1e9 };
    PyVector6 max_step{ 1e9, 1e9, 1e9, 1e9, 1e9, 1e9 };

    bool clamp_to_limits = true;
    bool enable_avoidance = false;
    double avoidance_gain = 0.0;
};

static robot::JointLimits toCore(const PyJointLimits& p) {
    robot::JointLimits l;
    l.lower = toEigen(p.lower);
    l.upper = toEigen(p.upper);
    l.max_step = toEigen(p.max_step);
    l.clamp_to_limits = p.clamp_to_limits;
    l.enable_avoidance = p.enable_avoidance;
    l.avoidance_gain = p.avoidance_gain;
    return l;
}

struct PyIkResult {
    robot::IkStatus status = robot::IkStatus::kInvalidInput;
    int iterations = 0;
    double final_error = 0.0;
    PyVector6 q{0, 0, 0, 0, 0, 0};
};

static PyIkResult fromCore(const robot::IkResult& r) {
    PyIkResult out;
    out.status = r.status;
    out.iterations = r.iterations;
    out.final_error = r.final_error;
    out.q = fromEigen(r.q);
    return out;
}

} // namespace

PYBIND11_MODULE(industrial_robot_core, m) {
    m.doc() = "industrial_robot_core (RobotMotionCore) Python bindings (no numpy required)";

    py::enum_<robot::IkStatus>(m, "IkStatus")
        .value("kSuccess", robot::IkStatus::kSuccess)
        .value("kNoConvergence", robot::IkStatus::kNoConvergence)
        .value("kInvalidInput", robot::IkStatus::kInvalidInput)
        .value("kLimitViolation", robot::IkStatus::kLimitViolation)
        .value("kNumericalIssue", robot::IkStatus::kNumericalIssue);

    py::class_<robot::DHParam>(m, "DHParam")
        .def(py::init<>())
        .def_readwrite("a", &robot::DHParam::a)
        .def_readwrite("alpha", &robot::DHParam::alpha)
        .def_readwrite("d", &robot::DHParam::d)
        .def_readwrite("theta", &robot::DHParam::theta);

    py::class_<robot::TaskSpaceWeights>(m, "TaskSpaceWeights")
        .def(py::init<>())
        .def_readwrite("pos_unit", &robot::TaskSpaceWeights::pos_unit)
        .def_readwrite("w_pos", &robot::TaskSpaceWeights::w_pos)
        .def_readwrite("w_rot", &robot::TaskSpaceWeights::w_rot);

    py::class_<PyJointLimits>(m, "JointLimits")
        .def(py::init<>())
        .def_readwrite("lower", &PyJointLimits::lower)
        .def_readwrite("upper", &PyJointLimits::upper)
        .def_readwrite("max_step", &PyJointLimits::max_step)
        .def_readwrite("clamp_to_limits", &PyJointLimits::clamp_to_limits)
        .def_readwrite("enable_avoidance", &PyJointLimits::enable_avoidance)
        .def_readwrite("avoidance_gain", &PyJointLimits::avoidance_gain);

    py::class_<robot::IkOptions>(m, "IkOptions")
        .def(py::init<>())
        .def_readwrite("max_iters", &robot::IkOptions::max_iters)
        .def_readwrite("tol", &robot::IkOptions::tol)
        .def_readwrite("step_size", &robot::IkOptions::step_size)
        .def_readwrite("lambda0", &robot::IkOptions::lambda0)
        .def_readwrite("lambda_min", &robot::IkOptions::lambda_min)
        .def_readwrite("lambda_max", &robot::IkOptions::lambda_max)
        .def_readwrite("w_ref", &robot::IkOptions::w_ref)
        .def_readwrite("eps", &robot::IkOptions::eps);

    py::class_<PyIkResult>(m, "IkResult")
        .def(py::init<>())
        .def_readwrite("status", &PyIkResult::status)
        .def_readwrite("iterations", &PyIkResult::iterations)
        .def_readwrite("final_error", &PyIkResult::final_error)
        .def_readwrite("q", &PyIkResult::q);

    py::class_<robot::RobotArm>(m, "RobotArm")
        .def(py::init<const std::vector<robot::DHParam>&>())
        .def("isValid", &robot::RobotArm::isValid)
        .def("computeFK",
             [](const robot::RobotArm& self, const PyVector6& joints) { return fromEigen(self.computeFK(toEigen(joints))); },
             py::arg("joints"))
        .def("computeJacobian",
             [](const robot::RobotArm& self, const PyVector6& joints) { return fromEigen(self.computeJacobian(toEigen(joints))); },
             py::arg("joints"))
        .def("forwardKinematicsChain",
             [](const robot::RobotArm& self, const PyVector6& joints) {
                 return fromChain(self.forwardKinematicsChain(toEigen(joints)));
             },
             py::arg("joints"));

    py::class_<robot::IKSolverDls>(m, "IKSolverDls")
        .def(py::init<const robot::RobotArm&>(), py::keep_alive<1, 2>())
        .def("solve",
             [](const robot::IKSolverDls& self,
                const PyMatrix4& target_pose,
                const PyVector6& seed,
                const robot::IkOptions& options,
                const std::optional<PyJointLimits>& limits,
                const robot::TaskSpaceWeights& weights) {
                 const robot::Matrix4d T = toEigen(target_pose);
                 const robot::Vector6d q0 = toEigen(seed);

                 if (limits) {
                     const robot::JointLimits l = toCore(*limits);
                     return fromCore(self.solve(T, q0, options, &l, weights, nullptr));
                 }
                 return fromCore(self.solve(T, q0, options, nullptr, weights, nullptr));
             },
             py::arg("target_pose"),
             py::arg("seed"),
             py::arg("options") = robot::IkOptions{},
             py::arg("limits") = py::none(),
             py::arg("weights") = robot::TaskSpaceWeights{})
        .def("solveBestOf",
             [](const robot::IKSolverDls& self,
                const PyMatrix4& target_pose,
                const std::vector<PyVector6>& seeds,
                const robot::IkOptions& options,
                const std::optional<PyJointLimits>& limits,
                const robot::TaskSpaceWeights& weights,
                const std::optional<PyVector6>& preferred) {
                 const robot::Matrix4d T = toEigen(target_pose);

                 std::vector<robot::Vector6d> seeds_e;
                 seeds_e.reserve(seeds.size());
                 for (const auto& s : seeds) seeds_e.push_back(toEigen(s));

                 const robot::Vector6d* pref_ptr = nullptr;
                 robot::Vector6d pref_e;
                 if (preferred) {
                     pref_e = toEigen(*preferred);
                     pref_ptr = &pref_e;
                 }

                 if (limits) {
                     const robot::JointLimits l = toCore(*limits);
                     return fromCore(self.solveBestOf(T, seeds_e, options, &l, weights, nullptr, pref_ptr));
                 }
                 return fromCore(self.solveBestOf(T, seeds_e, options, nullptr, weights, nullptr, pref_ptr));
             },
             py::arg("target_pose"),
             py::arg("seeds"),
             py::arg("options") = robot::IkOptions{},
             py::arg("limits") = py::none(),
             py::arg("weights") = robot::TaskSpaceWeights{},
             py::arg("preferred") = py::none());
}
