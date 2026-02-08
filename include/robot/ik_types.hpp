#pragma once

#include <Eigen/Dense>

namespace robot {

using Vector6d = Eigen::Matrix<double, 6, 1>;

enum class IkStatus { kSuccess, kNoConvergence, kInvalidInput, kLimitViolation, kNumericalIssue };

struct JointLimits {
    Vector6d lower = Vector6d::Constant(-1.0e9);
    Vector6d upper = Vector6d::Constant(1.0e9);
    Vector6d max_step = Vector6d::Constant(1.0e9);

    bool clamp_to_limits = true;

    // Optional soft limit avoidance (nullspace term).
    bool enable_avoidance = false;
    double avoidance_gain = 0.0;
};

struct TaskSpaceWeights {
    // Unit conversion for position error/Jacobian top block.
    // Example: DH in mm => pos_unit = 0.001 (mm -> m).
    double pos_unit = 1.0;

    // Relative weights for position and rotation tasks.
    double w_pos = 1.0;
    double w_rot = 1.0;
};

struct IkOptions {
    int max_iters = 100;
    double tol = 1e-4;

    double step_size = 0.8;

    // Adaptive damping based on manipulability.
    double lambda0 = 1e-2;
    double lambda_min = 1e-6;
    double lambda_max = 1.0;
    double w_ref = 1.0;

    // Numerical epsilon.
    double eps = 1e-12;
};

struct IkResult {
    IkStatus status = IkStatus::kInvalidInput;
    int iterations = 0;
    double final_error = 0.0;
    Vector6d q = Vector6d::Zero();
};

struct IkWorkspace {
    // Preallocated temporaries to reduce per-iteration allocations.
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> JJt = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> e = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> dq = Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt;

    // For nullspace: pinv(J) ~= J^T (J J^T + λ^2 I)^-1
    Eigen::Matrix<double, 6, 6> J_pinv = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> N = Eigen::Matrix<double, 6, 6>::Zero();
};

} // namespace robot
