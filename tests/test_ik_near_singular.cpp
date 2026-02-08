#include "robot_arm.hpp"

#include "robot/ik_solver_dls.hpp"
#include "robot/ik_types.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using robot::DHParam;
using robot::IKSolverDls;
using robot::IkOptions;
using robot::IkResult;
using robot::IkStatus;
using robot::RobotArm;
using robot::TaskSpaceWeights;
using robot::Vector6d;

static double deg2rad(double deg) { return deg * M_PI / 180.0; }

static bool allFinite(const Vector6d& q) {
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(q(i))) return false;
    }
    return true;
}

int main() {
    // IRB120-style DH (mm).
    const std::vector<DHParam> dh = {
        {0.0, deg2rad(-90.0), 290.0, 0.0},
        {270.0, deg2rad(0.0), 0.0, deg2rad(-90.0)},
        {70.0, deg2rad(-90.0), 0.0, 0.0},
        {0.0, deg2rad(90.0), 302.0, 0.0},
        {0.0, deg2rad(-90.0), 0.0, 0.0},
        {0.0, deg2rad(0.0), 72.0, 0.0}
    };

    RobotArm arm(dh);
    assert(arm.isValid());

    // Construct a reachable target, then perturb slightly to get a "hard" case
    // (often increases sensitivity when close to a kinematic singularity).
    Vector6d q_near;
    q_near << 0.0, deg2rad(-90.0), 0.0, 0.0, 0.0, 0.0;

    auto T_target = arm.computeFK(q_near);

    // Small translation perturbation (in mm) to avoid exact symmetry.
    T_target(0, 3) += 0.5;
    T_target(1, 3) -= 0.5;

    TaskSpaceWeights weights;
    weights.pos_unit = 0.001; // mm -> m
    weights.w_pos = 1.0;
    weights.w_rot = 1.0;

    IkOptions opt;
    opt.max_iters = 200;
    opt.tol = 1e-4;
    opt.step_size = 0.8;
    opt.lambda0 = 1e-2;
    opt.lambda_min = 1e-6;
    opt.lambda_max = 1e-1;

    IKSolverDls solver(arm);

    // A seed that is intentionally not too close.
    Vector6d seed;
    seed << 0.3, -0.2, 0.1, 0.4, -0.1, 0.2;

    robot::IkWorkspace ws;
    const IkResult r = solver.solve(T_target, seed, opt, nullptr, weights, &ws);

    // Stability requirements: never return NaN/Inf.
    if (!allFinite(r.q) || !std::isfinite(r.final_error)) {
        std::cerr << "Non-finite output detected. final_error=" << r.final_error << "\n";
        return 1;
    }

    // Status should be one of the defined values.
    switch (r.status) {
    case IkStatus::kSuccess:
    case IkStatus::kNoConvergence:
    case IkStatus::kInvalidInput:
    case IkStatus::kLimitViolation:
    case IkStatus::kNumericalIssue:
        break;
    default:
        std::cerr << "Unknown IkStatus value\n";
        return 1;
    }

    // If solver claims success, it must meet tolerance.
    if (r.status == IkStatus::kSuccess) {
        if (!(r.final_error < opt.tol)) {
            std::cerr << "Success but error above tol. final_error=" << r.final_error << " tol=" << opt.tol << "\n";
            return 1;
        }
    } else {
        // If it didn't succeed, it should still behave "stably" (bounded error).
        if (!(r.final_error >= 0.0) || r.final_error > 1e6) {
            std::cerr << "Unstable error magnitude. final_error=" << r.final_error << "\n";
            return 1;
        }
    }

    std::cout << "test_ik_near_singular PASS\n";
    return 0;
}
