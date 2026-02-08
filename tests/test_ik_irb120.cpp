#include "robot_arm.hpp"

#include "robot/ik_solver_dls.hpp"
#include "robot/ik_types.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using robot::DHParam;
using robot::IKSolverDls;
using robot::IkOptions;
using robot::IkResult;
using robot::IkStatus;
using robot::JointLimits;
using robot::RobotArm;
using robot::TaskSpaceWeights;
using robot::Vector6d;

static double deg2rad(double deg) { return deg * M_PI / 180.0; }

int main() {
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

    Vector6d q_target;
    q_target << 0.1, 0.2, -0.3, 0.4, -0.5, 0.6;

    const auto T_target = arm.computeFK(q_target);

    TaskSpaceWeights weights;
    weights.pos_unit = 0.001; // mm -> m
    weights.w_pos = 1.0;
    weights.w_rot = 1.0;

    JointLimits limits;
    limits.lower << deg2rad(-170), deg2rad(-110), deg2rad(-170), deg2rad(-190), deg2rad(-120), deg2rad(-400);
    limits.upper << deg2rad(170), deg2rad(110), deg2rad(170), deg2rad(190), deg2rad(120), deg2rad(400);
    limits.max_step = Vector6d::Constant(deg2rad(10.0));
    limits.clamp_to_limits = true;

    IKSolverDls solver(arm);

    IkOptions opt;
    opt.max_iters = 120;
    opt.tol = 1e-6;
    opt.step_size = 0.8;
    opt.lambda0 = 1e-2;
    opt.lambda_min = 1e-6;
    opt.lambda_max = 1e-1;
    opt.w_ref = 1.0;

    std::vector<Vector6d> seeds = {Vector6d::Zero(), q_target};

    robot::IkWorkspace ws;
    const IkResult r = solver.solveBestOf(T_target, seeds, opt, &limits, weights, &ws, &q_target);

    if (r.status != IkStatus::kSuccess) {
        std::cerr << "IK failed with status=" << static_cast<int>(r.status) << "\n";
        return 1;
    }

    const double joint_err = (r.q - q_target).norm();
    if (!(joint_err < 1e-3)) {
        std::cerr << "Joint error too large: " << joint_err << "\n";
        return 1;
    }

    // Limit behavior: seed out of bounds should clamp (since clamp_to_limits=true).
    Vector6d q_bad = Vector6d::Constant(1000.0);
    const IkResult r2 = solver.solve(T_target, q_bad, opt, &limits, weights, &ws);
    if (r2.status != IkStatus::kSuccess && r2.status != IkStatus::kNoConvergence) {
        // At least shouldn't be limit violation in clamp mode.
        std::cerr << "Unexpected status in clamp mode: " << static_cast<int>(r2.status) << "\n";
        return 1;
    }
    if (r2.q.minCoeff() < limits.lower.minCoeff() - 1e-9 || r2.q.maxCoeff() > limits.upper.maxCoeff() + 1e-9) {
        std::cerr << "Clamping did not respect limits\n";
        return 1;
    }

    std::cout << "test_ik_irb120 PASS\n";
    return 0;
}
