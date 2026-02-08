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

static bool withinLimits(const Vector6d& q, const JointLimits& lim, double eps) {
    for (int i = 0; i < 6; ++i) {
        if (q(i) < lim.lower(i) - eps) return false;
        if (q(i) > lim.upper(i) + eps) return false;
    }
    return true;
}

int main() {
    // Same IRB120-style DH (mm) as other regression tests.
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

    // Create a reachable target pose from a known joint configuration.
    Vector6d q_target;
    q_target << 0.35, -0.25, 0.20, 0.50, -0.40, 0.30;

    const auto T_target = arm.computeFK(q_target);

    TaskSpaceWeights weights;
    weights.pos_unit = 0.001; // mm -> m
    weights.w_pos = 1.0;
    weights.w_rot = 1.0;

    // Strict joint bounds that still contain q_target.
    JointLimits lim;
    lim.lower << q_target(0) - deg2rad(5.0), q_target(1) - deg2rad(5.0), q_target(2) - deg2rad(5.0),
        q_target(3) - deg2rad(5.0), q_target(4) - deg2rad(5.0), q_target(5) - deg2rad(5.0);
    lim.upper << q_target(0) + deg2rad(5.0), q_target(1) + deg2rad(5.0), q_target(2) + deg2rad(5.0),
        q_target(3) + deg2rad(5.0), q_target(4) + deg2rad(5.0), q_target(5) + deg2rad(5.0);
    lim.max_step = Vector6d::Constant(deg2rad(10.0));
    lim.clamp_to_limits = true;

    IkOptions opt;
    opt.max_iters = 200;
    opt.tol = 1e-3;
    opt.step_size = 0.8;

    IKSolverDls solver(arm);

    // Start from a seed inside the strict limits window.
    Vector6d seed;
    seed << q_target(0) + deg2rad(2.0), q_target(1) - deg2rad(2.0), q_target(2) + deg2rad(1.0),
        q_target(3) - deg2rad(1.0), q_target(4) + deg2rad(2.0), q_target(5) - deg2rad(2.0);

    robot::IkWorkspace ws;
    const IkResult r = solver.solve(T_target, seed, opt, &lim, weights, &ws);

    if (r.status != IkStatus::kSuccess) {
        std::cerr << "IK did not succeed. status=" << static_cast<int>(r.status)
                  << " final_error=" << r.final_error << "\n";
        return 1;
    }

    constexpr double kEps = 1e-9;
    if (!withinLimits(r.q, lim, kEps)) {
        std::cerr << "Solution violates joint limits.\n";
        std::cerr << "q=" << r.q.transpose() << "\n";
        std::cerr << "lower=" << lim.lower.transpose() << "\n";
        std::cerr << "upper=" << lim.upper.transpose() << "\n";
        return 1;
    }

    if (!(r.final_error < opt.tol)) {
        std::cerr << "final_error not below tol. final_error=" << r.final_error << " tol=" << opt.tol << "\n";
        return 1;
    }

    std::cout << "test_ik_joint_limits PASS\n";
    return 0;
}
