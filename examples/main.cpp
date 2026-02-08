#include "robot_arm.hpp"

#include "robot/ik_solver_dls.hpp"
#include "robot/ik_types.hpp"

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using robot::DHParam;
using robot::IkOptions;
using robot::IkResult;
using robot::IKSolverDls;
using robot::IkStatus;
using robot::JointLimits;
using robot::RobotArm;
using robot::TaskSpaceWeights;
using robot::Vector6d;

static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

static const char* statusToStr(IkStatus s) {
    switch (s) {
    case IkStatus::kSuccess:
        return "Success";
    case IkStatus::kNoConvergence:
        return "NoConvergence";
    case IkStatus::kInvalidInput:
        return "InvalidInput";
    case IkStatus::kLimitViolation:
        return "LimitViolation";
    case IkStatus::kNumericalIssue:
        return "NumericalIssue";
    }
    return "Unknown";
}

int main() {
    // ABB IRB 120 standard DH. Units: mm.
    // Note: Link 2 has joint offset -90 deg: theta = q2 - 90deg => dh.theta = -90deg.
    const std::vector<DHParam> dh = {
        {0.0, deg2rad(-90.0), 290.0, 0.0},          // 1
        {270.0, deg2rad(0.0), 0.0, deg2rad(-90.0)}, // 2 (offset)
        {70.0, deg2rad(-90.0), 0.0, 0.0},           // 3
        {0.0, deg2rad(90.0), 302.0, 0.0},           // 4
        {0.0, deg2rad(-90.0), 0.0, 0.0},            // 5
        {0.0, deg2rad(0.0), 72.0, 0.0}              // 6
    };

    RobotArm arm(dh);
    if (!arm.isValid()) {
        std::cerr << "RobotArm invalid (need 6 DH params)\n";
        return 2;
    }

    Vector6d q_target;
    q_target << 0.1, 0.2, -0.3, 0.4, -0.5, 0.6;

    const Eigen::Matrix<double, 4, 4> T_target = arm.computeFK(q_target);

    TaskSpaceWeights weights;
    weights.pos_unit = 0.001; // mm -> m
    weights.w_pos = 1.0;
    weights.w_rot = 1.0;

    JointLimits limits;
    // Reasonable generic bounds (rad). Can be replaced with exact ABB spec later.
    limits.lower << deg2rad(-170), deg2rad(-110), deg2rad(-170), deg2rad(-190), deg2rad(-120),
        deg2rad(-400);
    limits.upper << deg2rad(170), deg2rad(110), deg2rad(170), deg2rad(190), deg2rad(120),
        deg2rad(400);
    limits.max_step = Vector6d::Constant(deg2rad(10.0));
    limits.clamp_to_limits = true;

    IKSolverDls solver(arm);

    std::mt19937 rng(0);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);

    std::vector<Vector6d> seeds;
    seeds.push_back(Vector6d::Zero());
    seeds.push_back(q_target);
    for (int k = 0; k < 2; ++k) {
        Vector6d q;
        for (int i = 0; i < 6; ++i)
            q(i) = uni(rng);
        seeds.push_back(q);
    }

    IkOptions opt;
    opt.max_iters = 120;
    opt.tol = 1e-6;
    opt.step_size = 0.8;
    opt.lambda0 = 1e-2;
    opt.lambda_min = 1e-6;
    opt.lambda_max = 1e-1;
    opt.w_ref = 1.0;

    robot::IkWorkspace ws;
    const IkResult r = solver.solveBestOf(T_target, seeds, opt, &limits, weights, &ws, &q_target);

    std::cout.setf(std::ios::fixed);
    std::cout.precision(6);

    std::cout << "IK status    : " << statusToStr(r.status) << "\n";
    std::cout << "iters        : " << r.iterations << "\n";
    std::cout << "final_error  : " << r.final_error << "\n";
    std::cout << "q_target     : " << q_target.transpose() << "\n";
    std::cout << "q_solution   : " << r.q.transpose() << "\n";

    const double joint_err = (r.q - q_target).norm();
    std::cout << "||q_sol - q_target|| = " << joint_err << "\n";

    if (r.status == IkStatus::kSuccess && joint_err < 1e-3) {
        std::cout << "PASS: joint error < 1e-3\n";
        return 0;
    }

    std::cout << "FAIL\n";
    return 1;
}
