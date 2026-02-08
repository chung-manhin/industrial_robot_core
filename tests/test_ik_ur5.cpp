#include "robot_arm.hpp"

#include "robot/ik_solver_dls.hpp"
#include "robot/ik_types.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
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

static std::vector<DHParam> makeUr5DhM() {
    // UR5 classic DH (meters).
    // (a, alpha, d, theta_offset)
    const double pi = M_PI;
    return {
        {0.0, pi / 2.0, 0.089159, 0.0},
        {-0.425, 0.0, 0.0, 0.0},
        {-0.39225, 0.0, 0.0, 0.0},
        {0.0, pi / 2.0, 0.10915, 0.0},
        {0.0, -pi / 2.0, 0.09465, 0.0},
        {0.0, 0.0, 0.0823, 0.0},
    };
}

static bool isPoseClose(const robot::Matrix4d& A, const robot::Matrix4d& B, double pos_tol) {
    const auto pA = A.block<3, 1>(0, 3);
    const auto pB = B.block<3, 1>(0, 3);
    const double dp = (pA - pB).norm();
    return dp < pos_tol;
}

int main() {
    RobotArm arm(makeUr5DhM());
    assert(arm.isValid());

    IKSolverDls solver(arm);
    robot::IkWorkspace ws;

    TaskSpaceWeights weights;
    weights.pos_unit = 1.0; // meters
    weights.w_pos = 1.0;
    weights.w_rot = 1.0;

    JointLimits limits;
    limits.lower = Vector6d::Constant(-M_PI);
    limits.upper = Vector6d::Constant(M_PI);
    limits.max_step = Vector6d::Constant(10.0 * M_PI / 180.0);
    limits.clamp_to_limits = true;

    IkOptions opt;
    opt.max_iters = 120;
    opt.tol = 1e-6;
    opt.step_size = 0.8;

    std::mt19937 rng(0);
    std::uniform_real_distribution<double> uni(-M_PI, M_PI);

    std::vector<Vector6d> seeds(5);

    constexpr int kTrials = 20;
    constexpr double kPosTol = 1e-6; // meters

    for (int t = 0; t < kTrials; ++t) {
        Vector6d q_target;
        for (int i = 0; i < 6; ++i) q_target(i) = uni(rng);

        const robot::Matrix4d T_target = arm.computeFK(q_target);

        seeds[0] = q_target;
        seeds[1] = Vector6d::Zero();
        for (int k = 2; k < 5; ++k) {
            for (int i = 0; i < 6; ++i) seeds[k](i) = uni(rng);
        }

        const IkResult r = solver.solveBestOf(T_target, seeds, opt, &limits, weights, &ws, &q_target);
        if (r.status != IkStatus::kSuccess) {
            std::cerr << "UR5 IK failed at t=" << t << " status=" << static_cast<int>(r.status)
                      << " final_error=" << r.final_error << "\n";
            return 1;
        }

        const robot::Matrix4d T2 = arm.computeFK(r.q);
        if (!isPoseClose(T_target, T2, kPosTol)) {
            std::cerr << "UR5 FK/IK mismatch at t=" << t << "\n";
            return 1;
        }
    }

    std::cout << "test_ik_ur5 PASS\n";
    return 0;
}
