#include "robot_arm.hpp"

#include "robot/ik_solver_dls.hpp"
#include "robot/ik_types.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numeric>
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

static double deg2rad(double deg) { return deg * M_PI / 180.0; }

static std::vector<DHParam> makeIrb120Dh() {
    // ABB IRB 120 standard DH. Units: mm.
    return {
        {0.0, deg2rad(-90.0), 290.0, 0.0},           // 1
        {270.0, deg2rad(0.0), 0.0, deg2rad(-90.0)},  // 2 (offset)
        {70.0, deg2rad(-90.0), 0.0, 0.0},            // 3
        {0.0, deg2rad(90.0), 302.0, 0.0},            // 4
        {0.0, deg2rad(-90.0), 0.0, 0.0},             // 5
        {0.0, deg2rad(0.0), 72.0, 0.0}               // 6
    };
}

int main() {
    constexpr int kTrials = 10000;

    RobotArm arm(makeIrb120Dh());
    if (!arm.isValid()) {
        std::cerr << "RobotArm invalid\n";
        return 2;
    }

    IKSolverDls solver(arm);
    robot::IkWorkspace ws;

    TaskSpaceWeights weights;
    weights.pos_unit = 0.001; // mm -> m
    weights.w_pos = 1.0;
    weights.w_rot = 1.0;

    JointLimits limits;
    limits.lower << deg2rad(-170), deg2rad(-110), deg2rad(-170), deg2rad(-190), deg2rad(-120), deg2rad(-400);
    limits.upper << deg2rad(170), deg2rad(110), deg2rad(170), deg2rad(190), deg2rad(120), deg2rad(400);
    limits.max_step = Vector6d::Constant(deg2rad(10.0));
    limits.clamp_to_limits = true;

    IkOptions opt;
    opt.max_iters = 120;
    opt.tol = 1e-6;
    opt.step_size = 0.8;
    opt.lambda0 = 1e-2;
    opt.lambda_min = 1e-6;
    opt.lambda_max = 1e-1;
    opt.w_ref = 1.0;

    std::mt19937 rng(0);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);

    std::vector<double> times_us;
    times_us.reserve(kTrials);

    int success = 0;

    std::vector<Vector6d> seeds(5);

    // Warm up
    {
        Vector6d q0 = Vector6d::Zero();
        const auto T = arm.computeFK(q0);
        seeds[0] = q0;
        seeds[1] = Vector6d::Zero();
        seeds[2] = q0;
        seeds[3] = q0;
        seeds[4] = q0;
        (void)solver.solveBestOf(T, seeds, opt, &limits, weights, &ws, &q0);
    }

    for (int t = 0; t < kTrials; ++t) {
        // Sample a random reachable target by FK from a random joint vector.
        Vector6d q_target;
        for (int i = 0; i < 6; ++i) q_target(i) = uni(rng);

        const robot::Matrix4d T_target = arm.computeFK(q_target);

        // Exactly 5 seeds: [q_target, zero, random, random, random].
        seeds[0] = q_target;
        seeds[1] = Vector6d::Zero();
        for (int k = 2; k < 5; ++k) {
            for (int i = 0; i < 6; ++i) seeds[k](i) = uni(rng);
        }

        const auto t0 = std::chrono::high_resolution_clock::now();
        const IkResult r = solver.solveBestOf(T_target, seeds, opt, &limits, weights, &ws, &q_target);
        const auto t1 = std::chrono::high_resolution_clock::now();

        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        times_us.push_back(us);

        if (r.status == IkStatus::kSuccess) ++success;
    }

    std::sort(times_us.begin(), times_us.end());

    const double sum = std::accumulate(times_us.begin(), times_us.end(), 0.0);
    const double avg = sum / static_cast<double>(times_us.size());
    const size_t p99_idx = static_cast<size_t>(std::floor(0.99 * (times_us.size() - 1)));
    const double p99 = times_us[p99_idx];
    const double max = times_us.back();
    const double succ_rate = static_cast<double>(success) / static_cast<double>(kTrials);

    std::cout.setf(std::ios::fixed);
    std::cout.precision(3);

    std::cout << "IK Benchmark (" << kTrials << " trials)\n";
    std::cout << "Avg (us): " << avg << "\n";
    std::cout << "P99 (us): " << p99 << "\n";
    std::cout << "Max (us): " << max << "\n";
    std::cout << "Success rate: " << (succ_rate * 100.0) << "%\n";

    return 0;
}
