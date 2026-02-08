#include "robot_arm.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using robot::DHParam;
using robot::RobotArm;
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

    Vector6d q;
    q << 0.1, 0.2, -0.3, 0.4, -0.5, 0.6;

    const auto T = arm.computeFK(q);
    assert(T.allFinite());

    // Basic sanity: homogeneous matrix last row.
    assert(std::abs(T(3, 0)) < 1e-12);
    assert(std::abs(T(3, 1)) < 1e-12);
    assert(std::abs(T(3, 2)) < 1e-12);
    assert(std::abs(T(3, 3) - 1.0) < 1e-12);

    std::cout << "test_fk PASS\n";
    return 0;
}
