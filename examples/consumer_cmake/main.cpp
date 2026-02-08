#include "robot_arm.hpp"

#include <iostream>
#include <vector>

int main() {
    // Minimal IRB120-like DH (mm) just to verify external linking works.
    std::vector<robot::DHParam> dh = {
        {0.0, -M_PI / 2.0, 290.0, 0.0},
        {270.0, 0.0, 0.0, -M_PI / 2.0},
        {70.0, -M_PI / 2.0, 0.0, 0.0},
        {0.0, M_PI / 2.0, 302.0, 0.0},
        {0.0, -M_PI / 2.0, 0.0, 0.0},
        {0.0, 0.0, 72.0, 0.0},
    };

    robot::RobotArm arm(dh);
    if (!arm.isValid()) {
        std::cerr << "RobotArm invalid\n";
        return 2;
    }

    robot::Vector6d q;
    q << 0.1, -0.2, 0.3, 0.0, 0.0, 0.0;

    const robot::Matrix4d T = arm.computeFK(q);
    std::cout << "FK ok, x=" << T(0, 3) << "\n";
    return 0;
}
