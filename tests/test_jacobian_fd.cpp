#include "robot_arm.hpp"

#include <Eigen/Dense>

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using robot::DHParam;
using robot::RobotArm;
using robot::Vector6d;

static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

static Eigen::Vector3d vee(const Eigen::Matrix3d& S) {
    return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
}

int main() {
    const std::vector<DHParam> dh = {
        {0.0, deg2rad(-90.0), 290.0, 0.0}, {270.0, deg2rad(0.0), 0.0, deg2rad(-90.0)},
        {70.0, deg2rad(-90.0), 0.0, 0.0},  {0.0, deg2rad(90.0), 302.0, 0.0},
        {0.0, deg2rad(-90.0), 0.0, 0.0},   {0.0, deg2rad(0.0), 72.0, 0.0}};

    RobotArm arm(dh);
    assert(arm.isValid());

    Vector6d q;
    q << 0.2, -0.1, 0.3, -0.4, 0.5, -0.2;

    const auto T0 = arm.computeFK(q);
    const Eigen::Vector3d p0 = T0.block<3, 1>(0, 3);
    const Eigen::Matrix3d R0 = T0.block<3, 3>(0, 0);

    const auto J = arm.computeJacobian(q);

    const double h = 1e-6;

    for (int i = 0; i < 6; ++i) {
        Vector6d qp = q;
        Vector6d qm = q;
        qp(i) += h;
        qm(i) -= h;

        const auto Tp = arm.computeFK(qp);
        const auto Tm = arm.computeFK(qm);

        const Eigen::Vector3d pp = Tp.block<3, 1>(0, 3);
        const Eigen::Vector3d pm = Tm.block<3, 1>(0, 3);

        const Eigen::Vector3d v_fd = (pp - pm) / (2.0 * h);

        const Eigen::Matrix3d Rp = Tp.block<3, 3>(0, 0);
        const Eigen::Matrix3d Rm = Tm.block<3, 3>(0, 0);

        // small-angle approximation in body-ish form: dR ~ (Rp * R0^T - Rm * R0^T)/(2h)
        const Eigen::Matrix3d dR = (Rp * R0.transpose() - Rm * R0.transpose()) / (2.0 * h);
        const Eigen::Vector3d w_fd = vee(0.5 * (dR - dR.transpose()));

        const Eigen::Vector3d v_j = J.block<3, 1>(0, i);
        const Eigen::Vector3d w_j = J.block<3, 1>(3, i);

        const double v_err = (v_fd - v_j).norm();
        const double w_err = (w_fd - w_j).norm();

        if (!(v_err < 1e-3 && w_err < 1e-3)) {
            std::cerr << "Jacobian FD mismatch on joint " << i << ": v_err=" << v_err
                      << " w_err=" << w_err << "\n";
            return 1;
        }
    }

    std::cout << "test_jacobian_fd PASS\n";
    return 0;
}
