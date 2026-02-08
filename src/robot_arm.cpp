#include "robot_arm.hpp"

#include "robot/ik_solver_dls.hpp"

#include <array>
#include <cmath>

namespace robot {

RobotArm::RobotArm(const std::vector<DHParam>& dh_params) {
    valid_ = TryCreate(dh_params, this);
}

bool RobotArm::TryCreate(const std::vector<DHParam>& dh_params, RobotArm* out) noexcept {
    if (!out) return false;
    if (dh_params.size() != 6) {
        out->valid_ = false;
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        out->dh_[i] = dh_params[static_cast<size_t>(i)];
    }
    out->valid_ = true;
    return true;
}

Matrix4d RobotArm::dhToTransform(double a, double alpha, double d, double theta) {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);

    Matrix4d T;
    T << ct, -st * ca, st * sa, a * ct,
         st, ct * ca, -ct * sa, a * st,
         0.0, sa, ca, d,
         0.0, 0.0, 0.0, 1.0;
    return T;
}

Eigen::Vector3d RobotArm::so3LogVee(const Eigen::Matrix3d& R) noexcept {
    // Robust log map for SO(3).
    const double cos_theta_raw = 0.5 * (R.trace() - 1.0);
    const double cos_theta = std::max(-1.0, std::min(1.0, cos_theta_raw));
    const double theta = std::acos(cos_theta);

    // Near zero: log(R) ~ 0.5*(R - R^T)
    if (theta < 1e-9) {
        const Eigen::Matrix3d S = 0.5 * (R - R.transpose());
        return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
    }

    // Near pi: use diagonal-based axis extraction.
    if (M_PI - theta < 1e-6) {
        Eigen::Vector3d axis;
        axis.x() = std::sqrt(std::max(0.0, 0.5 * (R(0, 0) + 1.0)));
        axis.y() = std::sqrt(std::max(0.0, 0.5 * (R(1, 1) + 1.0)));
        axis.z() = std::sqrt(std::max(0.0, 0.5 * (R(2, 2) + 1.0)));

        // Pick signs using off-diagonals.
        if (R(2, 1) - R(1, 2) < 0) axis.x() = -axis.x();
        if (R(0, 2) - R(2, 0) < 0) axis.y() = -axis.y();
        if (R(1, 0) - R(0, 1) < 0) axis.z() = -axis.z();

        const double n = axis.norm();
        if (n < 1e-12) {
            axis = Eigen::Vector3d(1.0, 0.0, 0.0);
        } else {
            axis /= n;
        }
        return theta * axis;
    }

    const Eigen::Matrix3d S = (theta / (2.0 * std::sin(theta))) * (R - R.transpose());
    return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
}

Matrix4d RobotArm::computeFK(const Vector6d& joints) const {
    if (!valid_) return Matrix4d::Identity();

    Matrix4d T = Matrix4d::Identity();
    for (int i = 0; i < 6; ++i) {
        const double theta = joints(i) + dh_[i].theta;
        T = T * dhToTransform(dh_[i].a, dh_[i].alpha, dh_[i].d, theta);
    }
    return T;
}

std::array<Eigen::Vector3d, 7> RobotArm::forwardKinematicsChain(const Vector6d& joints) const {
    std::array<Eigen::Vector3d, 7> p{};
    if (!valid_) {
        for (auto& v : p) v.setZero();
        return p;
    }

    Matrix4d T = Matrix4d::Identity();
    p[0] = T.block<3, 1>(0, 3);

    for (int i = 0; i < 6; ++i) {
        const double theta = joints(i) + dh_[i].theta;
        T = T * dhToTransform(dh_[i].a, dh_[i].alpha, dh_[i].d, theta);
        p[i + 1] = T.block<3, 1>(0, 3);
    }

    return p;
}

Matrix6d RobotArm::computeJacobian(const Vector6d& joints) const {
    if (!valid_) return Matrix6d::Zero();

    std::array<Eigen::Vector3d, 7> p;
    std::array<Eigen::Vector3d, 7> z;

    Matrix4d T = Matrix4d::Identity();
    p[0] = T.block<3, 1>(0, 3);
    z[0] = T.block<3, 1>(0, 2);

    for (int i = 0; i < 6; ++i) {
        const double theta = joints(i) + dh_[i].theta;
        T = T * dhToTransform(dh_[i].a, dh_[i].alpha, dh_[i].d, theta);
        p[i + 1] = T.block<3, 1>(0, 3);
        z[i + 1] = T.block<3, 1>(0, 2);
    }

    const Eigen::Vector3d p_end = p[6];

    Matrix6d J;
    J.setZero();

    for (int i = 0; i < 6; ++i) {
        const Eigen::Vector3d zi = z[i];
        const Eigen::Vector3d pi = p[i];
        const Eigen::Vector3d Jv = zi.cross(p_end - pi);

        J.block<3, 1>(0, i) = Jv;
        J.block<3, 1>(3, i) = zi;
    }

    return J;
}

bool RobotArm::computeIK(const Matrix4d& target_pose, Vector6d& q) const {
    if (!valid_) return false;

    IKSolverDls solver(*this);
    TaskSpaceWeights weights;
    // Back-compat: previous demo is in mm; keep pos_unit=1 here.
    // Callers wanting unit consistency should pass weights to solver directly.
    const IkResult r = solver.solve(target_pose, q, IkOptions{}, nullptr, weights, nullptr);
    if (r.status == IkStatus::kSuccess) {
        q = r.q;
        return true;
    }
    q = r.q;
    return false;
}

} // namespace robot
