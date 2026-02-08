#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace robot {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;

struct DHParam {
    double a;     // link length (mm or m, consistent)
    double alpha; // link twist (rad)
    double d;     // link offset
    double theta; // joint angle offset (rad)
};

class RobotArm {
  public:
    RobotArm() = default;
    explicit RobotArm(const std::vector<DHParam>& dh_params);

    // Soft-realtime friendly creation helper: never throws.
    static bool TryCreate(const std::vector<DHParam>& dh_params, RobotArm* out) noexcept;

    bool isValid() const noexcept { return valid_; }

    // Forward kinematics: returns base->tool homogeneous transform
    Matrix4d computeFK(const Vector6d& joints) const;

    // Forward kinematics chain: base and all 6 joint positions (7 points total).
    std::array<Eigen::Vector3d, 7> forwardKinematicsChain(const Vector6d& joints) const;

    // Geometric Jacobian (spatial, expressed in base frame): [v; w]
    Matrix6d computeJacobian(const Vector6d& joints) const;

    // Convenience DLS IK wrapper (non-realtime convenience).
    // - target_pose: desired base->tool pose
    // - initial_guess: in/out, initial joints and solution
    // Returns true if converged.
    bool computeIK(const Matrix4d& target_pose, Vector6d& initial_guess) const;

    // SO(3) log map: returns rotation vector (axis * angle) in radians.
    static Eigen::Vector3d so3LogVee(const Eigen::Matrix3d& R) noexcept;

  private:
    std::array<DHParam, 6> dh_{};
    bool valid_ = false;

    static Matrix4d dhToTransform(double a, double alpha, double d, double theta);
};

} // namespace robot
