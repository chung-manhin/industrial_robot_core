#pragma once

#include "robot/ik_types.hpp"

#include "robot_arm.hpp"

#include <vector>

namespace robot {

class IKSolverDls {
  public:
    explicit IKSolverDls(const RobotArm& arm) : arm_(arm) {}

    // Damped Least-Squares (DLS) IK.
    //
    // Error definition (6x1):
    //   e = [ w_pos * pos_unit * (p_des - p) ; w_rot * log(R_des * R^T) ]
    // where positions use the same unit as the DH model (often mm), and pos_unit
    // converts it to a consistent scale (e.g. 0.001 for mm->m) so that the
    // translational and rotational parts are comparable.
    //
    // Stopping criteria:
    //   - success: ||e|| < options.tol
    //   - failure: non-finite error/Jacobian solve, invalid inputs, or max_iters
    //
    // Damping strategy:
    //   λ is adapted from a manipulability measure w = sqrt(det(J J^T)) and
    //   clamped into [lambda_min, lambda_max]:
    //     λ = clamp(lambda0 * (w_ref / (w + eps)), lambda_min, lambda_max)
    IkResult solve(const Matrix4d& target_pose, const Vector6d& seed,
                   const IkOptions& options = IkOptions{}, const JointLimits* limits = nullptr,
                   const TaskSpaceWeights& weights = TaskSpaceWeights{},
                   IkWorkspace* workspace = nullptr) const noexcept;

    // Solve IK with multiple initial guesses and select the best result.
    //
    // Selection rules:
    //   1) any success beats any failure
    //   2) smaller final_error wins
    //   3) if tied and preferred is provided, choose the solution closer to it
    //
    // Early return:
    //   If a seed reaches tolerance (final_error <= options.tol), return it
    //   immediately without trying remaining seeds.
    IkResult solveBestOf(const Matrix4d& target_pose, const std::vector<Vector6d>& seeds,
                         const IkOptions& options = IkOptions{},
                         const JointLimits* limits = nullptr,
                         const TaskSpaceWeights& weights = TaskSpaceWeights{},
                         IkWorkspace* workspace = nullptr,
                         const Vector6d* preferred = nullptr) const noexcept;

  private:
    const RobotArm& arm_;
};

} // namespace robot
