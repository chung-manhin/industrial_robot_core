#pragma once

#include "robot/ik_types.hpp"

#include "robot_arm.hpp"

#include <vector>

namespace robot {

class IKSolverDls {
public:
    explicit IKSolverDls(const RobotArm& arm) : arm_(arm) {}

    IkResult solve(const Matrix4d& target_pose,
                   const Vector6d& seed,
                   const IkOptions& options = IkOptions{},
                   const JointLimits* limits = nullptr,
                   const TaskSpaceWeights& weights = TaskSpaceWeights{},
                   IkWorkspace* workspace = nullptr) const noexcept;

    IkResult solveBestOf(const Matrix4d& target_pose,
                         const std::vector<Vector6d>& seeds,
                         const IkOptions& options = IkOptions{},
                         const JointLimits* limits = nullptr,
                         const TaskSpaceWeights& weights = TaskSpaceWeights{},
                         IkWorkspace* workspace = nullptr,
                         const Vector6d* preferred = nullptr) const noexcept;

private:
    const RobotArm& arm_;
};

} // namespace robot
