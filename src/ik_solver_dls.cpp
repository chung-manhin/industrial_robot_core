#include "robot/ik_solver_dls.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace robot {

namespace {

inline bool isFinite6(const Vector6d& v) {
    return v.allFinite();
}

inline double clampd(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

inline void clampQ(Vector6d* q, const JointLimits& limits) {
    for (int i = 0; i < 6; ++i) {
        (*q)(i) = clampd((*q)(i), limits.lower(i), limits.upper(i));
    }
}

inline bool violatesLimits(const Vector6d& q, const JointLimits& limits) {
    for (int i = 0; i < 6; ++i) {
        if (q(i) < limits.lower(i) - 1e-12)
            return true;
        if (q(i) > limits.upper(i) + 1e-12)
            return true;
    }
    return false;
}

inline void limitStep(Vector6d* dq, const Vector6d& max_step) {
    for (int i = 0; i < 6; ++i) {
        const double m = max_step(i);
        if (std::isfinite(m) && m > 0.0) {
            (*dq)(i) = clampd((*dq)(i), -m, m);
        }
    }
}

inline double manipulabilityFromJJt(const Eigen::Matrix<double, 6, 6>& JJt, double eps) {
    // w = sqrt(det(JJ^T)). 6x6 fixed size.
    const double det = JJt.determinant();
    if (!std::isfinite(det))
        return 0.0;
    return std::sqrt(std::max(0.0, det) + eps);
}

} // namespace

IkResult IKSolverDls::solve(const Matrix4d& target_pose, const Vector6d& seed,
                            const IkOptions& options, const JointLimits* limits,
                            const TaskSpaceWeights& weights,
                            IkWorkspace* workspace) const noexcept {
    IkResult out;

    if (!arm_.isValid()) {
        out.status = IkStatus::kInvalidInput;
        return out;
    }

    if (!isFinite6(seed) || !target_pose.allFinite()) {
        out.status = IkStatus::kInvalidInput;
        return out;
    }

    if (options.max_iters <= 0 || !(options.tol > 0.0) || !(options.step_size > 0.0)) {
        out.status = IkStatus::kInvalidInput;
        return out;
    }

    IkWorkspace local_ws;
    IkWorkspace* ws = workspace ? workspace : &local_ws;

    Vector6d q = seed;
    if (limits && limits->clamp_to_limits) {
        clampQ(&q, *limits);
    } else if (limits && violatesLimits(q, *limits)) {
        out.status = IkStatus::kLimitViolation;
        out.q = q;
        return out;
    }

    const Eigen::Vector3d p_des = target_pose.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_des = target_pose.block<3, 3>(0, 0);

    for (int iter = 0; iter < options.max_iters; ++iter) {
        out.iterations = iter;

        const Matrix4d T = arm_.computeFK(q);
        const Eigen::Vector3d p = T.block<3, 1>(0, 3);
        const Eigen::Matrix3d R = T.block<3, 3>(0, 0);

        const Eigen::Vector3d e_p = p_des - p; // same unit as DH
        const Eigen::Matrix3d R_err = R_des * R.transpose();
        const Eigen::Vector3d e_w = RobotArm::so3LogVee(R_err);

        // 6D task-space error used by the solver.
        // - translation error uses DH length unit (often mm) and is scaled by pos_unit
        // - rotation error uses so(3) logarithm (axis-angle vector in radians)
        ws->e.template block<3, 1>(0, 0) = weights.w_pos * (weights.pos_unit * e_p);
        ws->e.template block<3, 1>(3, 0) = weights.w_rot * e_w;

        const double err = ws->e.norm();
        out.final_error = err;
        if (!std::isfinite(err)) {
            out.status = IkStatus::kNumericalIssue;
            out.q = q;
            return out;
        }

        // Stop condition: reach requested tolerance.
        if (err < options.tol) {
            out.status = IkStatus::kSuccess;
            out.q = q;
            return out;
        }

        ws->J = arm_.computeJacobian(q);
        // Weight + unit scaling consistent with error definition above.
        ws->J.template block<3, 6>(0, 0) *= (weights.w_pos * weights.pos_unit);
        ws->J.template block<3, 6>(3, 0) *= weights.w_rot;

        ws->JJt.noalias() = ws->J * ws->J.transpose();

        // Adaptive damping based on manipulability.
        // w = sqrt(det(JJ^T)) (with an epsilon guard), then:
        //   λ = clamp(lambda0 * (w_ref / (w + eps)), [lambda_min, lambda_max])
        // DLS solve uses:
        //   dq = J^T (J J^T + λ^2 I)^{-1} e
        const double w = manipulabilityFromJJt(ws->JJt, options.eps);
        const double lambda = clampd(options.lambda0 * (options.w_ref / (w + options.eps)),
                                     options.lambda_min, options.lambda_max);

        ws->A = ws->JJt;
        ws->A.diagonal().array() += (lambda * lambda);

        ws->ldlt.compute(ws->A);
        if (ws->ldlt.info() != Eigen::Success) {
            out.status = IkStatus::kNumericalIssue;
            out.q = q;
            return out;
        }

        ws->dq.noalias() = ws->J.transpose() * ws->ldlt.solve(ws->e);
        if (!ws->dq.allFinite()) {
            out.status = IkStatus::kNumericalIssue;
            out.q = q;
            return out;
        }

        if (limits) {
            limitStep(&ws->dq, limits->max_step);
        }

        // Optional nullspace avoidance (soft constraint): push towards joint mid-range.
        if (limits && limits->enable_avoidance && limits->avoidance_gain > 0.0) {
            // J_pinv = J^T (JJ^T + λ^2 I)^-1
            ws->J_pinv.noalias() =
                ws->J.transpose() * ws->ldlt.solve(Eigen::Matrix<double, 6, 6>::Identity());
            ws->N.noalias() = Eigen::Matrix<double, 6, 6>::Identity() - ws->J_pinv * ws->J;

            Vector6d q_mid = 0.5 * (limits->lower + limits->upper);
            Vector6d range = (limits->upper - limits->lower);
            for (int i = 0; i < 6; ++i) {
                if (std::abs(range(i)) < 1e-12)
                    range(i) = 1.0;
            }
            const Vector6d g = (q - q_mid).cwiseQuotient(range);
            const Vector6d dq_avoid = -limits->avoidance_gain * g;

            ws->dq.noalias() += ws->N * dq_avoid;
            if (limits) {
                limitStep(&ws->dq, limits->max_step);
            }
        }

        q.noalias() += options.step_size * ws->dq;

        if (limits) {
            if (limits->clamp_to_limits) {
                clampQ(&q, *limits);
            } else if (violatesLimits(q, *limits)) {
                out.status = IkStatus::kLimitViolation;
                out.q = q;
                return out;
            }
        }
    }

    out.status = IkStatus::kNoConvergence;
    out.q = q;
    return out;
}

IkResult IKSolverDls::solveBestOf(const Matrix4d& target_pose, const std::vector<Vector6d>& seeds,
                                  const IkOptions& options, const JointLimits* limits,
                                  const TaskSpaceWeights& weights, IkWorkspace* workspace,
                                  const Vector6d* preferred) const noexcept {
    IkResult best;
    bool best_set = false;

    for (const auto& seed : seeds) {
        IkWorkspace local_ws;
        IkWorkspace* ws = workspace ? workspace : &local_ws;

        const IkResult r = solve(target_pose, seed, options, limits, weights, ws);

        const bool ok = (r.status == IkStatus::kSuccess);
        if (!best_set) {
            best = r;
            best_set = true;
        } else {
            const bool best_ok = (best.status == IkStatus::kSuccess);
            if (ok != best_ok) {
                if (ok)
                    best = r;
            } else if (r.final_error < best.final_error - 1e-12) {
                best = r;
            } else if (preferred && std::abs(r.final_error - best.final_error) <= 1e-12) {
                const double d_r = (r.q - *preferred).norm();
                const double d_b = (best.q - *preferred).norm();
                if (d_r < d_b)
                    best = r;
            }
        }

        // Multi-seed early return: if any seed already meets tolerance, return it
        // immediately without evaluating remaining seeds.
        if (best.status == IkStatus::kSuccess && best.final_error <= options.tol) {
            return best;
        }
    }

    if (!best_set) {
        best.status = IkStatus::kInvalidInput;
    }

    return best;
}

} // namespace robot
