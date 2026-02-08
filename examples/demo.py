import math
import os
import sys

import industrial_robot_core as irc

try:
    import matplotlib.pyplot as plt
except ImportError:
    print(
        "matplotlib is required for visualization. Install with: python3 -m pip install matplotlib"
    )
    sys.exit(1)


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def irb120_dh_mm():
    dh = []

    p = irc.DHParam()
    p.a = 0.0
    p.alpha = deg2rad(-90.0)
    p.d = 290.0
    p.theta = 0.0
    dh.append(p)

    p = irc.DHParam()
    p.a = 270.0
    p.alpha = 0.0
    p.d = 0.0
    p.theta = deg2rad(-90.0)
    dh.append(p)

    p = irc.DHParam()
    p.a = 70.0
    p.alpha = deg2rad(-90.0)
    p.d = 0.0
    p.theta = 0.0
    dh.append(p)

    p = irc.DHParam()
    p.a = 0.0
    p.alpha = deg2rad(90.0)
    p.d = 302.0
    p.theta = 0.0
    dh.append(p)

    p = irc.DHParam()
    p.a = 0.0
    p.alpha = deg2rad(-90.0)
    p.d = 0.0
    p.theta = 0.0
    dh.append(p)

    p = irc.DHParam()
    p.a = 0.0
    p.alpha = 0.0
    p.d = 72.0
    p.theta = 0.0
    dh.append(p)

    return dh


def set_axes_equal_3d(ax, xs, ys, zs):
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    z_min, z_max = min(zs), max(zs)

    cx = 0.5 * (x_min + x_max)
    cy = 0.5 * (y_min + y_max)
    cz = 0.5 * (z_min + z_max)

    radius = 0.5 * max(x_max - x_min, y_max - y_min, z_max - z_min, 1.0)

    ax.set_xlim(cx - radius, cx + radius)
    ax.set_ylim(cy - radius, cy + radius)
    ax.set_zlim(cz - radius, cz + radius)


def main():
    arm = irc.RobotArm(irb120_dh_mm())
    if not arm.isValid():
        raise RuntimeError("RobotArm invalid")

    # 1) Set a reachable target pose using known q_target.
    q_target = [0.35, -0.25, 0.20, 0.50, -0.40, 0.30]
    T_target = arm.computeFK(q_target)

    # 2) Solve IK with solveBestOf and exactly 5 seeds.
    solver = irc.IKSolverDls(arm)

    weights = irc.TaskSpaceWeights()
    weights.pos_unit = 0.001
    weights.w_pos = 1.0
    weights.w_rot = 1.0

    limits = irc.JointLimits()
    limits.lower = [
        deg2rad(-170),
        deg2rad(-110),
        deg2rad(-170),
        deg2rad(-190),
        deg2rad(-120),
        deg2rad(-400),
    ]
    limits.upper = [
        deg2rad(170),
        deg2rad(110),
        deg2rad(170),
        deg2rad(190),
        deg2rad(120),
        deg2rad(400),
    ]
    limits.max_step = [deg2rad(10.0)] * 6
    limits.clamp_to_limits = True

    opt = irc.IkOptions()
    opt.max_iters = 120
    opt.tol = 1e-6
    opt.step_size = 0.8

    seeds = [
        q_target,
        [0.0] * 6,
        [0.3, -0.2, 0.1, 0.4, 0.0, -0.3],
        [-0.5, 0.1, -0.2, 0.2, -0.1, 0.5],
        [0.2, 0.2, -0.4, 0.1, 0.3, -0.2],
    ]

    res = solver.solveBestOf(T_target, seeds, opt, limits, weights, q_target)

    print("status:", res.status)
    print("iterations:", res.iterations)
    print("final_error:", res.final_error)
    print("q_solution:", list(res.q))

    if res.status != irc.IkStatus.kSuccess:
        raise RuntimeError("IK failed, cannot draw pose")

    # 3) FK chain points (base + 6 joints)
    chain = arm.forwardKinematicsChain(res.q)
    xs = [p[0] for p in chain]
    ys = [p[1] for p in chain]
    zs = [p[2] for p in chain]

    # 4) 3D stick figure
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(xs, ys, zs, marker="o", linewidth=2.0, markersize=5)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("IRB120 IK Pose (Stick Figure)")
    set_axes_equal_3d(ax, xs, ys, zs)

    out_path = os.path.join(os.path.dirname(__file__), "robot_pose.png")
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    print("saved:", out_path)


if __name__ == "__main__":
    main()
