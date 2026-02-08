import math
import os
import random
import time

import industrial_robot_core as irc

try:
    import matplotlib.pyplot as plt
except ImportError as e:
    raise SystemExit(
        "matplotlib is required for visualization. Install with: python -m pip install matplotlib"
    ) from e


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def ur5_dh_m():
    """UR5 (classic/commonly-used DH table), units: meters.

    Parameters: (a, alpha, d, theta_offset)

    Note: This is a commonly used UR5 DH parameterization (standard UR5 DH table).
    """

    # a (m), alpha (rad), d (m), theta_offset (rad)
    rows = [
        (0.0, math.pi / 2.0, 0.089159, 0.0),
        (-0.425, 0.0, 0.0, 0.0),
        (-0.39225, 0.0, 0.0, 0.0),
        (0.0, math.pi / 2.0, 0.10915, 0.0),
        (0.0, -math.pi / 2.0, 0.09465, 0.0),
        (0.0, 0.0, 0.0823, 0.0),
    ]

    dh = []
    for a, alpha, d, theta in rows:
        p = irc.DHParam()
        p.a = float(a)
        p.alpha = float(alpha)
        p.d = float(d)
        p.theta = float(theta)
        dh.append(p)
    return dh


def set_axes_equal_3d(ax, xs, ys, zs):
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    z_min, z_max = min(zs), max(zs)

    cx = 0.5 * (x_min + x_max)
    cy = 0.5 * (y_min + y_max)
    cz = 0.5 * (z_min + z_max)

    radius = 0.5 * max(x_max - x_min, y_max - y_min, z_max - z_min, 1e-6)

    ax.set_xlim(cx - radius, cx + radius)
    ax.set_ylim(cy - radius, cy + radius)
    ax.set_zlim(cz - radius, cz + radius)


def main():
    rng = random.Random(0)

    arm = irc.RobotArm(ur5_dh_m())
    if not arm.isValid():
        raise RuntimeError("RobotArm invalid")

    solver = irc.IKSolverDls(arm)

    weights = irc.TaskSpaceWeights()
    weights.pos_unit = 1.0  # UR5 DH is in meters
    weights.w_pos = 1.0
    weights.w_rot = 1.0

    # Rough UR5 joint limits (radians). Use symmetric bounds for demo purposes.
    limits = irc.JointLimits()
    limits.lower = [deg2rad(-180.0)] * 6
    limits.upper = [deg2rad(180.0)] * 6
    limits.max_step = [deg2rad(10.0)] * 6
    limits.clamp_to_limits = True

    opt = irc.IkOptions()
    opt.max_iters = 120
    opt.tol = 1e-6
    opt.step_size = 0.8

    n = 100
    successes = 0
    total_iters = 0
    total_time_s = 0.0

    # Reuse a fixed-size seed list (no growth/extra allocations).
    # Seeds: [q_target, zero, random, random, random]
    seeds = [[0.0] * 6 for _ in range(5)]
    zeros = [0.0] * 6

    for _ in range(n):
        q_target = [rng.uniform(deg2rad(-180.0), deg2rad(180.0)) for _ in range(6)]
        T_target = arm.computeFK(q_target)

        seeds[0] = q_target
        seeds[1] = zeros
        for k in range(2, 5):
            seeds[k] = [rng.uniform(deg2rad(-180.0), deg2rad(180.0)) for _ in range(6)]

        t0 = time.perf_counter()
        res = solver.solveBestOf(T_target, seeds, opt, limits, weights, q_target)
        t1 = time.perf_counter()

        total_time_s += t1 - t0
        total_iters += int(res.iterations)

        if res.status == irc.IkStatus.kSuccess:
            successes += 1

    success_rate = 100.0 * successes / float(n)
    avg_iters = total_iters / float(n)
    avg_time_ms = 1000.0 * total_time_s / float(n)

    print(f"N={n}")
    print(f"success_rate={success_rate:.1f}%")
    print(f"avg_iterations={avg_iters:.2f}")
    print(f"avg_solve_time={avg_time_ms:.3f} ms")

    # Render one pose from a known configuration.
    q_draw = [0.35, -0.25, 0.20, 0.50, -0.40, 0.30]
    chain = arm.forwardKinematicsChain(q_draw)
    xs = [p[0] for p in chain]
    ys = [p[1] for p in chain]
    zs = [p[2] for p in chain]

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(xs, ys, zs, marker="o", linewidth=2.0, markersize=5)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("UR5 FK Pose (Stick Figure)")
    set_axes_equal_3d(ax, xs, ys, zs)

    out_path = os.path.join(os.path.dirname(__file__), "ur5_pose.png")
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)

    size = os.path.getsize(out_path)
    print("saved:", out_path)
    print("size_bytes:", size)

    if size <= 0:
        raise RuntimeError("Output image is empty")


if __name__ == "__main__":
    main()
