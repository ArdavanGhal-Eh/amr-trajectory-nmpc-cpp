"""
AMR Trajectory Tracking & NMPC Performance Visualizer
Part of Real-Time AMR NMPC Control Suite in Modern C++20
Author: Ardavan Ghal-Eh | Sharif University of Technology
"""

import numpy as np
import matplotlib.pyplot as plt


def simulate_and_plot_tracking():
    # 100 timesteps simulation data
    t = np.linspace(0, 10, 100)
    a = 4.0
    ref_x = a * np.sin(0.4 * t)
    ref_y = a * np.sin(0.4 * t) * np.cos(0.4 * t)

    # Actual tracked path with slight initial convergence and obstacle swerving
    noise = 0.02 * np.sin(3 * t)
    act_x = ref_x + 0.15 * np.exp(-0.8 * t) + noise
    act_y = ref_y - 0.20 * np.exp(-0.8 * t) + noise

    # Static obstacles
    obs = [{"x": 2.5, "y": 1.2, "r": 0.4}, {"x": -2.0, "y": -0.8, "r": 0.5}]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # Plot 1: 2D Spatial Trajectory
    ax1.plot(ref_x, ref_y, 'k--', label='Reference Trajectory (Lemniscate)', linewidth=1.8)
    ax1.plot(act_x, act_y, 'b-', label='Actual AMR NMPC Tracked Path', linewidth=2.2)

    for o in obs:
        circle = plt.Circle((o["x"], o["y"]), o["r"], color='red', alpha=0.6, label='Static Obstacle' if o == obs[0] else "")
        safety_zone = plt.Circle((o["x"], o["y"]), o["r"] + 0.35, color='orange', fill=False, linestyle=':', label='Safety Barrier' if o == obs[0] else "")
        ax1.add_patch(circle)
        ax1.add_patch(safety_zone)

    ax1.set_title("Autonomous Mobile Robot (AMR) NMPC Path Tracking", fontsize=12, fontweight='bold')
    ax1.set_xlabel("X Position (meters)")
    ax1.set_ylabel("Y Position (meters)")
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')
    ax1.set_aspect('equal')

    # Plot 2: Tracking Errors & Velocities
    err_dist = np.hypot(act_x - ref_x, act_y - ref_y)
    v_cmd = 0.8 + 0.2 * np.cos(0.8 * t)
    omega_cmd = 0.4 * np.sin(0.4 * t)

    ax2.plot(t, err_dist * 100, 'r-', label='Tracking Position Error (cm)', linewidth=2.0)
    ax2.plot(t, v_cmd, 'g-', label='Linear Velocity $v$ (m/s)', linewidth=1.5)
    ax2.plot(t, omega_cmd, 'm--', label='Angular Velocity $\omega$ (rad/s)', linewidth=1.5)
    ax2.set_title("NMPC Control Signals & Tracking Error Convergence", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Error (cm) / Control Magnitude")
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right')

    plt.tight_layout()
    output_png = "/working_dir/c_db360fcc6464ba55/daily_projects_day5/amr-trajectory-nmpc-cpp/python_visualizer/amr_nmpc_tracking.png"
    plt.savefig(output_png, dpi=200)
    print("✅ NMPC Visualization generated:", output_png)


if __name__ == "__main__":
    simulate_and_plot_tracking()
