#!/usr/bin/env python3
"""
optimize.py – Simple evaluation / optimization driver.

1. Builds (or loads) a contour.
2. Runs a headless physics simulation of the stalk driving past it.
3. Produces two evaluation figures:
     - Contour kinematics (x, vx, ax vs y)   – from LinearCamContour
     - Stalk motion histories (pos, vel, acc vs time)
4. Prints a few scalar metrics that can later be turned into a cost function.

The physics path is identical to the interactive drive (same fixed step,
same contact iterations, same spring-damper + friction model).
"""

from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from contour import LinearCamContour, Contour, resample_contour_even_y
from physics import simulate_drive, RADIUS


def build_default_contour() -> tuple[LinearCamContour, Contour]:
    """Reproduce the original geomtry_opt example profile."""
    cam = LinearCamContour(vy=0.447)
    segments = [
        {'dir': 'lead', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.06},
        {'dir': 'lead', 'type': 'dwell', 'length': 0.100, 'x': 0.06},
        {'dir': 'lead', 'type': 'rise',  'length': 0.263, 'delta_x': -0.06},
        {'dir': 'lead', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.00},
        {'dir': 'lead', 'type': 'dwell', 'length': 0.050, 'x': -0.000},
        {'dir': 'lead', 'type': 'rise',  'length': 0.677, 'delta_x': -0.401},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.045, 'x': 0.06},
        {'dir': 'trail', 'type': 'rise',  'length': 0.263, 'delta_x': -0.06},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.050, 'x': 0.00},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.00},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.053, 'x': 0.00},
        {'dir': 'trail', 'type': 'rise',  'length': 0.390, 'delta_x': -0.401},
    ]
    cam.build_profile(segments, curve_func=cam.poly_345)

    # Resample for the physics engine (same density as drive.py)
    pts = cam.to_contour_points()
    pts = resample_contour_even_y(pts, 1000)
    return cam, Contour(pts)


def plot_stalk_motion(hist: dict, save_path: str) -> None:
    """Time-series plots of stalk position, velocity and acceleration."""
    t = hist["t"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    axes[0].plot(hist['origin_y'], hist["pos_x"], label="x (lateral)")
    # axes[0].plot(hist['origin_y'], hist["pos_y"], label="y (drive axis)")
    axes[0].set_ylabel("Position (m)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(hist['origin_y'], hist["vel_x"], label="vx")
    axes[1].plot(hist['origin_y'], hist["vel_y"], label="vy")
    axes[1].set_ylabel("Velocity (m/s)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(hist['origin_y'], hist["acc_x"], label="ax")
    axes[2].plot(hist['origin_y'], hist["acc_y"], label="ay")
    axes[2].set_ylabel("Acceleration (m/s²)")
    axes[2].set_xlabel("Origin Y-pos")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    # Highlight contact intervals
    contact = hist["contact"]
    for ax in axes:
        ymin, ymax = ax.get_ylim()
        ax.fill_between(hist['origin_y'], ymin, ymax, where=contact, color="red", alpha=0.08, label="_contact")
        ax.xaxis.set_inverted(True)

    fig.suptitle("Stalk Motion during Contour Drive")
    plt.tight_layout()
    plt.show(block=True)
    fig.savefig(save_path, dpi=150)
    
    print(f"Saved stalk motion figure → {save_path}")


def main() -> None:
    out_dir = os.path.dirname(__file__)

    print("=" * 60)
    print("1. Building contour …")
    cam, contour = build_default_contour()
    print(f"   Contour extent ≈ {contour.extent:.3f} m, "
          f"{len(contour.points)} points")

    # Export CSV so drive.py can also use it
    csv_path = os.path.join(out_dir, "new_profile.csv")
    cam.export_profile(path=csv_path)

    # Contour kinematics figure
    kin_png = os.path.join(out_dir, "contour_kinematics.png")
    cam.plot_kinematics(vy=cam.vy, save_path=kin_png)

    print("\n2. Running headless physics simulation …")
    # Duration long enough to traverse the whole profile at 0.10 m/s
    duration = (contour.y_max - contour.y_min) / 0.10 + 2.0
    hist = simulate_drive(
        contour,
        drive_speed=0.10,
        lateral_x=-0.020,
        duration=duration,
        record_every=20,          # ~ every 10 ms
    )
    print(f"   Simulated {hist['t'][-1]:.2f} s  ({len(hist['t'])} samples)")

    # Stalk motion figure
    motion_png = os.path.join(out_dir, "stalk_motion.png")
    plot_stalk_motion(hist, motion_png)

    # Simple scalar metrics (ready to become a cost function)
    peak_ax = float(np.max(np.abs(hist["acc_x"])))
    peak_ay = float(np.max(np.abs(hist["acc_y"])))
    peak_vx = float(np.max(np.abs(hist["vel_x"])))
    contact_fraction = float(np.mean(hist["contact"]))

    print("\n3. Evaluation metrics")
    print(f"   Peak lateral accel |ax|  : {peak_ax:8.2f} m/s²")
    print(f"   Peak drive-axis accel    : {peak_ay:8.2f} m/s²")
    print(f"   Peak lateral velocity    : {peak_vx:8.3f} m/s")
    print(f"   Contact time fraction    : {contact_fraction:8.3f}")

    print("\nDone.  Figures written to:")
    print(f"  {kin_png}")
    print(f"  {motion_png}")
    print(f"  Contour CSV : {csv_path}")


if __name__ == "__main__":
    main()
