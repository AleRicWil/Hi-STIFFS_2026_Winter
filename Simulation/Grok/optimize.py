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
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from contour import LinearCamContour, Contour, resample_contour_even_y
from physics import simulate_drive, RADIUS

DRIVE_SPEED_MPS = 0.1   # speed of stalk-origin in -Y
MEAS_ZONES = [[None, None], [0.100, 0.000], [None, None]] # array of y-values marking the start (bigger y) and end (smaller y) of each sensor's measurement zone (exposed cantilever)

SEGMENTS = [
        # ---- LEAD (+y) ----
        {'dir': 'lead', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.06},     # start of this segment is y=0 of B-sensor, goes forward by 'length'
        {'dir': 'lead', 'type': 'dwell', 'length': 0.001, 'x': 0.06},
        {'dir': 'lead', 'type': 'rise',  'length': 0.700, 'delta_x': -0.06, 'tp': 0.5},
        {'dir': 'lead', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.00},
        {'dir': 'lead', 'type': 'dwell', 'length': 0.001, 'x': -0.000},
        {'dir': 'lead', 'type': 'rise',  'length': 0.677, 'delta_x': -0.150, 'tp': 0.5},
        # ---- TRAIL (−y) ----
        {'dir': 'trail', 'type': 'dwell', 'length': 0.045, 'x': 0.06},          # accomodate tail of B-sensor, start of this segment is y=0 of B-sensor, goes backward by 'length'
        {'dir': 'trail', 'type': 'rise',  'length': 0.700, 'delta_x': -0.06, 'tp': 0.7},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.001, 'x': 0.00},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.00},    # end of this segment is y=0 of C-sensor
        {'dir': 'trail', 'type': 'dwell', 'length': 0.053, 'x': 0.00},          # accomodate tail of C-sensor
        {'dir': 'trail', 'type': 'rise',  'length': 0.390, 'delta_x': -0.150, 'tp': 0.5},
    ]

def build_default_contour() -> tuple[LinearCamContour, Contour]:
    """Reproduce the original geomtry_opt example profile."""
    tail_length = 0.077362   # distance behind y=0 that contour must clear with >= 2.4mm
    cam = LinearCamContour(vy=0.447)
    segments = SEGMENTS
    cam.build_profile(segments, curve_func=cam.poly_3456)

    # ------------------------------------------------------------------
    # Locate the sensor origins in global y, then sample the contour
    # a distance tail_length behind each of them.
    # ------------------------------------------------------------------
    y_B0 = 0.0          # by construction of build_profile

    # A-sensor y=0 is the *end* of the first lead dwell (length 0.100+0.015).
    # Walk the lead segments exactly as build_profile does.
    y = 0.0
    y_A0 = None
    for seg in segments:
        if seg.get('dir') != 'lead':
            continue
        y += float(seg['length'])
        if (seg['type'] == 'dwell'
                and abs(float(seg['length']) - (0.100 + 0.015)) < 1e-9
                and abs(float(seg.get('x', 99.0))) < 1e-9):
            y_A0 = y - 0.100 - 0.015
            break
    if y_A0 is None:
        raise RuntimeError("Could not locate A-sensor y=0 dwell in segment list")

    # Walk the trail segments exactly as build_profile does until we finish
    # the dwell that ends at C-sensor y=0.
    y = 0.0
    y_C0 = None
    for seg in segments:
        if seg.get('dir') != 'trail':
            continue
        y -= float(seg['length'])
        # Identify the annotated C-zero dwell by its exact length & x
        if (seg['type'] == 'dwell'
                and abs(float(seg['length']) - (0.100 + 0.015)) < 1e-9
                and abs(float(seg.get('x', 99.0))) < 1e-9):
            y_C0 = y
            break
    if y_C0 is None:
        raise RuntimeError("Could not locate C-sensor y=0 dwell in segment list")

    MEAS_ZONES[0] = [y_A0 + 0.100, y_A0]
    MEAS_ZONES[1] = [y_B0 + 0.100, y_B0]
    MEAS_ZONES[2] = [y_C0 + 0.100, y_C0]

    y_B_root = y_B0 - tail_length + 0.020   # root/base of cantilever beam
    y_B_tail = y_B0 - tail_length           # end of sensor body
    y_C_root = y_C0 - tail_length + 0.020
    y_C_tail = y_C0 - tail_length

    # cam.y is strictly increasing; np.interp is safe
    x_B_root = float(np.interp(y_B_root, cam.y, cam.x))
    x_C_root = float(np.interp(y_C_root, cam.y, cam.x))
    x_B_tail = float(np.interp(y_B_tail, cam.y, cam.x))
    x_C_tail = float(np.interp(y_C_tail, cam.y, cam.x))
    print(f'\tContour X @ B-root: {60-x_B_root*1e3:.3f}mm, @ B-tail: {60-x_B_tail*1e3:.3f}mm')
    print(f'\tContour X @ C-root: {x_C_root*1e3:.3f}mm, @ C-tail: {x_C_tail*1e3:.3f}mm')

    # Resample for the physics engine (same density as drive.py)
    pts = cam.to_contour_points()
    pts = resample_contour_even_y(pts, 5000, (1.3, -1.0))
    return cam, Contour(pts)


def plot_stalk_motion(hist: dict, save_path: str) -> None:
    """Time-series plots of stalk position, velocity and acceleration."""
    matplotlib.use("Qt5Agg")
    t = hist["t"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    axes[0].plot(hist['origin_y'], hist['lag_x'] - 1*RADIUS, alpha=0.5, label='Lag-X')
    axes[0].plot(hist['origin_y'], hist['lag_y'], alpha=0.5, label='Lag-Y')
    axes[0].plot(hist['origin_y'], hist["pos_x"] - 1*RADIUS, alpha=0.5, label="x (lateral)")
    axes[0].plot(hist['origin_y'], hist["contour_x"], label="Contour")
    axes[0].set_ylim(-0.060, 0.080)
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
        for s in MEAS_ZONES:
            if s[0] is not None:
                ax.axvline(s[0], color='k', linestyle='--', linewidth=1.0, alpha=0.7, zorder=1)
                ax.axvline(s[1], color='k', linestyle='--', linewidth=1.0, alpha=0.7, zorder=1)
        ymin, ymax = ax.get_ylim()
        ax.fill_between(hist['origin_y'], ymin, ymax, where=contact, color="red", alpha=0.08, label="_contact")
        ax.xaxis.set_inverted(True)

    fig.suptitle("Stalk Motion during Contour Drive")
    plt.tight_layout()
    plt.show()

    # fig.savefig(save_path, dpi=150)
    # print(f"Saved stalk motion figure → {save_path}")


def main() -> None:
    out_dir = os.path.dirname(__file__)

    print("=" * 60)
    print("1. Building contour …")
    start_time = time.time()
    cam, contour = build_default_contour()
    print(f"   Contour extent ≈ {contour.extent:.3f} m, "
          f"{len(contour.points)} points")

    # Export CSV so drive.py can also use it
    csv_path = os.path.join(out_dir, "new_profile.csv")
    cam.export_profile(path=csv_path)

    # Contour kinematics figure
    kin_png = os.path.join(out_dir, "contour_kinematics.png")
    # cam.plot_kinematics(vy=cam.vy, save_path=kin_png)
    end_time = time.time()
    print(f'\tLoad/Build time: {end_time-start_time:.3f}s')

    print("\n2. Running headless physics simulation …")
    start_time = time.time()
    # Duration long enough to traverse the whole profile at 0.10 m/s
    duration = (contour.y_max - contour.y_min) / DRIVE_SPEED_MPS + 2.0
    hist = simulate_drive(
        contour,
        drive_speed=DRIVE_SPEED_MPS,
        lateral_x=-0.020,
        duration=duration,
        record_every=0.005,          # meters
    )
    end_time = time.time()
    print(f"\tSimulated {hist['t'][-1]:.2f} s  ({len(hist['t'])} samples)")
    print(f'\tRun time: {end_time-start_time:.3f}s')


    print("\n3. Evaluation metrics")
    # Stalk motion figure
    motion_png = os.path.join(out_dir, "stalk_motion.png")
    plot_stalk_motion(hist, motion_png)

    # Simple scalar metrics (ready to become a cost function)
    peak_ax = float(np.max(np.abs(hist["acc_x"])))
    peak_ay = float(np.max(np.abs(hist["acc_y"])))
    peak_vx = float(np.max(np.abs(hist["vel_x"])))
    contact_fraction = float(np.mean(hist["contact"]))

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
