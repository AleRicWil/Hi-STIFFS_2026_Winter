#!/usr/bin/env python3
"""
optimize.py - Simple evaluation / optimization driver.

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
import copy

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from contour import LinearCamContour, Contour, resample_contour_even_y
from physics import simulate_drive
from physics import (RADIUS, EI, h, K_SPRING, MASS, B_DAMPER, MU_KINETIC, MU_STATIC)

DRIVE_SPEED_MPS = 0.1   # speed of stalk-origin in -Y
MEAS_ZONES = [[None, None], [0.100, 0.000], [None, None]] # array of y-values marking the start (bigger y) and end (smaller y) of each sensor's measurement zone (exposed cantilever)
ICB_LENGTH = 0.100 + 0.015
tail_length = 0.077362   # distance behind y=0 that contour must clear with >= 2.4mm

# ===== Design Variables for optimization - defaults =====
# Tip       (pre->A):         length, delta_x,            tp, leading_dwell.
# Leading     (A->B):         length,      tp, leading_dwell,          None.
# Trailing    (B->C): trailing_dwell,  length,            tp, leading_dwell.
# Tail     (C->post): trailing_dwell,  length,       delta_x,            tp.
DVS = [ # Tip (pre->A)
        [0.240, -0.060, 1.0, 0.001],  
        # Leading (A->B)
        [0.500, 1.0, 0.001],   
        # Trailing (B->C) 
        [0.005, 0.500, 1.0, 0.001],  
        # Tail (C->post)
        [0.053, 0.12, -0.060, 1.0] ]


SEGMENTS = [
        # ---- LEAD (+y) ----
        {'dir': 'lead', 'type': 'dwell', 'length': ICB_LENGTH, 'x': 0.06},     # start of this segment is y=0 of B-sensor, goes forward by 'length'
        {'dir': 'lead', 'type': 'dwell', 'length': DVS[1][2], 'x': 0.06},
        {'dir': 'lead', 'type': 'rise',  'length': DVS[1][0], 'delta_x': -0.06, 'tp': DVS[1][1]},
        {'dir': 'lead', 'type': 'dwell', 'length': ICB_LENGTH, 'x': 0.00},
        {'dir': 'lead', 'type': 'dwell', 'length': DVS[0][3], 'x': -0.000},
        {'dir': 'lead', 'type': 'rise',  'length': DVS[0][0], 'delta_x': DVS[0][1], 'tp': DVS[0][2]},
        # ---- TRAIL (−y) ----
        {'dir': 'trail', 'type': 'dwell', 'length': DVS[2][0], 'x': 0.06},          # accomodate tail of B-sensor, start of this segment is y=0 of B-sensor, goes backward by 'length'
        {'dir': 'trail', 'type': 'rise',  'length': DVS[2][1], 'delta_x': -0.06, 'tp': DVS[2][2]},
        {'dir': 'trail', 'type': 'dwell', 'length': DVS[2][3], 'x': 0.00},
        {'dir': 'trail', 'type': 'dwell', 'length': ICB_LENGTH, 'x': 0.00},    # end of this segment is y=0 of C-sensor
        {'dir': 'trail', 'type': 'dwell', 'length': DVS[3][0], 'x': 0.00},          # accomodate tail of C-sensor
        {'dir': 'trail', 'type': 'rise',  'length': DVS[3][1], 'delta_x': DVS[3][2], 'tp': DVS[3][3]},    # tp - bigger is gentler leading
    ]

def build_default_contour() -> tuple[LinearCamContour, Contour]:
    """Reproduce the original geomtry_opt example profile."""
    cam = LinearCamContour(vy=0.447)
    segments = SEGMENTS
    cam.build_profile(segments, curve_func=cam.poly_345_biased)
    cam.trim_and_ramp_to(x_cut=-0.020, x_end=-0.060)

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

    MEAS_ZONES[0] = [y_A0 + 0.100, y_A0, y_A0 - tail_length]
    MEAS_ZONES[1] = [y_B0 + 0.100, y_B0, y_B0 - tail_length]
    MEAS_ZONES[2] = [y_C0 + 0.100, y_C0, y_C0 - tail_length]

    y_B_root = y_B0 - tail_length + 0.020   # root/base of cantilever beam
    y_B_tail = y_B0 - tail_length           # end of sensor body
    y_C_root = y_C0 - tail_length + 0.020
    y_C_tail = y_C0 - tail_length

    # cam.y is strictly increasing; np.interp is safe
    x_B_root = float(np.interp(y_B_root, cam.y, cam.x))
    x_C_root = float(np.interp(y_C_root, cam.y, cam.x))
    x_B_tail = float(np.interp(y_B_tail, cam.y, cam.x))
    x_C_tail = float(np.interp(y_C_tail, cam.y, cam.x))
    print(f'\tContour X @ B-root: {x_B_root*1e3-60:.3f}mm, @ B-tail: {x_B_tail*1e3-60:.3f}mm')
    print(f'\tContour X @ C-root: {x_C_root*1e3:.3f}mm, @ C-tail: {x_C_tail*1e3:.3f}mm')

    # Resample for the physics engine (same density as drive.py)
    pts = cam.to_contour_points()
    pts = resample_contour_even_y(pts, 5000, (1.5, -1.5))
    return cam, Contour(pts)

def build_contour(design_vars: list) -> tuple[LinearCamContour, Contour, list]:
    """Reproduce the defined profile."""
    cam = LinearCamContour(vy=0.447)
    this_segments = SEGMENTS
    this_meas_zones = copy.deepcopy(MEAS_ZONES)

    # Leading (A->B): design_vars[1] = [length, tp, leading_dwell]
    this_segments[1]['length'] = design_vars[1][2]
    this_segments[2]['length'] = design_vars[1][0]
    this_segments[2]['tp']     = design_vars[1][1]

    # Tip (pre->A): design_vars[0] = [length, delta_x, tp, leading_dwell]
    this_segments[4]['length']   = design_vars[0][3]
    this_segments[5]['length']   = design_vars[0][0]
    this_segments[5]['delta_x']  = design_vars[0][1]
    this_segments[5]['tp']       = design_vars[0][2]

    # Trailing (B->C): design_vars[2] = [trailing_dwell, length, tp, leading_dwell]
    this_segments[6]['length'] = design_vars[2][0]
    this_segments[7]['length'] = design_vars[2][1]
    this_segments[7]['tp']     = design_vars[2][2]
    this_segments[8]['length'] = design_vars[2][3]

    # Tail (C->post): design_vars[3] = [trailing_dwell, length, delta_x, tp]
    this_segments[10]['length']  = design_vars[3][0]
    this_segments[11]['length']  = design_vars[3][1]
    this_segments[11]['delta_x'] = design_vars[3][2]
    this_segments[11]['tp']      = design_vars[3][3]

    cam.build_profile(this_segments, curve_func=cam.poly_345_biased)
    cam.trim_and_ramp_to(x_cut=-0.020, x_end=-0.060)

    # ------------------------------------------------------------------
    # Locate the sensor origins in global y, then sample the contour
    # a distance tail_length behind each of them.
    # ------------------------------------------------------------------
    y_B0 = 0.0          # by construction of build_profile

    # A-sensor y=0 is the *end* of the first lead dwell (length 0.100+0.015).
    # Walk the lead segments exactly as build_profile does.
    y = 0.0
    y_A0 = None
    for seg in this_segments:
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
    for seg in this_segments:
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

    this_meas_zones[0] = [y_A0 + 0.100, y_A0, y_A0 - tail_length]
    this_meas_zones[1] = [y_B0 + 0.100, y_B0, y_B0 - tail_length]
    this_meas_zones[2] = [y_C0 + 0.100, y_C0, y_C0 - tail_length]

    y_B_root = y_B0 - tail_length + 0.020   # root/base of cantilever beam
    y_B_tail = y_B0 - tail_length           # end of sensor body
    y_C_root = y_C0 - tail_length + 0.020
    y_C_tail = y_C0 - tail_length

    # cam.y is strictly increasing; np.interp is safe
    x_B_root = float(np.interp(y_B_root, cam.y, cam.x))
    x_C_root = float(np.interp(y_C_root, cam.y, cam.x))
    x_B_tail = float(np.interp(y_B_tail, cam.y, cam.x))
    x_C_tail = float(np.interp(y_C_tail, cam.y, cam.x))
    print(f'\tContour X @ B-root: {x_B_root*1e3-60:.3f}mm, @ B-tail: {x_B_tail*1e3-60:.3f}mm')
    print(f'\tContour X @ C-root: {x_C_root*1e3:.3f}mm, @ C-tail: {x_C_tail*1e3:.3f}mm')

    # Resample for the physics engine (same density as drive.py)
    pts = cam.to_contour_points()
    pts = resample_contour_even_y(pts, 5000, (1.5, -1.5))
    return cam, Contour(pts), this_meas_zones

def characterize_stalk_motion(hist: dict, meas_zones: list, zone: str, y_range: list):
    '''
    For a specified zone, compute charateristics of stalk motion

    returns:
    zone length
    rms lag x and rms lag y
    peak lag x and peak lag y
    rms ax and rms ay
    peak ax and peak ay
    '''
    # extract y_lims for zone from meas_zones list
    if zone == 'Tip':
        y_max = y_range[0]
        y_min = meas_zones[0][1]
    elif zone == 'Leading':
        y_max = meas_zones[0][1]
        y_min = meas_zones[1][1]
    elif zone == 'Trailing':
        y_max = meas_zones[1][1]
        y_min = meas_zones[2][1]
    elif zone == 'Tail':
        y_max = meas_zones[2][1]
        y_min = y_range[1]

    zone_length = y_max - y_min
    print(zone_length, y_max, y_min)

    this_hist = copy.deepcopy(hist)

    lag_x = this_hist['lag_x']
    lag_y = this_hist['lag_y']
    accel_x = this_hist['acc_x']
    accel_y = this_hist['acc_y']

    rms_lag_x = np.linalg.norm(lag_x) / np.sqrt(lag_x.size)
    rms_lag_y = np.linalg.norm(lag_y) / np.sqrt(lag_y.size)
    rms_accel_x = np.linalg.norm(accel_x) / np.sqrt(accel_x.size)
    rms_accel_y = np.linalg.norm(accel_y) / np.sqrt(accel_y.size)

    peak_lag_x = np.max(lag_x)
    peak_lag_y = np.max(lag_y)
    peak_accel_x = np.max(accel_x)
    peak_accel_y = np.max(accel_y)

    return zone_length, rms_lag_x, peak_lag_x, rms_lag_y, peak_lag_y, rms_accel_x, peak_accel_x, rms_accel_y, peak_accel_y

    

def plot_stalk_motion(hist: dict, save_path: str = None,
                      meas_zones: list = None, show: bool = True,
                      close: bool = True, title_suffix: str = "") -> plt.Figure:
    """Time-series plots of stalk position, velocity and acceleration.
    Returns the Figure so callers can keep multiple windows open simultaneously.
    """
    if meas_zones is None:
        meas_zones = MEAS_ZONES

    # Ensure interactive backend once (safe to call repeatedly)
    matplotlib.use("Qt5Agg")
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    axes[0].plot(hist['origin_y'], hist['lag_x'] - 1*RADIUS, alpha=0.5, label='Lag-X')
    axes[0].plot(hist['origin_y'], hist['lag_y'], alpha=0.5, label='Lag-Y')
    axes[0].plot(hist['origin_y'], hist["pos_x"] - 1*RADIUS, alpha=0.5, label="x (lateral)")
    axes[0].plot(hist['origin_y'], hist["contour_x"], label="Contour")
    axes[0].set_ylim(-0.060, 0.070)
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

    contact = hist["contact"]
    for ax in axes:
        for s in meas_zones:
            if s[0] is not None:
                ax.axvline(s[0], color='k', linestyle='--', linewidth=1.0, alpha=0.7, zorder=1)
                ax.axvline(s[1], color='k', linestyle='--', linewidth=1.0, alpha=0.7, zorder=1)
                ax.axvline(s[2], color='k', linestyle='--', linewidth=1.0, alpha=0.7, zorder=1)
        ymin, ymax = ax.get_ylim()
        ax.fill_between(hist['origin_y'], ymin, ymax, where=contact,
                        color="red", alpha=0.08, label="_contact")
        ax.xaxis.set_inverted(True)

    fig.suptitle(f"Stalk Motion during Contour Drive{title_suffix}")
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved stalk motion figure → {save_path}")

    if show:
        plt.show()
    if close:
        plt.close(fig)

    return fig

def main() -> None:
    out_dir = os.path.dirname(__file__)

    print("=" * 60)
    print("1. Building contour …")
    start_time = time.time()
    cam, contour = build_default_contour()
     # --- overall length metric (Y-span between the two x = -0.020 intersections)
    length, y_lead, y_trail = cam.overall_length()
    print(f"   Overall length @ x=-0.080 m : {length:.4f} m "
          f"(lead y={y_lead:.4f} → trail y={y_trail:.4f})")
    print(f"   Contour points              : {len(contour.points)}")

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
    # print(f"  {kin_png}")
    # print(f"  {motion_png}")
    # print(f"  Contour CSV : {csv_path}")

def run_sweep(i: int, j: int, range: list[float], steps: int = 3) -> None:
    '''
    Build probe contour and simulate stalk interaction across sweep of design variable at i,j in DVS.
    Opens every kinematics + motion figure interactively at the same time.
    '''
    # Independent copy so we never touch the module-level DVS
    this_dvs = [row[:] for row in DVS]

    test_vals = np.linspace(range[0], range[1], steps)

    cam_list = []
    contour_list = []
    meas_zones_list = []
    hist_list = []
    char_list = []          # (val, overall_length, y_lead, y_trail)

    out_dir = os.path.dirname(__file__)
    open_figs = []          # keep references so windows stay alive

    for idx, val in enumerate(test_vals):
        this_dvs[i][j] = float(val)

        cam, contour, meas_zones = build_contour(this_dvs)
        cam_list.append(cam)
        contour_list.append(contour)
        meas_zones_list.append(meas_zones)

        length, y_lead, y_trail = cam.overall_length()
        char_list.append((val, length, y_lead, y_trail))

        # Unique CSV for this probe
        csv_path = os.path.join(out_dir, f"new_profile_{idx}.csv")
        cam.export_profile(path=csv_path)

        # Physics (identical path to main / interactive)
        duration = (contour.y_max - contour.y_min) / DRIVE_SPEED_MPS + 2.0
        hist = simulate_drive(
            contour,
            drive_speed=DRIVE_SPEED_MPS,
            lateral_x=-0.020,
            duration=duration,
            record_every=0.005,
        )
        hist_list.append(hist)


        characterize_stalk_motion(hist, meas_zones, 'Trailing', [y_lead, y_trail])

    # ------------------------------------------------------------------
    # Summary table (fast design decision view)
    # ------------------------------------------------------------------
    print("\n=== Sweep summary ===")
    print(f"{'idx':>3}  {'value':>10}  {'length':>8}  {'|ax|_peak':>10}  {'|ay|_peak':>10}  {'contact':>8}")
    for idx, (val, length, _, _) in enumerate(char_list):
        hist = hist_list[idx]
        peak_ax = float(np.max(np.abs(hist["acc_x"])))
        peak_ay = float(np.max(np.abs(hist["acc_y"])))
        contact_frac = float(np.mean(hist["contact"]))
        print(f"{idx:3d}  {val:10.4f}  {length:8.4f}  {peak_ax:10.2f}  {peak_ay:10.2f}  {contact_frac:8.3f}")

        # # Kinematics – save + keep window open
        # kin_png = os.path.join(out_dir, f"contour_kinematics_{idx}.png")
        # fig_kin = cam_list[idx].plot_kinematics(
        #     vy=cam_list[idx].vy,
        #     save_path=kin_png,
        #     show=False,
        #     close=False,
        # )
        # if fig_kin is not None:
        #     open_figs.append(fig_kin)

        # Motion – save + keep window open, correct per-probe zones
        motion_png = os.path.join(out_dir, f"stalk_motion_{idx}.png")
        fig_mot = plot_stalk_motion(
            hist_list[idx],
            save_path=motion_png,
            meas_zones=meas_zones_list[idx],
            show=False,
            close=False,
            title_suffix=f"  [var[{i}][{j}] = {val:.4f}]",
        )
        open_figs.append(fig_mot)

    print("\nAll CSVs and PNG archives written to", out_dir)
    print(f"Opening {len(open_figs)} interactive figures simultaneously …")

    # Single blocking show() brings every window up at once
    matplotlib.use("Qt5Agg")
    plt.show()




if __name__ == "__main__":
    run_sweep(0, 0, [0.1, 0.5], 3)
