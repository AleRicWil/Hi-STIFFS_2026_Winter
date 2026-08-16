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

DRIVE_SPEED_MPS = 0.224   # speed of stalk-origin in -Y
MEAS_ZONES = [[None, None], [0.100, 0.000], [None, None]] # array of y-values marking the start (bigger y) and end (smaller y) of each sensor's measurement zone (exposed cantilever)
ICB_LENGTH = 0.100 + 0.015
tail_length = 0.077362   # distance behind y=0 that contour must clear with >= 2.4mm

# ===== Design Variables for optimization - defaults =====
# Tip       (pre->A):         length, delta_x,            tp, leading_dwell.
# Leading     (A->B):         length,      tp, leading_dwell,          None.
# Trailing    (B->C): trailing_dwell,  length,            tp, leading_dwell.
# Tail     (C->post): trailing_dwell,  length,       delta_x,            tp.
DVS = [ # Tip (pre->A)
        [0.433, -0.075, 1.4, 0.001],  
        # Leading (A->B)
        [0.57, 1.16, 0.001],   
        # Trailing (B->C) 
        [0.001, 0.560, 1.19, 0.001],  
        # Tail (C->post)
        [0.001, 0.220, -0.030, 1.9] ]


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
    pts = resample_contour_even_y(pts, 2000, (1.2, -1.0))
    return cam, Contour(pts)

def build_contour(design_vars: list, quiet: bool = False) -> tuple[LinearCamContour, Contour, list]:
    """Reproduce the defined profile from a full design-variable table.

    Parameters
    ----------
    design_vars : list
        Same nested structure as the module-level DVS.
    quiet : bool
        Suppress diagnostic prints (useful for dense grid sweeps).
    """
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
    print(f'Sensor Zeros: {y_A0:.6f}, {y_B0:.6f}, {y_C0:.6f}')

    y_B_root = y_B0 - tail_length + 0.020   # root/base of cantilever beam
    y_B_tail = y_B0 - tail_length           # end of sensor body
    y_C_root = y_C0 - tail_length + 0.020
    y_C_tail = y_C0 - tail_length

    # cam.y is strictly increasing; np.interp is safe
    x_B_root = float(np.interp(y_B_root, cam.y, cam.x))
    x_C_root = float(np.interp(y_C_root, cam.y, cam.x))
    x_B_tail = float(np.interp(y_B_tail, cam.y, cam.x))
    x_C_tail = float(np.interp(y_C_tail, cam.y, cam.x))
    if not quiet:
        
        print(f'\tContour X @ B-root: {x_B_root*1e3-60:.3f}mm, @ B-tail: {x_B_tail*1e3-60:.3f}mm')
        print(f'\tContour X @ C-root: {x_C_root*1e3:.3f}mm, @ C-tail: {x_C_tail*1e3:.3f}mm')

    # Resample for the physics engine (same density as drive.py)
    pts = cam.to_contour_points()
    pts = resample_contour_even_y(pts, 5000, (1.15, -1.00))
    return cam, Contour(pts), this_meas_zones

# Human-readable names for the design variables of each zone (order matches DVS rows)
ZONE_DV_NAMES = {
    "Tip":      ["length", "delta_x", "tp", "leading_dwell"],
    "Leading":  ["length", "tp", "leading_dwell"],
    "Trailing": ["trailing_dwell", "length", "tp", "leading_dwell"],
    "Tail":     ["trailing_dwell", "length", "delta_x", "tp"],
}
ZONE_TO_DVS_IDX = {"Tip": 0, "Leading": 1, "Trailing": 2, "Tail": 3}

# Ordered list of scalar characteristics produced by characterize_stalk_motion
CHAR_NAMES = [
    "zone_length",
    "rms_lag_x", "peak_lag_x",
    "rms_lag_y", "peak_lag_y",
    "rms_accel_x", "peak_accel_x",
    "rms_accel_y", "peak_accel_y",
    "L2.5_rms_ay",   # zone_length * rms(ay) – length-effectiveness of y-dynamics
]


def characterize_stalk_motion(hist: dict, meas_zones: list, zone: str, y_range: list,
                              verbose: bool = False):
    """
    Zone-local characterization of stalk motion (and geometric zone length).

    Metrics are computed only on the samples whose origin_y lies inside the
    zone.  This is essential for design decisions that target a specific
    segment of the profile.

    Parameters
    ----------
    hist : dict
        History returned by simulate_drive.
    meas_zones : list
        Per-sensor [y_start, y_zero, y_tail] as produced by build_contour.
    zone : str
        One of 'Tip', 'Leading', 'Trailing', 'Tail'.
    y_range : list
        [y_lead, y_trail] from cam.overall_length(); used only by Tip / Tail.
    verbose : bool
        If True, print the geometric zone bounds.

    Returns
    -------
    tuple of 10 floats
        (zone_length,
         rms_lag_x, peak_lag_x,
         rms_lag_y, peak_lag_y,
         rms_accel_x, peak_accel_x,
         rms_accel_y, peak_accel_y,
         L_rms_ay)          # zone_length * rms(ay)
    """
    # Geometric y-limits of the requested zone
    if zone == "Tip":
        y_max = y_range[0]
        y_min = meas_zones[0][1]
    elif zone == "Leading":
        y_max = meas_zones[0][1]
        y_min = meas_zones[1][1]
    elif zone == "Trailing":
        y_max = meas_zones[1][1]
        y_min = meas_zones[2][1]
    elif zone == "Tail":
        y_max = meas_zones[2][1]
        y_min = y_range[1]
    else:
        raise ValueError(f"Unknown zone '{zone}'. Expected Tip|Leading|Trailing|Tail.")

    zone_length = float(y_max - y_min)
    if verbose:
        print(f"  zone={zone}  length={zone_length:.4f}  y=[{y_max:.4f} → {y_min:.4f}]")

    # Mask history to samples that fall inside the zone (origin_y decreases with time)
    oy = np.asarray(hist["origin_y"])
    mask = (oy <= y_max) & (oy >= y_min)
    n = int(np.count_nonzero(mask))
    if n < 2:
        # Degenerate / empty zone – return NaNs so downstream surfaces show the hole
        return (zone_length,) + (np.nan,) * 9

    lag_x   = np.asarray(hist["lag_x"])[mask]
    lag_y   = np.asarray(hist["lag_y"])[mask]
    accel_x = np.asarray(hist["acc_x"])[mask]
    accel_y = np.asarray(hist["acc_y"])[mask]

    # RMS (Euclidean / sqrt(N)) and peak (max absolute value for engineering relevance)
    rms_lag_x   = float(np.linalg.norm(lag_x)   / np.sqrt(n))
    rms_lag_y   = float(np.linalg.norm(lag_y)   / np.sqrt(n))
    rms_accel_x = float(np.linalg.norm(accel_x) / np.sqrt(n))
    rms_accel_y = float(np.linalg.norm(accel_y) / np.sqrt(n))

    peak_lag_x   = float(np.max(np.abs(lag_x)))
    peak_lag_y   = float(np.max(np.abs(lag_y)))
    peak_accel_x = float(np.max(np.abs(accel_x)))
    peak_accel_y = float(np.max(np.abs(accel_y)))

    # Length-effectiveness of y-dynamics: lower is better (more motion reduction per unit length)
    L_rms_ay = zone_length**2.5 * rms_accel_y

    print(f"  rms_ay={rms_accel_y:.3f}, L*rms_ay={L_rms_ay:.4f}")

    return (zone_length,
            rms_lag_x, peak_lag_x,
            rms_lag_y, peak_lag_y,
            rms_accel_x, peak_accel_x,
            rms_accel_y, peak_accel_y,
            L_rms_ay)

    

def plot_stalk_motion(hist: dict, save_path: str = None,
                      meas_zones: list = None, show: bool = True,
                      close: bool = True, title_suffix: str = "",
                      zone: str | None = None) -> plt.Figure:
    """Time-series plots of stalk position, velocity and acceleration.

    Designed for both single evaluations and grid-sweep points.  The title
    always includes the zone (when provided) and any extra context passed via
    ``title_suffix`` (typically the design-variable values being simulated).

    Parameters
    ----------
    hist : dict
        History returned by ``simulate_drive``.
    save_path : str, optional
        If given, the figure is written to this path (dpi=150).
    meas_zones : list, optional
        Per-sensor measurement windows; defaults to module-level MEAS_ZONES.
    show : bool
        If True, switch to an interactive backend and call ``plt.show()``.
        Use False for pure save-only / batch runs.
    close : bool
        If True, close the figure after saving / showing (frees memory).
        Set False when you want to keep multiple windows alive for a later
        collective ``plt.show()``.
    title_suffix : str
        Extra text appended to the title (e.g. variable values).
    zone : str, optional
        Zone name ('Tip', 'Leading', …).  Included in the title when given.

    Returns
    -------
    matplotlib.figure.Figure
    """
    if meas_zones is None:
        meas_zones = MEAS_ZONES

    # Only switch to interactive backend when the caller actually wants a window.
    # Leaving the module-level Agg backend alone keeps headless / CI runs clean.
    if show:
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

    # Rich title: zone + any caller-supplied context (DV values, etc.)
    title = f"Stalk Motion - {DRIVE_SPEED_MPS*2.23694:.2f}mph"
    if zone:
        title += f" - {zone} zone"
    if title_suffix:
        title += title_suffix
    fig.suptitle(title)
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
    cam, contour, meas_zones = build_contour(DVS, quiet=False)
     # --- overall length metric (Y-span between the two x = -0.020 intersections)
    length, y_lead, y_trail = cam.overall_length()
    print(f"   Overall length @ x=-0.080 m : {length:.4f} m "
          f"(lead y={y_lead:.4f} → trail y={y_trail:.4f})")
    print(f"   Contour points              : {len(contour.points)}")

    # Export CSV so drive.py can also use it
    csv_path = os.path.join(out_dir, "new_profile.csv")
    csv_path_2 = os.path.join(out_dir, "contour_export.csv")
    cam.export_profile(path=csv_path)
    contour.export_profile(path=csv_path_2)

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
    chars = characterize_stalk_motion(hist, meas_zones, 'Tip', [y_lead, y_trail], verbose=True)
    # Stalk motion figure
    motion_png = os.path.join(out_dir, "stalk_motion.png")
    plot_stalk_motion(hist, motion_png, meas_zones=meas_zones)

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


def run_grid_sweep(
    zone: str,
    idx_a: int,
    idx_b: int,
    range_a: list[float],
    range_b: list[float],
    steps_a: int = 5,
    steps_b: int = 5,
    out_dir: str | None = None,
    show_motion_plots: bool = False
) -> dict:
    """
    2-variable grid sweep of design variables belonging to one zone.

    For every (val_a, val_b) on the Cartesian product of the two linspaces:
      1. Mutate a fresh copy of DVS at the zone's row / (idx_a, idx_b).
      2. Build the corresponding LinearCamContour + Contour.
      3. Run the headless physics simulation (identical settings to main / drive).
      4. Characterize both geometry (zone length) and stalk motion inside the zone.

    Results are written to a plain-text CSV (easy to reload later).  Plotting is
    deliberately left to plot_grid_results so expensive sims can be re-visualized
    without re-running.

    Parameters
    ----------
    zone : str
        'Tip' | 'Leading' | 'Trailing' | 'Tail'
    idx_a, idx_b : int
        Column indices into that zone's DVS list (see ZONE_DV_NAMES).
    range_a, range_b : list[float]
        [min, max] for each variable.
    steps_a, steps_b : int
        Number of samples along each axis (inclusive endpoints).
    out_dir : str, optional
        Directory for the CSV.  Defaults to the package directory.
    show_motion_plots : bool
        If True, keep the figures open and call ``plt.show()`` after the whole
        grid finishes (useful only for small grids – e.g. 3×3).  When both
        flags are False the expensive per-point plotting is skipped entirely.

    Returns
    -------
    dict
        {
          'zone': str,
          'idx_a': int, 'idx_b': int,
          'label_a': str, 'label_b': str,
          'zone_row': int,
          'vals_a': 1-D array,
          'vals_b': 1-D array,
          'grid_a': meshgrid X,
          'grid_b': meshgrid Y,
          'metrics': {name: 2-D array of shape (steps_a, steps_b)},
          'csv_path': str,
        }
    """
    if zone not in ZONE_TO_DVS_IDX:
        raise ValueError(f"Unknown zone '{zone}'. Expected one of {list(ZONE_TO_DVS_IDX)}")

    zone_row = ZONE_TO_DVS_IDX[zone]
    names = ZONE_DV_NAMES[zone]
    if not (0 <= idx_a < len(names) and 0 <= idx_b < len(names)):
        raise IndexError(
            f"Zone '{zone}' has only {len(names)} design variables "
            f"{names}; requested indices ({idx_a}, {idx_b})."
        )
    if idx_a == idx_b:
        raise ValueError("idx_a and idx_b must be distinct.")

    label_a = names[idx_a]
    label_b = names[idx_b]

    if out_dir is None:
        out_dir = os.path.dirname(__file__)
    os.makedirs(out_dir, exist_ok=True)

    vals_a = np.linspace(range_a[0], range_a[1], steps_a)
    vals_b = np.linspace(range_b[0], range_b[1], steps_b)
    grid_a, grid_b = np.meshgrid(vals_a, vals_b, indexing="ij")  # (steps_a, steps_b)

    metrics: dict[str, np.ndarray] = {
        name: np.full((steps_a, steps_b), np.nan, dtype=float) for name in CHAR_NAMES
    }
    table_rows: list[list[float]] = []
    open_figs: list[plt.Figure] = []

    n_total = steps_a * steps_b
    print("=" * 70)
    print(f"GRID SWEEP  zone={zone}  vars=({label_a}[{idx_a}], {label_b}[{idx_b}])")
    print(f"  {label_a}: {range_a[0]:.4g} → {range_a[1]:.4g}  ({steps_a} steps)")
    print(f"  {label_b}: {range_b[0]:.4g} → {range_b[1]:.4g}  ({steps_b} steps)")
    print(f"  Total evaluations: {n_total}")
    print("=" * 70)

    t0_all = time.time()
    for ia in range(steps_a):
        for ib in range(steps_b):
            k = ia * steps_b + ib + 1
            va = float(vals_a[ia])
            vb = float(vals_b[ib])

            # Fresh independent copy of the design-variable table
            this_dvs = [row[:] for row in DVS]
            this_dvs[zone_row][idx_a] = va
            this_dvs[zone_row][idx_b] = vb

            t0 = time.time()
            try:
                cam, contour, meas_zones = build_contour(this_dvs, quiet=True)
                length_overall, y_lead, y_trail = cam.overall_length()

                duration = (contour.y_max - contour.y_min) / DRIVE_SPEED_MPS + 2.0
                hist = simulate_drive(
                    contour,
                    drive_speed=DRIVE_SPEED_MPS,
                    lateral_x=-0.020,
                    duration=duration,
                    record_every=0.005,
                )

                chars = characterize_stalk_motion(
                    hist, meas_zones, zone, [y_lead, y_trail], verbose=False
                )
            except (ValueError, RuntimeError, Exception) as exc:
                # Extreme DV combinations can make build_profile / trim_and_ramp_to /
                # overall_length / sensor-location logic raise.  Leave metrics as NaN
                # and continue so the rest of the grid still runs.
                dt = time.time() - t0
                print(
                    f"  [{k:3d}/{n_total}]  {label_a}={va:8.4f}  {label_b}={vb:8.4f}  "
                    f"SKIPPED ({type(exc).__name__}: {exc})  ({dt:.2f}s)"
                )
                table_rows.append([va, vb] + [float("nan")] * len(CHAR_NAMES))
                continue

            dt = time.time() - t0

            for name, val in zip(CHAR_NAMES, chars):
                metrics[name][ia, ib] = val

            table_rows.append([va, vb] + list(chars))

            print(
                f"  [{k:3d}/{n_total}]  {label_a}={va:8.4f}  {label_b}={vb:8.4f}  "
                f"len={chars[0]:.4f}  rms_ay={chars[7]:.3f}  "
                f"L*rms_ay={chars[9]:.4f}  ({dt:.2f}s)"
            )

            # ------------------------------------------------------------------
            # Optional per-point stalk-motion figure
            # ------------------------------------------------------------------
            suffix = f"  [{label_a}={va:.4g}, {label_b}={vb:.4g}]"
            motion_png = None
            motion_png = os.path.join(
                out_dir,
                f"stalk_motion_{zone}_{ia:02d}_{ib:02d}_{label_a}_{label_b}.png",
            )
            fig = plot_stalk_motion(
                hist,
                save_path=motion_png,
                meas_zones=meas_zones,
                show=False,                 # never block inside the loop
                close=not show_motion_plots,
                title_suffix=suffix,
                zone=zone,
            )
            if show_motion_plots:
                open_figs.append(fig)



    elapsed = time.time() - t0_all
    print(f"\nGrid complete in {elapsed:.1f}s  ({elapsed / n_total:.2f}s per eval)")

    # ------------------------------------------------------------------
    # Text-readable results file (CSV with header – trivial to reload)
    # ------------------------------------------------------------------
    csv_name = f"grid_sweep_{zone}_{label_a}_vs_{label_b}.csv"
    csv_path = os.path.join(out_dir, csv_name)
    header = f"{label_a},{label_b}," + ",".join(CHAR_NAMES)
    with open(csv_path, "w", encoding="utf-8") as fh:
        fh.write("# zone=" + zone + "\n")
        fh.write(f"# idx_a={idx_a} ({label_a}), idx_b={idx_b} ({label_b})\n")
        fh.write(f"# range_a={list(range_a)}, steps_a={steps_a}\n")
        fh.write(f"# range_b={list(range_b)}, steps_b={steps_b}\n")
        fh.write(header + "\n")
        for row in table_rows:
            fh.write(",".join(f"{v:.8g}" for v in row) + "\n")
    print(f"Results saved → {csv_path}")

    return {
        "zone": zone,
        "idx_a": idx_a,
        "idx_b": idx_b,
        "label_a": label_a,
        "label_b": label_b,
        "zone_row": zone_row,
        "vals_a": vals_a,
        "vals_b": vals_b,
        "grid_a": grid_a,
        "grid_b": grid_b,
        "metrics": metrics,
        "csv_path": csv_path,
    }


def plot_grid_results(
    data: dict | None = None,
    csv_path: str | None = None,
    show_plots: bool = False,
    out_dir: str | None = None,
    metrics_to_plot: list[str] | None = None,
) -> list[str]:
    """
    Produce the series of 3-D surface plots from a completed grid sweep.

    Accepts either the dict returned by run_grid_sweep or a path to a previously
    written CSV.  When both are given, the in-memory dict takes precedence.

    Parameters
    ----------
    data : dict, optional
        Return value of run_grid_sweep.
    csv_path : str, optional
        Path to a grid_sweep_*.csv file.  Used when data is None.
    show_plots : bool
        If True, switch to an interactive backend and call plt.show().
    out_dir : str, optional
        Directory for the PNG files.  Defaults to the directory of the CSV
        (or the package directory when only an in-memory dict is supplied).
    metrics_to_plot : list[str], optional
        Subset of CHAR_NAMES to plot.  Defaults to all characteristics.

    Returns
    -------
    list[str]
        Paths of the written PNG files.
    """
    if data is None and csv_path is None:
        raise ValueError("Provide either data= (from run_grid_sweep) or csv_path=.")

    file_dir = os.path.dirname(__file__)
    csv_path = os.path.join(file_dir, csv_path)

    # ------------------------------------------------------------------
    # Resolve data source
    # ------------------------------------------------------------------
    if data is not None:
        zone      = data["zone"]
        idx_a     = data["idx_a"]
        idx_b     = data["idx_b"]
        label_a   = data["label_a"]
        label_b   = data["label_b"]
        zone_row  = data["zone_row"]
        vals_a    = data["vals_a"]
        vals_b    = data["vals_b"]
        grid_a    = data["grid_a"]
        grid_b    = data["grid_b"]
        metrics   = data["metrics"]
        if out_dir is None:
            out_dir = os.path.dirname(data.get("csv_path", os.path.dirname(__file__)))
    else:
        # Load from CSV (comment lines carry the metadata)
        if out_dir is None:
            out_dir = os.path.dirname(os.path.abspath(csv_path))

        meta: dict[str, str] = {}
        with open(csv_path, "r", encoding="utf-8") as fh:
            for line in fh:
                if not line.startswith("#"):
                    break
                line = line[1:].strip()
                if "=" in line:
                    key, _, val = line.partition("=")
                    meta[key.strip()] = val.strip()

        zone = meta.get("zone", "Unknown")
        # "idx_a=0 (trailing_dwell), idx_b=1 (length)"
        idx_line = meta.get("idx_a", "0 ()")
        # crude but robust parse of the two parenthetical labels
        import re
        m = re.search(
            r"idx_a=(\d+)\s*\(([^)]+)\).*idx_b=(\d+)\s*\(([^)]+)\)",
            meta.get("idx_a", "") + ", " + meta.get("idx_b", ""),
        )
        if m:
            idx_a, label_a, idx_b, label_b = int(m.group(1)), m.group(2), int(m.group(3)), m.group(4)
        else:
            # fallback: read header row for labels
            idx_a, idx_b = 0, 1
            label_a, label_b = "var_a", "var_b"

        zone_row = ZONE_TO_DVS_IDX.get(zone, -1)

        # Numeric table (skip comment lines)
        raw = np.loadtxt(csv_path, delimiter=",", comments="#", skiprows=5)
        if raw.ndim == 1:
            raw = raw.reshape(1, -1)

        # Reconstruct unique sorted axes and the 2-D metric grids
        vals_a = np.unique(raw[:, 0])
        vals_b = np.unique(raw[:, 1])
        steps_a, steps_b = len(vals_a), len(vals_b)
        grid_a, grid_b = np.meshgrid(vals_a, vals_b, indexing="ij")

        metrics = {name: np.full((steps_a, steps_b), np.nan) for name in CHAR_NAMES}
        # Map each row back onto the grid (order is row-major: ia outer, ib inner)
        for row in raw:
            ia = int(np.argmin(np.abs(vals_a - row[0])))
            ib = int(np.argmin(np.abs(vals_b - row[1])))
            for j, name in enumerate(CHAR_NAMES):
                metrics[name][ia, ib] = row[2 + j]

    os.makedirs(out_dir, exist_ok=True)

    if metrics_to_plot is None:
        metrics_to_plot = list(CHAR_NAMES)
    else:
        unknown = set(metrics_to_plot) - set(CHAR_NAMES)
        if unknown:
            raise ValueError(f"Unknown metric names: {unknown}")

    # ------------------------------------------------------------------
    # 3-D surfaces
    # ------------------------------------------------------------------
    # Backend must be set *before* any figures are created.
    if show_plots:
        matplotlib.use("Qt5Agg")

    fig_paths: list[str] = []
    for name in metrics_to_plot:
        Z = metrics[name]
        if name == 'L2.5_rms_ay':
            Z = metrics['zone_length']**4.0 * metrics['rms_accel_y']
        fig = plt.figure(figsize=(8.5, 6.5))
        ax = fig.add_subplot(111, projection="3d")

        surf = ax.plot_surface(
            grid_a, grid_b, Z,
            cmap="viridis", edgecolor="none", alpha=0.85, antialiased=True,
        )
        ax.scatter(grid_a, grid_b, Z, c="k", s=12, depthshade=True, alpha=0.7)

        ax.set_xlabel(f"{label_a}  (DVS[{zone_row}][{idx_a}])", labelpad=8)
        ax.set_ylabel(f"{label_b}  (DVS[{zone_row}][{idx_b}])", labelpad=8)
        ax.set_zlabel(name, labelpad=8)
        ax.set_title(f"{zone} zone – {name}\n({label_a} × {label_b})")
        fig.colorbar(surf, ax=ax, shrink=0.55, pad=0.08, label=name)

        fig.tight_layout()
        png_name = f"grid3d_{zone}_{label_a}_vs_{label_b}_{name}.png"
        png_path = os.path.join(out_dir, png_name)
        fig.savefig(png_path, dpi=140)
        fig_paths.append(png_path)
        print(f"  3-D figure → {png_path}")

        if not show_plots:
            plt.close(fig)

    if show_plots:
        plt.show()

    print(f"Plotted {len(fig_paths)} characteristic(s).")
    return fig_paths

def run_cube_sweep(
    zone: str,
    idx_a: int,
    idx_b: int,
    idx_c: int,
    range_a: list[float],
    range_b: list[float],
    range_c: list[float],
    steps_a: int = 4,
    steps_b: int = 4,
    steps_c: int = 4,
    out_dir: str | None = None,
    show_motion_plots: bool = False,
) -> dict:
    """
    3-variable cube (Cartesian) sweep of design variables belonging to one zone.

    For every (val_a, val_b, val_c) on the product of the three linspaces:
      1. Mutate a fresh copy of DVS at the zone's row / (idx_a, idx_b, idx_c).
      2. Build the corresponding LinearCamContour + Contour.
      3. Run the headless physics simulation (identical settings to main / drive).
      4. Characterize both geometry (zone length) and stalk motion inside the zone.

    Results are written to a plain-text CSV (easy to reload later).  Plotting is
    deliberately left to plot_cube_results so expensive sims can be re-visualized
    without re-running.  Per-point stalk-motion figures are always saved; the
    show_motion_plots flag only controls whether figures stay open for a final
    interactive plt.show().

    Parameters
    ----------
    zone : str
        'Tip' | 'Leading' | 'Trailing' | 'Tail'
    idx_a, idx_b, idx_c : int
        Distinct column indices into that zone's DVS list (see ZONE_DV_NAMES).
    range_a, range_b, range_c : list[float]
        [min, max] for each variable.
    steps_a, steps_b, steps_c : int
        Number of samples along each axis (inclusive endpoints).  Keep modest
        (3–5) for interactive design loops; 5³ = 125 evaluations.
    out_dir : str, optional
        Directory for the CSV and motion PNGs.  Defaults to the package directory.
    show_motion_plots : bool
        If True, keep the figures open and call ``plt.show()`` after the whole
        cube finishes (useful only for tiny grids).  Motion PNGs are always
        written regardless of this flag.

    Returns
    -------
    dict
        {
          'zone': str,
          'idx_a': int, 'idx_b': int, 'idx_c': int,
          'label_a': str, 'label_b': str, 'label_c': str,
          'zone_row': int,
          'vals_a': 1-D array, 'vals_b': 1-D array, 'vals_c': 1-D array,
          'grid_a': 3-D mesh, 'grid_b': 3-D mesh, 'grid_c': 3-D mesh,
          'metrics': {name: 3-D array of shape (steps_a, steps_b, steps_c)},
          'csv_path': str,
        }
    """
    if zone not in ZONE_TO_DVS_IDX:
        raise ValueError(f"Unknown zone '{zone}'. Expected one of {list(ZONE_TO_DVS_IDX)}")

    zone_row = ZONE_TO_DVS_IDX[zone]
    names = ZONE_DV_NAMES[zone]
    n_names = len(names)
    for idx, tag in ((idx_a, "a"), (idx_b, "b"), (idx_c, "c")):
        if not (0 <= idx < n_names):
            raise IndexError(
                f"Zone '{zone}' has only {n_names} design variables "
                f"{names}; requested idx_{tag}={idx}."
            )
    if len({idx_a, idx_b, idx_c}) != 3:
        raise ValueError("idx_a, idx_b and idx_c must be three distinct indices.")

    label_a = names[idx_a]
    label_b = names[idx_b]
    label_c = names[idx_c]

    if out_dir is None:
        out_dir = os.path.dirname(__file__)
    os.makedirs(out_dir, exist_ok=True)

    vals_a = np.linspace(range_a[0], range_a[1], steps_a)
    vals_b = np.linspace(range_b[0], range_b[1], steps_b)
    vals_c = np.linspace(range_c[0], range_c[1], steps_c)
    # indexing="ij" keeps the first axis aligned with vals_a, etc.
    grid_a, grid_b, grid_c = np.meshgrid(vals_a, vals_b, vals_c, indexing="ij")

    metrics: dict[str, np.ndarray] = {
        name: np.full((steps_a, steps_b, steps_c), np.nan, dtype=float)
        for name in CHAR_NAMES
    }
    table_rows: list[list[float]] = []
    open_figs: list[plt.Figure] = []

    n_total = steps_a * steps_b * steps_c
    print("=" * 70)
    print(f"CUBE SWEEP  zone={zone}  vars=("
          f"{label_a}[{idx_a}], {label_b}[{idx_b}], {label_c}[{idx_c}])")
    print(f"  {label_a}: {range_a[0]:.4g} → {range_a[1]:.4g}  ({steps_a} steps)")
    print(f"  {label_b}: {range_b[0]:.4g} → {range_b[1]:.4g}  ({steps_b} steps)")
    print(f"  {label_c}: {range_c[0]:.4g} → {range_c[1]:.4g}  ({steps_c} steps)")
    print(f"  Total evaluations: {n_total}")
    print("=" * 70)

    t0_all = time.time()
    for ia in range(steps_a):
        for ib in range(steps_b):
            for ic in range(steps_c):
                k = (ia * steps_b + ib) * steps_c + ic + 1
                va = float(vals_a[ia])
                vb = float(vals_b[ib])
                vc = float(vals_c[ic])

                # Fresh independent copy of the design-variable table
                this_dvs = [row[:] for row in DVS]
                this_dvs[zone_row][idx_a] = va
                this_dvs[zone_row][idx_b] = vb
                this_dvs[zone_row][idx_c] = vc

                t0 = time.time()
                try:
                    cam, contour, meas_zones = build_contour(this_dvs, quiet=True)
                    length_overall, y_lead, y_trail = cam.overall_length()

                    duration = (contour.y_max - 0.115) / DRIVE_SPEED_MPS #+ 2.0
                    hist = simulate_drive(
                        contour,
                        drive_speed=DRIVE_SPEED_MPS,
                        lateral_x=-0.020,
                        duration=duration,
                        record_every=0.005,
                    )

                    chars = characterize_stalk_motion(
                        hist, meas_zones, zone, [y_lead, y_trail], verbose=False
                    )
                except (ValueError, RuntimeError, Exception) as exc:
                    # Extreme DV combinations can make build_profile / trim /
                    # overall_length / sensor-location logic raise.  Leave
                    # metrics as NaN and continue so the rest of the cube runs.
                    dt = time.time() - t0
                    print(
                        f"  [{k:3d}/{n_total}]  "
                        f"{label_a}={va:8.4f}  {label_b}={vb:8.4f}  {label_c}={vc:8.4f}  "
                        f"SKIPPED ({type(exc).__name__}: {exc})  ({dt:.2f}s)"
                    )
                    table_rows.append([va, vb, vc] + [float("nan")] * len(CHAR_NAMES))
                    continue

                dt = time.time() - t0

                for name, val in zip(CHAR_NAMES, chars):
                    metrics[name][ia, ib, ic] = val

                table_rows.append([va, vb, vc] + list(chars))

                print(
                    f"  [{k:3d}/{n_total}]  "
                    f"{label_a}={va:8.4f}  {label_b}={vb:8.4f}  {label_c}={vc:8.4f}  "
                    f"len={chars[0]:.4f}  rms_ay={chars[7]:.3f}  "
                    f"L*rms_ay={chars[9]:.4f}  ({dt:.2f}s)"
                )

                # ------------------------------------------------------------------
                # Optional per-point stalk-motion figure (always saved to disk)
                # ------------------------------------------------------------------
                suffix = (
                    f"  [{label_a}={va:.4g}, {label_b}={vb:.4g}, {label_c}={vc:.4g}]"
                )
                motion_png = os.path.join(
                    out_dir,
                    f"stalk_motion_{zone}_{ia:02d}_{ib:02d}_{ic:02d}"
                    f"_{label_a}_{label_b}_{label_c}.png",
                )
                matplotlib.use("Qt5Agg")
                fig = plot_stalk_motion(
                    hist,
                    save_path=motion_png,
                    meas_zones=meas_zones,
                    show=False,                 # never block inside the loop
                    close=not show_motion_plots,
                    title_suffix=suffix,
                    zone=zone,
                )
                if show_motion_plots:
                    open_figs.append(fig)

    elapsed = time.time() - t0_all
    print(f"\nCube complete in {elapsed:.1f}s  ({elapsed / n_total:.2f}s per eval)")

    # ------------------------------------------------------------------
    # Text-readable results file (CSV with header – trivial to reload)
    # ------------------------------------------------------------------
    csv_name = f"cube_sweep_{zone}_{label_a}_vs_{label_b}_vs_{label_c}.csv"
    csv_path = os.path.join(out_dir, csv_name)
    header = f"{label_a},{label_b},{label_c}," + ",".join(CHAR_NAMES)
    with open(csv_path, "w", encoding="utf-8") as fh:
        fh.write("# zone=" + zone + "\n")
        fh.write(
            f"# idx_a={idx_a} ({label_a}), "
            f"idx_b={idx_b} ({label_b}), "
            f"idx_c={idx_c} ({label_c})\n"
        )
        fh.write(f"# range_a={list(range_a)}, steps_a={steps_a}\n")
        fh.write(f"# range_b={list(range_b)}, steps_b={steps_b}\n")
        fh.write(f"# range_c={list(range_c)}, steps_c={steps_c}\n")
        fh.write(header + "\n")
        for row in table_rows:
            fh.write(",".join(f"{v:.8g}" for v in row) + "\n")
    print(f"Results saved → {csv_path}")

    return {
        "zone": zone,
        "idx_a": idx_a,
        "idx_b": idx_b,
        "idx_c": idx_c,
        "label_a": label_a,
        "label_b": label_b,
        "label_c": label_c,
        "zone_row": zone_row,
        "vals_a": vals_a,
        "vals_b": vals_b,
        "vals_c": vals_c,
        "grid_a": grid_a,
        "grid_b": grid_b,
        "grid_c": grid_c,
        "metrics": metrics,
        "csv_path": csv_path,
    }


def plot_cube_results(
    data: dict | None = None,
    csv_path: str | None = None,
    show_plots: bool = False,
    out_dir: str | None = None,
    metrics_to_plot: list[str] | None = None,
) -> list[str]:
    """
    Produce 3-D sphere-scatter plots from a completed cube sweep.

    Each design point is drawn as a sphere whose:
      - position = (val_a, val_b, val_c)
      - color   = value of the plotted characteristic
      - size    = scaled from the same characteristic (Option A: larger metric
                  value → larger sphere).  Sizes are normalized across the
                  finite values of that metric so the visual dynamic range is
                  useful.

    Accepts either the dict returned by run_cube_sweep or a path to a previously
    written CSV.  When both are given, the in-memory dict takes precedence.

    Parameters
    ----------
    data : dict, optional
        Return value of run_cube_sweep.
    csv_path : str, optional
        Path to a cube_sweep_*.csv file.  Used when data is None.
    show_plots : bool
        If True, switch to an interactive backend and call plt.show().
    out_dir : str, optional
        Directory for the PNG files.  Defaults to the directory of the CSV
        (or the package directory when only an in-memory dict is supplied).
    metrics_to_plot : list[str], optional
        Subset of CHAR_NAMES to plot.  Defaults to all characteristics.

    Returns
    -------
    list[str]
        Paths of the written PNG files.
    """
    if data is None and csv_path is None:
        raise ValueError("Provide either data= (from run_cube_sweep) or csv_path=.")

    file_dir = os.path.dirname(__file__)

    # ------------------------------------------------------------------
    # Resolve data source
    # ------------------------------------------------------------------
    if data is not None:
        zone      = data["zone"]
        idx_a     = data["idx_a"]
        idx_b     = data["idx_b"]
        idx_c     = data["idx_c"]
        label_a   = data["label_a"]
        label_b   = data["label_b"]
        label_c   = data["label_c"]
        zone_row  = data["zone_row"]
        vals_a    = data["vals_a"]
        vals_b    = data["vals_b"]
        vals_c    = data["vals_c"]
        grid_a    = data["grid_a"]
        grid_b    = data["grid_b"]
        grid_c    = data["grid_c"]
        metrics   = data["metrics"]
        if out_dir is None:
            out_dir = os.path.dirname(data.get("csv_path", file_dir))
    else:
        # Load from CSV (comment lines carry the metadata)
        # Allow relative names; join only when the path is not already absolute.
        if not os.path.isabs(csv_path):
            csv_path = os.path.join(file_dir, csv_path)
        if out_dir is None:
            out_dir = os.path.dirname(os.path.abspath(csv_path))

        # Collect all comment lines as a single blob for robust regex parsing.
        # Also keep a simple key=val dict for zone / ranges.
        comment_blob = ""
        meta: dict[str, str] = {}
        with open(csv_path, "r", encoding="utf-8") as fh:
            for line in fh:
                if not line.startswith("#"):
                    break
                comment_blob += line
                line = line[1:].strip()
                if "=" in line:
                    key, _, val = line.partition("=")
                    meta[key.strip()] = val.strip()

        zone = meta.get("zone", "Unknown")

        # Prefer parsing the full comment text so the leading "idx_a=" is present.
        import re
        m = re.search(
            r"idx_a=(\d+)\s*\(([^)]+)\).*?"
            r"idx_b=(\d+)\s*\(([^)]+)\).*?"
            r"idx_c=(\d+)\s*\(([^)]+)\)",
            comment_blob,
        )
        if m:
            idx_a   = int(m.group(1))
            label_a = m.group(2).strip()
            idx_b   = int(m.group(3))
            label_b = m.group(4).strip()
            idx_c   = int(m.group(5))
            label_c = m.group(6).strip()
        else:
            # Fallback: use header row for labels; indices are secondary.
            idx_a, idx_b, idx_c = 0, 1, 2
            label_a, label_b, label_c = "var_a", "var_b", "var_c"
            # Try to recover labels from the first non-comment line (CSV header)
            with open(csv_path, "r", encoding="utf-8") as fh:
                for line in fh:
                    if not line.startswith("#") and line.strip():
                        hdr = [h.strip() for h in line.split(",")]
                        if len(hdr) >= 3:
                            label_a, label_b, label_c = hdr[0], hdr[1], hdr[2]
                        break

        zone_row = ZONE_TO_DVS_IDX.get(zone, -1)

        # Numeric table (skip comment lines)
        raw = np.loadtxt(csv_path, delimiter=",", comments="#", skiprows=6)
        if raw.ndim == 1:
            raw = raw.reshape(1, -1)

        # Reconstruct unique sorted axes and the 3-D metric grids
        vals_a = np.unique(raw[:, 0])
        vals_b = np.unique(raw[:, 1])
        vals_c = np.unique(raw[:, 2])
        steps_a, steps_b, steps_c = len(vals_a), len(vals_b), len(vals_c)
        grid_a, grid_b, grid_c = np.meshgrid(vals_a, vals_b, vals_c, indexing="ij")

        metrics = {
            name: np.full((steps_a, steps_b, steps_c), np.nan)
            for name in CHAR_NAMES
        }
        # Map each row back onto the grid (order is ia outer → ic innermost)
        for row in raw:
            ia = int(np.argmin(np.abs(vals_a - row[0])))
            ib = int(np.argmin(np.abs(vals_b - row[1])))
            ic = int(np.argmin(np.abs(vals_c - row[2])))
            for j, name in enumerate(CHAR_NAMES):
                metrics[name][ia, ib, ic] = row[3 + j]

    os.makedirs(out_dir, exist_ok=True)

    if metrics_to_plot is None:
        metrics_to_plot = list(CHAR_NAMES)
    else:
        unknown = set(metrics_to_plot) - set(CHAR_NAMES)
        if unknown:
            raise ValueError(f"Unknown metric names: {unknown}")

    # ------------------------------------------------------------------
    # 3-D sphere scatters (color + size both driven by the metric)
    # ------------------------------------------------------------------
    if show_plots:
        matplotlib.use("Qt5Agg")

    fig_paths: list[str] = []
    for name in metrics_to_plot:
        Z = metrics[name]
        if name == 'L2.5_rms_ay':
            Z = metrics['zone_length'] * metrics['rms_accel_y']
        # Flatten for easy size / color handling; keep the grids aligned
        Xa = grid_a.ravel()
        Xb = grid_b.ravel()
        Xc = grid_c.ravel()
        Zf = Z.ravel()

        valid = np.isfinite(Zf)
        if not np.any(valid):
            print(f"  Skipping {name}: all values are NaN")
            continue

        zmin = float(np.min(Zf[valid]))
        zmax = float(np.max(Zf[valid]))
        # Option A: size scales with the metric value itself.
        # Normalize to a useful visual range of marker areas (points²).
        if zmax > zmin:
            sizes = 40.0 + 220.0 * (Zf - zmin) / (zmax - zmin)
        else:
            sizes = np.full_like(Zf, 100.0)
        # Hide invalid points by giving them zero area
        sizes = np.where(valid, sizes, 0.0)

        fig = plt.figure(figsize=(9.0, 7.0))
        ax = fig.add_subplot(111, projection="3d")

        sc = ax.scatter(
            Xa, Xb, Xc,
            c=Zf,
            s=sizes,
            cmap="viridis",
            depthshade=True,
            alpha=0.85,
            edgecolors="k",
            linewidths=0.35,
            vmin=zmin,
            vmax=zmax,
        )

        ax.set_xlabel(f"{label_a}  (DVS[{zone_row}][{idx_a}])", labelpad=8)
        ax.set_ylabel(f"{label_b}  (DVS[{zone_row}][{idx_b}])", labelpad=8)
        ax.set_zlabel(f"{label_c}  (DVS[{zone_row}][{idx_c}])", labelpad=8)
        ax.set_title(
            f"{zone} zone – {name}\n"
            f"({label_a} × {label_b} × {label_c})  [size ∝ {name}]"
        )
        fig.colorbar(sc, ax=ax, shrink=0.55, pad=0.08, label=name)

        fig.tight_layout()
        png_name = (
            f"cube3d_{zone}_{label_a}_vs_{label_b}_vs_{label_c}_{name}.png"
        )
        png_path = os.path.join(out_dir, png_name)
        fig.savefig(png_path, dpi=140)
        fig_paths.append(png_path)
        print(f"  3-D sphere figure → {png_path}")

        if not show_plots:
            plt.close(fig)

    if show_plots:
        plt.show()

    print(f"Plotted {len(fig_paths)} characteristic(s).")
    return fig_paths



if __name__ == "__main__":
    main()
    # Example 1-D sweep (legacy)
    # run_sweep(0, 0, [0.1, 0.5], 3)

    # Example 2-variable grid sweep for the Trailing zone
    #   idx 0 = trailing_dwell, idx 1 = length
    # run_grid_sweep(
    #     zone="Trailing",
    #     idx_a=1,
    #     idx_b=2,
    #     range_a=[0.50,  0.800],
    #     range_b=[1.05, 1.3],
    #     steps_a=10,
    #     steps_b=10,
    #     show_motion_plots=True,
    # )

    # plot_grid_results(
    #     csv_path="grid_sweep_Trailing_length_vs_tp.csv",
    #     show_plots=True,
    #     metrics_to_plot=["L2.5_rms_ay"]#, "rms_accel_y", "peak_accel_y", "zone_length"],  # optional subset
    # )

       # ------------------------------------------------------------------
    # Example 3-variable cube sweep for the Tip zone
    #   idx 0 = length, 1 = delta_x, 2 = tp
    # Keep steps modest (4³ = 64) while exploring; raise carefully.
    # ------------------------------------------------------------------
    # run_cube_sweep(
    #     zone="Tip",
    #     idx_a=0,          # length
    #     idx_b=1,          # delta_x
    #     idx_c=2,          # tp
    #     range_a=[0.24, 0.500],
    #     range_b=[-0.060, -0.400],
    #     range_c=[0.60, 1.4],
    #     steps_a=6,
    #     steps_b=6,
    #     steps_c=6,
    #     show_motion_plots=False,
    # )
    
    # plot_cube_results(
    #     csv_path="cube_sweep_Tip_length_vs_delta_x_vs_tp.csv",
    #     show_plots=True,
    #     metrics_to_plot=["L2.5_rms_ay", "rms_accel_y", "peak_accel_y", "zone_length"],
    # )
