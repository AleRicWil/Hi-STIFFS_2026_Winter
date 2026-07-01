#!/usr/bin/env python3
"""
Hi-STIFFS IMU CSV Post-Processor & 3D Path Visualizer
=====================================================

PURPOSE
-------
Convert raw int16 CSV logs (from read_IMU_serial.py) into a 3D trajectory
with origin forced to t=0.0 of the selected analysis window. Default view
is top-down (X-Y) — perfect for quick sanity checks of 1-2 s rolling pose
windows used for tractor-mounted probe drift correction in stalk stiffness
measurements.

This script is the offline validation twin of the real-time deque-based
pose estimator we will deploy on the RPi5. Every constant and scaling
factor is taken directly from 9DOF_IMU_WING_test.ino so results are
directly comparable to what the embedded node will see.

DESIGN PRINCIPLES (matching project constraints)
------------------------------------------------
- Zero external heavy libs beyond numpy + matplotlib + scipy (already in
  your Hi-STIFFS / ME 575 environment).
- All hot loops are O(n) and vector-friendly where possible.
- Windowing (--start / --duration) lets you zoom to any 1-2 s stalk
  contact simulation without re-logging.
- Extremely verbose comments so you (or future team members) can
  understand every engineering decision and safely extend it.
- Origin is ALWAYS reset to the first sample of the chosen window →
  relative displacement is what matters for flexural stiffness correction.

PACKET / SCALING ALIGNMENT
--------------------------
Packet: sync(0xAA) + uint64_t ts_us + 6×int16 (gx gy gz ax ay az) + 3×int16 (mx my mz)
All values remain RAW sensor integers until this script — exactly as the
pose estimator will receive them over WiFi or USB.

Sensitivities (datasheet-driven, match your current sketch):
  ISM330DHCX accel  ±8 g   → 4096 LSB/g
  ISM330DHCX gyro   ±2000 dps → 16.384 LSB/dps
  LIS3MDL mag       ±12 gauss → 2281 LSB/gauss

USAGE EXAMPLES (copy-paste ready)
---------------------------------
# Whole log, top-down view, origin at t=0
python histiffs_imu_pose_visualizer.py --csv histiffs_imu_20250628_231045.csv

# Focus on a specific 2-second "stalk contact" window starting at 45.3 s
python histiffs_imu_pose_visualizer.py --csv histiffs_imu_20250628_231045.csv \
    --start 45.3 --duration 2.0

# 3D view instead of default top-down
python histiffs_imu_pose_visualizer.py --csv ... --start 10 --duration 5 --no-topdown

Author: Lead Engineer – Hi-STIFFS IMU / 3D Pose Tracking
Date:   2026-06-28
"""

import sys
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid


# ====================== EXACT SKETCH CONSTANTS (DO NOT CHANGE) ======================
# These must stay in sync with 9DOF_IMU_WING_test.ino so physical units are correct.
TARGET_ACCEL_FS_G   = 8          # g
TARGET_GYRO_FS_DPS  = 2000       # dps
TARGET_MAG_FS_GAUSS = 12         # gauss

# Derived sensitivities from datasheets (ISM330DHCX Tables 42/45, LIS3MDL Table 22)
ACCEL_SENS_LSB_PER_G   = 4096.0                    # 2**15 / 8 g
GYRO_SENS_LSB_PER_DPS  = 32768.0 / TARGET_GYRO_FS_DPS   # 16.384 LSB/dps
MAG_SENS_LSB_PER_GAUSS = 2281.0                    # for ±12 gauss range

G = 9.80665  # standard gravity (m/s^2) — used for both removal and initial attitude


def load_csv_window(csv_path: str, start_s: float, duration_s: float):
    """
    Load CSV, skip the 8-line metadata header written by the monitor,
    apply time window, and reset time origin to 0.0 for the selected window.
    This gives us exactly the relative 1-2 s pose we care about for stalk
    contact drift correction.
    """
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    # The monitor writes exactly 8 comment lines (#) then the plain header line.
    # np.genfromtxt with names=True reads the header automatically.
    data = np.genfromtxt(
        csv_path,
        delimiter=',',
        skip_header=9,          # 8 comment lines + 1 header line
        names=True,
        dtype=None,             # let it infer
        encoding=None
    )

    if len(data) == 0:
        raise ValueError("CSV appears empty after header.")

    ts = data['ts_s']
    mask = (ts >= start_s) & (ts <= start_s + duration_s)
    if np.sum(mask) < 5:
        raise ValueError(f"Window [{start_s:.2f}, {start_s+duration_s:.2f}] contains < 5 samples. "
                         "Check your --start/--duration or log length.")

    ts_win = ts[mask] - ts[mask][0]   # FORCE ORIGIN TO t=0.0 of this window
    gx = data['gx'][mask].astype(np.int16)
    gy = data['gy'][mask].astype(np.int16)
    gz = data['gz'][mask].astype(np.int16)
    ax = data['ax'][mask].astype(np.int16)
    ay = data['ay'][mask].astype(np.int16)
    az = data['az'][mask].astype(np.int16)

    print(f"[INFO] Loaded {len(ts_win)} samples from {start_s:.2f}s to {start_s+duration_s:.2f}s "
          f"(window duration {ts_win[-1]:.3f}s)")
    return ts_win, gx, gy, gz, ax, ay, az


def raw_to_physical(gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw):
    """
    Convert raw int16 counts → SI units using the exact sensitivities
    that match your current Arduino sketch configuration.
    All downstream pose math operates in these units.
    """
    # Accelerometer: body-frame m/s²
    ax = (ax_raw / ACCEL_SENS_LSB_PER_G) * G
    ay = (ay_raw / ACCEL_SENS_LSB_PER_G) * G
    az = (az_raw / ACCEL_SENS_LSB_PER_G) * G

    # Gyroscope: body-frame rad/s
    gx = np.deg2rad(gx_raw / GYRO_SENS_LSB_PER_DPS)
    gy = np.deg2rad(gy_raw / GYRO_SENS_LSB_PER_DPS)
    gz = np.deg2rad(gz_raw / GYRO_SENS_LSB_PER_DPS)

    return gx, gy, gz, ax, ay, az


def estimate_initial_attitude(ax0, ay0, az0):
    """
    Compute roll & pitch from the gravity vector at the first sample of the window.
    Yaw is set to 0 (we will add magnetometer fusion or gyro bias tracking later).
    This gives a reasonable starting orientation for short 1-2 s windows.
    For best results in real tests: keep the probe still for the first 0.5–1 s of the log.
    """
    roll  = np.arctan2(ay0, az0)
    pitch = np.arctan2(-ax0, np.sqrt(ay0**2 + az0**2))
    yaw   = 0.0
    return roll, pitch, yaw


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Body-to-world rotation matrix (ZYX Euler convention).
    v_world = R @ v_body
    Used to rotate accelerometer readings into the inertial frame before
    gravity removal and double integration.
    NOTE: For production code we will switch to quaternions to avoid
    gimbal lock and reduce trigonometric calls.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [ cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy ],
        [ cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy ],
        [ -sp,    sr*cp,             cr*cp            ]
    ])
    return R


def compute_relative_trajectory(ts, gx, gy, gz, ax, ay, az):
    """
    Core INS-style pose estimator for a single analysis window.

    Steps (exactly what the future RPi5 real-time node will do at 100–200 Hz):
      1. Start with attitude from gravity at t_window=0.
      2. Integrate gyro (simple forward Euler) to propagate attitude.
      3. Rotate body accel → world frame using current attitude.
      4. Subtract gravity in world Z.
      5. Double integrate (velocity then position) with trapezoidal-style accumulation.

    Returns position arrays (origin = 0,0,0 at start of window) + final attitude.
    """
    n = len(ts)
    if n < 2:
        return np.zeros(1), np.zeros(1), np.zeros(1), 0.0, 0.0, 0.0

    # Pre-allocate
    pos_x = np.zeros(n)
    pos_y = np.zeros(n)
    pos_z = np.zeros(n)
    vel_x = np.zeros(n)
    vel_y = np.zeros(n)
    vel_z = np.zeros(n)

    # Initial conditions for this window
    roll, pitch, yaw = estimate_initial_attitude(ax[0], ay[0], az[0])
    print(f"[INFO] Initial attitude (window start): roll={np.rad2deg(roll):.1f}°, "
          f"pitch={np.rad2deg(pitch):.1f}°, yaw={np.rad2deg(yaw):.1f}°")

    for i in range(1, n):
        dti = ts[i] - ts[i-1]
        if dti <= 0:
            dti = 1e-6   # guard against duplicate timestamps

        # 1. Propagate attitude with gyro (forward Euler — replace with quaternion later)
        roll  += gx[i] * dti
        pitch += gy[i] * dti
        yaw   += gz[i] * dti

        # 2. Rotate current body accel into world frame
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        a_body = np.array([ax[i], ay[i], az[i]])
        a_world = R @ a_body

        # 3. Remove gravity (world Z is up)
        a_world[2] -= G

        # 4. Integrate acceleration → velocity (simple accumulation; trapezoidal would need a_world history)
        vel_x[i] = vel_x[i-1] + a_world[0] * dti
        vel_y[i] = vel_y[i-1] + a_world[1] * dti
        vel_z[i] = vel_z[i-1] + a_world[2] * dti

        # 5. Integrate velocity → position
        pos_x[i] = pos_x[i-1] + (vel_x[i] + vel_x[i-1]) * 0.5 * dti   # trapezoidal
        pos_y[i] = pos_y[i-1] + (vel_y[i] + vel_y[i-1]) * 0.5 * dti
        pos_z[i] = pos_z[i-1] + (vel_z[i] + vel_z[i-1]) * 0.5 * dti

    # Summary metrics for this window (useful for stalk contact analysis)
    final_pos = np.array([pos_x[-1], pos_y[-1], pos_z[-1]])
    path_length = np.sum(np.sqrt(np.diff(pos_x)**2 + np.diff(pos_y)**2 + np.diff(pos_z)**2))
    print(f"[RESULT] Window displacement: X={final_pos[0]*1000:.1f} mm, "
          f"Y={final_pos[1]*1000:.1f} mm, Z={final_pos[2]*1000:.1f} mm")
    print(f"[RESULT] Total path length in window: {path_length*1000:.1f} mm")

    return pos_x, pos_y, pos_z, roll, pitch, yaw


def plot_trajectory(ts, pos_x, pos_y, pos_z, start_s, duration_s, topdown=True):
    """
    Matplotlib visualization. Default = top-down (X-Y) view as requested.
    Also shows a small 3D inset or side view if desired.
    """
    fig = plt.figure(figsize=(11, 8))
    fig.suptitle(f"Hi-STIFFS IMU Relative Path | Window [{start_s:.2f}–{start_s+duration_s:.2f} s] | "
                 f"Origin forced to t=0 of window", fontsize=13, fontweight='bold')

    if topdown:
        ax = fig.add_subplot(111)
        ax.plot(pos_x, pos_y, 'b-', linewidth=2.0, label='Estimated trajectory')
        ax.plot(0, 0, 'go', markersize=12, markeredgecolor='k', label='Origin (start of window)')
        ax.set_xlabel('X (m)  [body X rotated to world]', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)
        ax.set_title('Top-Down View (default) — use for quick drift sanity check', fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.legend(loc='upper right')
        # Add a light arrow showing final displacement
        if len(pos_x) > 1:
            ax.annotate('', xy=(pos_x[-1], pos_y[-1]), xytext=(0, 0),
                        arrowprops=dict(arrowstyle='->', color='red', lw=1.5))
    else:
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(pos_x, pos_y, pos_z, 'b-', linewidth=1.8, label='3D path')
        ax.plot([0], [0], [0], 'go', markersize=10, label='Origin')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D View (use --no-topdown to force)', fontsize=11)
        ax.legend()

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Hi-STIFFS IMU trajectory visualizer — origin reset per 1-2 s analysis window"
    )
    parser.add_argument("--csv", default=r'histiffs_imu_20260628_232635.csv', help="Path to histiffs_imu_*.csv from the monitor")
    parser.add_argument("--start", type=float, default=2.0,
                        help="Start time of analysis window in seconds (default 0.0)")
    parser.add_argument("--duration", type=float, default=7.0,
                        help="Duration of analysis window in seconds (default 2.0 → typical stalk contact)")
    parser.add_argument("--no-topdown", dest="topdown", action="store_false",
                        help="Force 3D plot instead of default top-down (X-Y) view")
    args = parser.parse_args()

    print("╔════════════════════════════════════════════════════════════════════════════╗")
    print("║   Hi-STIFFS IMU Post-Processor — Relative 3D Pose for 1-2 s Windows        ║")
    print("╚════════════════════════════════════════════════════════════════════════════╝")

    ts, gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = load_csv_window(
        args.csv, args.start, args.duration
    )

    gx, gy, gz, ax, ay, az = raw_to_physical(gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw)

    pos_x, pos_y, pos_z, final_roll, final_pitch, final_yaw = compute_relative_trajectory(
        ts, gx, gy, gz, ax, ay, az
    )

    plot_trajectory(ts, pos_x, pos_y, pos_z, args.start, args.duration, topdown=args.topdown)

    print("\n[INFO] Script finished. Next steps for production:")
    print("  • Port the compute_relative_trajectory() core (quaternion version) into the RPi5 node")
    print("  • Add gyro bias estimation from stationary prefix of each 1-2 s window")
    print("  • Fuse LIS3MDL magnetometer for yaw drift correction (Madgwick or EKF)")
    print("  • Use the same deque maxlen ring-buffer pattern as read_IMU_serial.py for O(1) memory")


if __name__ == "__main__":
    main()