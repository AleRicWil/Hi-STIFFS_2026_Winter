#!/usr/bin/env python3
"""
contour.py – Contour representation, loading, resampling and parametric cam builder.

Contains:
  - load_contour_from_csv / resample_contour_even_y (exact from stalk_contour_drive)
  - Contour dataclass (exact from stalk_contour_drive)
  - LinearCamContour class (exact from original geomtry_opt.py) for creation &
    kinematics evaluation
"""

from __future__ import annotations

import os
import csv
from dataclasses import dataclass, field
from typing import List, Tuple, Callable, Optional

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use("Agg")          # headless-safe
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Loading & resampling (verbatim from stalk_contour_drive.py)
# ---------------------------------------------------------------------------
def load_contour_from_csv(csv_path: str) -> np.ndarray:
    """
    Load a sequence of contour points from a CSV file.

    ----------------------------------------------------------------
    USER: replace the body of this function with the exact parsing
    required by your CSV format.  The only hard requirement is that
    the function returns a NumPy array of shape (N, 2) and dtype
    float64 containing [x, y] coordinates in metres.
    ----------------------------------------------------------------

    Current skeleton (works for many simple CSVs):
      - comma-separated
      - optional header row (skipped if it cannot be parsed as float)
      - at least two columns; only the first two are used
    """
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Contour CSV not found: {csv_path}")

    # ---- BEGIN USER-EDITABLE SECTION ---------------------------------
    data = np.genfromtxt(
        csv_path,
        delimiter=",",
        comments="#",
        skip_header=1,          # set to 1 if first row is a header
        usecols=(0, 1),         # columns that contain x and y
        dtype=np.float64,
    )
    # ---- END USER-EDITABLE SECTION -----------------------------------

    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 2:
        raise ValueError(
            f"CSV must supply at least two columns (x,y); got shape {data.shape}"
        )
    points = np.ascontiguousarray(data[:, :2], dtype=np.float64)

    if points.shape[0] < 2:
        raise ValueError("Contour must contain at least two points")

    # Basic sanity: replace any NaNs that may have come from a header
    if np.isnan(points).any():
        mask = ~np.isnan(points).any(axis=1)
        points = points[mask]
        if points.shape[0] < 2:
            raise ValueError(
                "After removing non-numeric rows the contour has < 2 points. "
                "Check skip_header / CSV format."
            )

    # Auto-detect units: if the first few numeric values are large (> 10),
    # assume the CSV is in millimetres and convert to metres.
    n_sample = min(5, points.shape[0])
    sample = np.abs(points[:n_sample]).ravel()
    if sample.size > 0 and np.nanmax(sample) > 10.0:
        points *= 0.001  # mm → m
        print(
            "Contour values appear to be in mm (max sample > 10); "
            "auto-scaled by 0.001 to metres."
        )

    return points


def resample_contour_even_y(points: np.ndarray, n_points: int, 
                            y_lim: tuple[float, float] | list[float] | None = None,) -> np.ndarray:
    """
    Resample a polyline so the output points have Y-coordinates that are
    evenly spaced.

    Parameters
    ----------
    points : (N, 2) array
        Original contour points [x, y].
    n_points : int
        Desired number of output points.
    y_lim : (y_start, y_end) or None, optional
        If given, the n_points are placed from y_start to y_end (inclusive).
        When None the data’s own y_max → y_min are used (original behaviour).

    Returns
    -------
    (n_points, 2) array with columns [x, y], y running from the chosen
    start limit to the end limit.
    """
    if n_points < 2 or points.shape[0] < 2:
        return points

    y = points[:, 1]
    data_ymin = float(y.min())
    data_ymax = float(y.max())
    if abs(data_ymax - data_ymin) < 1e-12:
        return points  # degenerate

    # Determine the sampling interval
    if y_lim is None:
        y_start, y_end = data_ymax, data_ymin
    else:
        y_start, y_end = float(y_lim[0]), float(y_lim[1])

    target_ys = np.linspace(y_start, y_end, n_points)

    # Sort once so interp is well-defined
    sort_idx = np.argsort(y)
    y_sorted = y[sort_idx]
    x_sorted = points[sort_idx, 0]

    # Drop exact duplicate y-values (keeps interp stable)
    keep = np.concatenate(([True], np.diff(y_sorted) > 1e-12))
    y_sorted = y_sorted[keep]
    x_sorted = x_sorted[keep]

    x_new = np.interp(target_ys, y_sorted, x_sorted)
    return np.column_stack((x_new, target_ys))

# ---------------------------------------------------------------------------
# Contour – static polyline (exact from stalk_contour_drive.py)
# ---------------------------------------------------------------------------
@dataclass
class Contour:
    """Static polyline defined by an ordered sequence of points."""

    points: np.ndarray                    # (N, 2)
    segments: List[Tuple[np.ndarray, np.ndarray]] = field(init=False)
    # Per-segment axis-aligned bounding boxes: (xmin, xmax, ymin, ymax)
    seg_aabb: List[Tuple[float, float, float, float]] = field(init=False)
    centroid: np.ndarray = field(init=False)
    y_min: float = field(init=False)
    y_max: float = field(init=False)
    x_min: float = field(init=False)
    x_max: float = field(init=False)

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=np.float64)
        self.segments = []
        self.seg_aabb = []
        for i in range(len(self.points) - 1):
            a = self.points[i].copy()
            b = self.points[i + 1].copy()
            self.segments.append((a, b))
            self.seg_aabb.append((
                float(min(a[0], b[0])),
                float(max(a[0], b[0])),
                float(min(a[1], b[1])),
                float(max(a[1], b[1])),
            ))
        self.centroid = self.points.mean(axis=0)
        self.x_min, self.y_min = self.points.min(axis=0)
        self.x_max, self.y_max = self.points.max(axis=0)

    @property
    def extent(self) -> float:
        """Characteristic size used for auto-scaling the view."""
        return max(self.x_max - self.x_min, self.y_max - self.y_min)


# ---------------------------------------------------------------------------
# LinearCamContour – parametric builder + kinematics (from geomtry_opt.py)
# ---------------------------------------------------------------------------
class LinearCamContour:
    """Linear cam profile generator for stalk pusher.
    Optimizes transition length L for smoothness vs. total device length.
    All equations taken directly from CAM MECHANISMS.pdf with Y-linear mapping.
    """

    def __init__(self, vy: float = 1.0, path: str = None):
        self.vy = vy  # forward speed m/s
        self.y: np.ndarray | None = None
        self.x: np.ndarray | None = None
        self.delta_y: float | None = None
        if path is not None:
            self._import_profile(path)

    def _import_profile(self, path: str) -> Tuple[np.ndarray, np.ndarray]:
        """Import contour [x, y] profile from existing CSV."""
        df = pd.read_csv(path)
        self.x = df['X(mm)'].values * 1e-3
        self.y = df['Y(mm)'].values * 1e-3
        self._resample_y()
        return self.y, self.x

    def _resample_y(self) -> Tuple[np.ndarray, np.ndarray]:
        """Resamples for evenly spaced points along y (strictly monotonic)."""
        if self.y is None or self.x is None:
            raise ValueError("No profile loaded yet.")

        y_orig, x_orig = self.y.copy(), self.x.copy()

        # Ensure y is sorted (robust for imported or generated data)
        sort_idx = np.argsort(y_orig)
        y_orig = y_orig[sort_idx]
        x_orig = x_orig[sort_idx]

        self.delta_y = round(np.mean(np.diff(y_orig)), 6)
        print(f"Δy: {self.delta_y*1e3:.3f} mm")
        print(f"Number of original points: {len(y_orig)}")

        y_min = y_orig.min()
        y_max = y_orig.max()
        self.y = np.arange(y_min, y_max + self.delta_y / 2, self.delta_y)
        self.x = np.interp(self.y, y_orig, x_orig)

        return self.y, self.x

    @staticmethod
    def cycloidal(tau: np.ndarray) -> np.ndarray:
        """Normalized cycloidal (tau ∈ [0,1] → s_norm ∈ [0,1])."""
        return tau - 0.5 / np.pi * np.sin(2 * np.pi * tau)

    @staticmethod
    def poly_345(tau: np.ndarray) -> np.ndarray:
        """3-4-5 polynomial."""
        return 10*tau**3 - 15*tau**4 + 6*tau**5

    @staticmethod
    def poly_3456(tau: np.ndarray, tau_p: float = 0.50) -> np.ndarray:
        """
        6th-degree polynomial with the same end conditions as classic 3-4-5
        (s = s' = s'' = 0 at both ends) plus one interior condition that places
        the acceleration peak at tau_p.

        tau_p = 0.5 recovers the ordinary 3-4-5 (handled by fallback).
        tau_p > 0.5 → gentler start, steeper finish.
        tau_p < 0.5 → steeper start, gentler finish.
        Useful practical range ≈ 0.40 … 0.65.
        """
        tau = np.asarray(tau, dtype=float)
        if abs(tau_p - 0.5) < 1e-3:
            # classic 3-4-5
            print('Tau_peak was too-near 0.5. Reverting from poly_3456 to poly_345')
            return 10*tau**3 - 15*tau**4 + 6*tau**5

        den = 20*tau_p**3 - 30*tau_p**2 + 12*tau_p - 1
        a3 = 20*tau_p*(10*tau_p**2 - 12*tau_p + 3) / den
        a4 = 15*(-20*tau_p**3 + 18*tau_p**2 - 1) / den
        a5 = 12*(10*tau_p**3 - 9*tau_p + 2) / den
        a6 = 10*(-6*tau_p**2 + 6*tau_p - 1) / den

        return a3*tau**3 + a4*tau**4 + a5*tau**5 + a6*tau**6

    @staticmethod
    def poly_4567(tau: np.ndarray) -> np.ndarray:
        """4-5-6-7 polynomial."""
        return 35*tau**4 - 84*tau**5 + 70*tau**6 - 20*tau**7

    def _transition(self, y: np.ndarray, y_start: float, x_start: float,
                    H: float, L: float, curve: Callable[[np.ndarray], np.ndarray],
                    direction: int = 1, tau_p: float = 0.5) -> np.ndarray:
        """Generate X(y) for one transition. Works for both lead (+y) and trail (−y)."""
        if L <= 0:
            raise ValueError("Transition length L must be positive.")

        # tau always progresses from 0 → 1 along the path direction
        tau = direction * (y - y_start) / L
        s_norm = np.zeros_like(y, dtype=float)
        mask = (tau >= 0) & (tau <= 1)
        s_norm[mask] = curve(np.clip(tau[mask], 0.0, 1.0), tau_p)

        return x_start + H * s_norm

    def build_profile(self, segments: List[dict], curve_func: Callable = None) -> Tuple[np.ndarray, np.ndarray]:
        """Build full X(Y) profile from list of segments.
        Guarantees continuity at every junction and between lead/trail sides.
        """
        if curve_func is None:
            curve_func = self.cycloidal

        dy = 1.0e-4
        points: List[Tuple[float, float]] = []  # (y, x) pairs

        # === LEAD segments (+y) ===
        y_current = 0.0
        x_current = 0.0
        lead_segs = [s for s in segments if s.get('dir') == 'lead']

        for i, seg in enumerate(lead_segs):
            L = float(seg['length'])
            if seg['type'] == 'dwell':
                target_x = seg.get('x', x_current)
                if abs(target_x - x_current) > 1e-6 and i > 0:
                    print(f"Warning: Lead dwell jump at y={y_current:.4f} m: "
                          f"{x_current:.5f} → {target_x:.5f}")
                x_current = target_x
                y_end = y_current + L
                y_seg = np.linspace(y_current, y_end, int(L / dy) + 1)
                x_seg = np.full_like(y_seg, x_current)
            else:  # rise or fall
                H = float(seg.get('delta_x', 0.0))
                tp = float(seg.get('tp', 0.5))
                y_end = y_current + L
                y_seg = np.linspace(y_current, y_end, int(L / dy) + 1)
                x_seg = self._transition(y_seg, y_current, x_current, H, L, curve_func, direction=1, tau_p=tp)
                x_current += H

            for yy, xx in zip(y_seg, x_seg):
                points.append((yy, xx))
            y_current = y_end

        # Capture x at y=0 for perfect trail continuity
        x_at_origin = points[0][1] if points else 0.0

        # === TRAIL segments (−y) ===
        y_current = 0.0
        x_current = x_at_origin   # must match lead side at y=0
        trail_segs = [s for s in segments if s.get('dir') == 'trail']

        for i, seg in enumerate(trail_segs):
            L = float(seg['length'])
            if seg['type'] == 'dwell':
                target_x = seg.get('x', x_current)
                if abs(target_x - x_current) > 1e-6:
                    print(f"Warning: Trail dwell jump at y={y_current:.4f} m: "
                          f"{x_current:.5f} → {target_x:.5f}")
                x_current = target_x
                y_end = y_current - L
                y_seg = np.linspace(y_current, y_end, int(L / dy) + 1)
                x_seg = np.full_like(y_seg, x_current)
            else:
                H = float(seg.get('delta_x', 0.0))
                tp = float(seg.get('tp', 0.5))
                y_end = y_current - L
                y_seg = np.linspace(y_current, y_end, int(L / dy) + 1)
                x_seg = self._transition(y_seg, y_current, x_current, H, L, curve_func, direction=-1, tau_p=tp)
                x_current += H

            for yy, xx in zip(y_seg, x_seg):
                points.append((yy, xx))
            y_current = y_end

        # Convert to arrays and guarantee monotonic y
        if not points:
            raise ValueError("No segments provided.")

        y_arr = np.array([p[0] for p in points])
        x_arr = np.array([p[1] for p in points])

        # Sort by increasing y
        sort_idx = np.argsort(y_arr)
        self.y = y_arr[sort_idx]
        self.x = x_arr[sort_idx]

        # Remove near-duplicates at junctions (including y=0)
        diff_y = np.diff(self.y)
        keep = np.concatenate(([True], diff_y > 1e-9))
        self.y = self.y[keep]
        self.x = self.x[keep]

        self._resample_y()
        print(f"Final profile: {len(self.y)} points, y range [{self.y.min():.4f}, {self.y.max():.4f}] m")
        self.x[0] = self.x[1]
        self.x[-1] = self.x[-2]
        return self.y, self.x

    def plot_kinematics(self, vy: float = None, save_path: Optional[str] = None):
        """Plot displacement, velocity, acceleration for engineering review.
        If save_path is given, writes a PNG instead of (or in addition to) showing.
        """
        if vy is None:
            vy = self.vy
        if self.y is None or self.x is None:
            print("No profile built yet. Call build_profile() first.")
            return

        # Numerical derivatives + Savitzky-Golay smoothing
        window_length = 31
        polyorder = 1

        dx_dy   = savgol_filter(self.x,  window_length, polyorder, deriv=1, delta=self.delta_y)
        d2x_dy2 = savgol_filter(dx_dy,   window_length, polyorder, deriv=1, delta=self.delta_y)
        d3x_dy3 = savgol_filter(d2x_dy2, window_length, polyorder, deriv=1, delta=self.delta_y)

        vx = dx_dy * vy
        ax = d2x_dy2 * vy**2
        jx = d3x_dy3 * vy**3

        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

        axes[0].plot(self.y, self.x, 'b-', linewidth=1.5)
        axes[0].set_ylabel('Lateral Position (m)')
        axes[1].plot(self.y, vx, 'r-', linewidth=1.5)
        axes[1].set_ylabel('Vx (m/s)')
        axes[2].plot(self.y, ax, 'g-', linewidth=1.5)
        axes[2].set_ylabel('Ax (m/s²)')
        axes[2].set_xlabel('Longitudinal Position (m)')

        for axis in axes:
            axis.axhline(0, linewidth=0.8, color='k', alpha=0.6)
            axis.axvline(0, linewidth=0.8, color='k', alpha=0.6)
            axis.grid(True, alpha=0.3)
            axis.xaxis.set_inverted(True)

        axes[0].axis('equal')
        fig.suptitle(f'Cam Kinematics — Forward Speed: {vy:.3f} m/s ({vy*2.23694:.2f} mph)')
        plt.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=150)
            print(f"Saved kinematics figure → {save_path}")
        else:
            # Attempt interactive show only if a display is available
            try:
                plt.get_current_fig_manager().window.move(0, 0)
            except Exception:
                pass
            plt.show()

        print(f"Peak lateral acceleration: {np.max(np.abs(ax)):.2f} m/s²")
        print(f"Peak jerk: {np.max(np.abs(jx)):.2f} m/s³")
        plt.close(fig)

    def export_profile(self, path: str = None):
        '''Saves the current profile as a CSV of XYZ coordinates (Z=0)'''
        if self.y is None or self.x is None:
            raise ValueError("No profile available. Build or import a profile first.")

        if path is None:
            path = "cam_profile_export.csv"

        # Convert from meters to mm to match the import format
        x_mm = self.x * 1000
        y_mm = self.y * 1000
        z_mm = np.zeros_like(x_mm)

        with open(path, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            # Header compatible with _import_profile and extended for XYZ
            writer.writerow(['X(mm)', 'Y(mm)', 'Z(mm)'])

            # Write rows with nano-meter precision
            for x_val, y_val, z_val in zip(x_mm, y_mm, z_mm):
                writer.writerow([f"{x_val:.6f}", f"{y_val:.6f}", f"{z_val:.6f}"])
        print(f"Exported profile → {path}")

    def to_contour_points(self) -> np.ndarray:
        """Return (N,2) array suitable for Contour(points=...)."""
        if self.y is None or self.x is None:
            raise ValueError("No profile built.")
        return np.column_stack([self.x, self.y])
