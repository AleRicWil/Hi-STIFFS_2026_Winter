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


def resample_contour_even_y(points: np.ndarray, n_points: int) -> np.ndarray:
    """
    Resample a polyline so the output points have Y-coordinates that are
    evenly spaced between the original y_max and y_min (descending, matching
    the -Y drive direction).

    For each target Y the corresponding X is obtained by linear interpolation
    on the first polyline segment that straddles that Y.  If no segment
    crosses the target Y (possible with non-monotonic or multi-valued
    contours), the nearest original point in Y is used as a fallback.

    This produces a uniform contact-test density along the drive axis while
    preserving the essential shape of typical stalk / cam profiles.
    """
    if n_points < 2 or points.shape[0] < 2:
        return points

    y = points[:, 1]
    y_min = float(y.min())
    y_max = float(y.max())
    if abs(y_max - y_min) < 1e-12:
        return points  # degenerate (all points at same Y)

    target_ys = np.linspace(y_max, y_min, n_points)
    new_pts = np.empty((n_points, 2), dtype=np.float64)

    for i, ty in enumerate(target_ys):
        found = False
        for j in range(len(points) - 1):
            y0 = points[j, 1]
            y1 = points[j + 1, 1]
            # Does segment [j, j+1] straddle or touch ty?
            if (y0 - ty) * (y1 - ty) <= 0.0:
                if abs(y1 - y0) < 1e-12:
                    x = points[j, 0]
                else:
                    t = (ty - y0) / (y1 - y0)
                    x = points[j, 0] + t * (points[j + 1, 0] - points[j, 0])
                new_pts[i] = (x, ty)
                found = True
                break
        if not found:
            # Fallback: nearest original point in Y
            idx = int(np.argmin(np.abs(y - ty)))
            new_pts[i] = points[idx]

    return new_pts


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
    def poly_4567(tau: np.ndarray) -> np.ndarray:
        """4-5-6-7 polynomial."""
        return 35*tau**4 - 84*tau**5 + 70*tau**6 - 20*tau**7

    def _transition(self, y: np.ndarray, y_start: float, x_start: float,
                    H: float, L: float, curve: Callable[[np.ndarray], np.ndarray],
                    direction: int = 1) -> np.ndarray:
        """Generate X(y) for one transition. Works for both lead (+y) and trail (−y)."""
        if L <= 0:
            raise ValueError("Transition length L must be positive.")

        # tau always progresses from 0 → 1 along the path direction
        tau = direction * (y - y_start) / L
        s_norm = np.zeros_like(y, dtype=float)
        mask = (tau >= 0) & (tau <= 1)
        s_norm[mask] = curve(np.clip(tau[mask], 0.0, 1.0))

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
                y_end = y_current + L
                y_seg = np.linspace(y_current, y_end, int(L / dy) + 1)
                x_seg = self._transition(y_seg, y_current, x_current, H, L, curve_func, direction=1)
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
                y_end = y_current - L
                y_seg = np.linspace(y_current, y_end, int(L / dy) + 1)
                x_seg = self._transition(y_seg, y_current, x_current, H, L, curve_func, direction=-1)
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
