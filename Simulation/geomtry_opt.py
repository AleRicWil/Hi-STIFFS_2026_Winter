import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import csv
from typing import List, Tuple, Callable

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

    def plot_kinematics(self, vy: float = None):
        """Plot displacement, velocity, acceleration, and jerk for engineering review."""
        if vy is None:
            vy = self.vy
        if self.y is None or self.x is None:
            print("No profile built yet. Call build_profile() first.")
            return

        # Numerical derivatives + Savitzky-Golay smoothing
        window_length = 31
        polyorder = 1
        
        # dx_dy = np.gradient(self.x, self.y)
        # d2x_dy2 = np.gradient(dx_dy, self.y)
        # d3x_dy3 = np.gradient(d2x_dy2, self.y)
        
        dx_dy   = savgol_filter(self.x,  window_length, polyorder, deriv=1, delta=self.delta_y)
        d2x_dy2 = savgol_filter(dx_dy,   window_length, polyorder, deriv=1, delta=self.delta_y)
        d3x_dy3 = savgol_filter(d2x_dy2, window_length, polyorder, deriv=1, delta=self.delta_y)

        vx = dx_dy * vy
        ax = d2x_dy2 * vy**2
        jx = d3x_dy3 * vy**3

        pos_y_mask = self.y > 0.0
        x_pos = self.x[pos_y_mask]
        # Index of the point in the y < 0 region whose x is closest to -0.2
        # local_idx = np.argmin(np.abs(x_pos + 0.02))
        # global_idx = np.where(pos_y_mask)[0][local_idx]
        # ax_at_target = ax[global_idx]
        # x_at_target = self.x[global_idx]
        # y_at_target = self.y[global_idx]
        # print("\n=== Cam kinematics at target location ===")
        # print(f"Nearest point to x = -0.020m on leading half")
        # print(f"  Index (global) : {global_idx}")
        # print(f"  x              : {x_at_target:.6f} m")
        # print(f"  y              : {y_at_target:.6f} m")
        # print(f"  ax             : {ax_at_target:.6f} m/s²")

        # neg_y_mask = self.y < 0.0
        # x_neg = self.x[neg_y_mask]
        # # Index of the point in the y < 0 region whose x is closest to -0.2
        # local_idx = np.argmin(np.abs(x_neg + 0.02))
        # global_idx = np.where(neg_y_mask)[0][local_idx]
        # ax_at_target = ax[global_idx]
        # x_at_target = self.x[global_idx]
        # y_at_target = self.y[global_idx]
        # print(f"Nearest point to x = -0.020m on trailing half")
        # print(f"  Index (global) : {global_idx}")
        # print(f"  x              : {x_at_target:.6f} m")
        # print(f"  y              : {y_at_target:.6f} m")
        # print(f"  ax             : {ax_at_target:.6f} m/s²")
        # print("=========================================\n")

        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

        axes[0].plot(self.y, self.x, 'b-', linewidth=1.5)
        axes[0].set_ylabel('Lateral Position (m)')
        axes[1].plot(self.y, vx, 'r-', linewidth=1.5)
        axes[1].set_ylabel('Vx (m/s)')
        axes[2].plot(self.y, ax, 'g-', linewidth=1.5)
        axes[2].set_ylabel('Ax (m/s²)')
        # axes[3].plot(self.y, jx, 'm-', linewidth=1.5)
        # axes[3].set_ylabel('Jx (m/s³)')
        axes[2].set_xlabel('Longitudinal Position (m)')

        for axis in axes:
            axis.axhline(0, linewidth=0.8, color='k', alpha=0.6)
            axis.axvline(0, linewidth=0.8, color='k', alpha=0.6)
            axis.grid(True, alpha=0.3)
            axis.xaxis.set_inverted(True)

        axes[0].axis('equal')
        fig.suptitle(f'Cam Kinematics — Forward Speed: {vy:.3f} m/s ({vy*2.23694:.2f} mph)')
        plt.tight_layout()
        plt.get_current_fig_manager().window.move(0, 0)
        plt.show()
        
        print(f"Peak lateral acceleration: {np.max(np.abs(ax)):.2f} m/s²")
        print(f"Peak jerk: {np.max(np.abs(jx)):.2f} m/s³")

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

# Example usage – tune these to your machine
# filepath = r"C:\Users\Alex R. Williams\Desktop\two_straights.csv"
contour = LinearCamContour(vy=0.447)  # m/s forward speed

segments = [
    {'dir': 'lead', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.06},     # length of ICB sensor from 0 of its measurement region + its body
    {'dir': 'lead', 'type': 'dwell', 'length': 0.100, 'x': 0.06},           # recovery zone before contacting middle sensor
    {'dir': 'lead', 'type': 'rise',  'length': 0.263, 'delta_x': -0.06},    # leading transition, capped at d2x/dy2=(1 m/s^2) @ dy=(1 mph)
    {'dir': 'lead', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.00},     # leading ICB sensor measurement region + body
    {'dir': 'lead', 'type': 'dwell', 'length': 0.050, 'x': -0.000},         # recovery zone before contacting leading sensor
    {'dir': 'lead', 'type': 'rise',  'length': 0.677, 'delta_x': -0.401},   # fall to center of two-sided probe, capped at d2x/dy2=(2 m/s^2) @ dy=(1 mph) @ x=(-0.020 m)
    # {'dir': 'lead', 'type': 'dwell', 'length': 0.001, 'x': -0.401},

    {'dir': 'trail', 'type': 'dwell', 'length': 0.045, 'x': 0.06},          # trailing allowance for recessed portion of ICB sensor
    {'dir': 'trail', 'type': 'rise',  'length': 0.263, 'delta_x': -0.06},   # trailing transition, capped at d2x/dy2=(1 m/s^2) @ dy=(1 mph)
    {'dir': 'trail', 'type': 'dwell', 'length': 0.050, 'x': 0.00},          # recovery zone before contacting trailing sensor
    {'dir': 'trail', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.00},    # length of trailing ICB sensor measurement region + body     
    {'dir': 'trail', 'type': 'dwell', 'length': 0.053, 'x': 0.00},          # trailing allowance for recessed portion of ICB sensor
    {'dir': 'trail', 'type': 'rise',  'length': 0.390, 'delta_x': -0.401}   # fall to center of two-sided probe, capped at d2x/dy2=(5 m/s^2) @ dy=(1 mph) @ x=(-0.020 m)
    # {'dir': 'trail', 'type': 'dwell', 'length': 0.001, 'x': -0.401}
]

y, x = contour.build_profile(segments, curve_func=contour.poly_345)

contour.plot_kinematics(vy=contour.vy)
# contour.export_profile(path='new_profile.csv')


# def find_min_L(H: float, vy: float, a_max_allow: float = 5.0, curve=LinearCamContour.poly_345):
#     """Binary search for shortest L that keeps a_x <= a_max_allow"""
#     # Use normalized Ca from PDF (or compute numerically)
#     # For 3-4-5: Ca ≈ 5.77 → a_max = 5.77 * H / L**2 * vy**2
#     L_low, L_high = 0.01, 1.0
#     while L_high - L_low > 1e-4:
#         L_mid = (L_low + L_high) / 2
#         a_peak = 5.77 * H * vy**2 / L_mid**2   # replace with actual curve calc
#         if a_peak > a_max_allow:
#             L_low = L_mid
#         else:
#             L_high = L_mid
#     return L_low