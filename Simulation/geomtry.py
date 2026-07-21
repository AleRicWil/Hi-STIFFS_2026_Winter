import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.signal import savgol_filter

# ================== CONFIGURATION ==================
path = r"C:\Users\Alex R. Williams\Desktop\v3.2.csv"

# Smoothing parameters (tune as needed)
window_length = 31      # Must be odd and greater than polyorder; larger = more smoothing
polyorder = 1           # Polynomial degree
# ===================================================

# Load data
df = pd.read_csv(path)
x_orig = df['X(mm)'].values
y_orig = df['Y(mm)'].values

# Compute original mean spacing (for reference and uniform grid)
delta_y_orig = np.mean(np.diff(y_orig))
print(f"Original mean Δy: {delta_y_orig:.4f} mm")
print(f"Number of original points: {len(y_orig)}")

# Create uniform longitudinal (y) grid using original mean spacing
y_min = y_orig.min()
y_max = y_orig.max()
y_new = np.arange(y_min, y_max + delta_y_orig / 2, delta_y_orig)

# Linear interpolation of lateral position x onto the uniform y grid
x_new = np.interp(y_new, y_orig, x_orig)

print(f"Resampled to {len(y_new)} points with uniform Δy = {delta_y_orig:.4f} mm")

# Compute smoothed derivatives on the uniform grid
vx = savgol_filter(x_new, window_length, 1, deriv=1, delta=delta_y_orig)
ax = savgol_filter(vx, window_length, 1, deriv=1, delta=delta_y_orig)
jx = savgol_filter(ax, window_length, 1, deriv=1, delta=delta_y_orig)

# ================== PLOTTING ==================
fig, axes = plt.subplots(4, 1, sharex=True)

axes[0].scatter(y_new, x_new, s=1)
axes[0].axhline(0, linewidth=1, c='k')
axes[0].axvline(0, linewidth=1, c='k')
axes[0].set_ylabel('Lateral Position (mm)')

axes[1].scatter(y_new, vx, s=1)
axes[1].axhline(0, linewidth=1, c='k')
axes[1].axvline(0, linewidth=1, c='k')
axes[1].set_ylabel('Velocity')

axes[2].scatter(y_new, ax, s=1)
axes[2].axhline(0, linewidth=1, c='k')
axes[2].axvline(0, linewidth=1, c='k')
axes[2].set_ylabel('Acceleration')

axes[3].scatter(y_new, jx, s=1)
axes[3].axhline(0, linewidth=1, c='k')
axes[3].axvline(0, linewidth=1, c='k')
axes[3].set_ylabel('Jerk')
axes[3].set_xlabel('Longitudinal Position (mm)')

axes[0].axis('equal')

for ax in axes:
    ax.xaxis.set_inverted(True)

plt.tight_layout()

plt.show()