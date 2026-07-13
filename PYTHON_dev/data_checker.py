import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.integrate import cumulative_simpson



path = r'Hi-STIFFS_2026_Winter\Raw Data\2026-07-07\2026-07-07_150357_01_IMU.csv'
path2 = r'Hi-STIFFS_2026_Winter\Raw Data\2026-07-07\2026-07-07_140435_01_IMU.csv'

df = pd.read_csv(path, skiprows= 6)

time = df['Time (sec)'].to_numpy()

gx = df['Gyro X'].to_numpy()
gy = df['Gyro Y'].to_numpy()
gz = df['Gyro Z'].to_numpy()

ax = df['Accel X'].to_numpy()
ay = df['Accel Y'].to_numpy()
az = df['Accel Z'].to_numpy()

mx = df['Mag X'].to_numpy()
my = df['Mag Y'].to_numpy()
mz = df['Mag Z'].to_numpy()

wl = 9
po = 4
gx_sg = savgol_filter(gx, window_length=wl, polyorder=po)
gy_sg = savgol_filter(gy, window_length=wl, polyorder=po)
gz_sg = savgol_filter(gz, window_length=wl, polyorder=po)
ax_sg = savgol_filter(ax, window_length=wl, polyorder=po)
ay_sg = savgol_filter(ay, window_length=wl, polyorder=po)
az_sg = savgol_filter(az, window_length=wl, polyorder=po)

fig, axes = plt.subplots(3,3, sharex=True)

axes[0,0].scatter(time, gx, s=1)
axes[0,0].plot(time, gx_sg, linewidth=1, c='orange')
axes[0,0].set_ylabel('Gyro X')
axes[1,0].scatter(time, gy, s=1)
axes[1,0].plot(time, gy_sg, linewidth=1, c='orange')
axes[1,0].set_ylabel('Gyro Y')
axes[2,0].scatter(time, gz, s=1)
axes[2,0].plot(time, gz_sg, linewidth=1, c='orange')
axes[2,0].set_ylabel('Gyro Z')
axes[2,0].set_xlabel('Time')

axes[0,1].scatter(time, ax, s=1)
axes[0,1].plot(time, ax_sg, linewidth=1, c='orange')
axes[0,1].set_ylabel('Accel X')
axes[1,1].scatter(time, ay, s=1)
axes[1,1].plot(time, ay_sg, linewidth=1, c='orange')
axes[1,1].set_ylabel('Accel Y')
axes[2,1].scatter(time, az, s=1)
axes[2,1].plot(time, az_sg, linewidth=1, c='orange')
axes[2,1].set_ylabel('Accel Z')
axes[2,1].set_xlabel('Time')

axes[0,2].scatter(time, mx, s=1)
axes[0,2].set_ylabel('Mag X')
axes[1,2].scatter(time, my, s=1)
axes[1,2].set_ylabel('Mag Y')
axes[2,2].scatter(time, mz, s=1)
axes[2,2].set_ylabel('Mag Z')
axes[2,2].set_xlabel('Time')

fig.subplots_adjust(
    left=0.055,   # Accommodates y-axis labels
    right=0.99,   # Minimal right margin
    top=0.98,     # Minimal top margin
    bottom=0.065, # Accommodates x-axis labels on bottom row
    hspace=0.08,  # Vertical spacing between rows
    wspace=0.18   # Horizontal spacing between columns
)


vx = cumulative_simpson(ax_sg, x=time, initial=0)
vy = cumulative_simpson(ay_sg, x=time, initial=0)
vz = cumulative_simpson(az_sg, x=time, initial=0)

fig, axes = plt.subplots(3,1, sharex=True)

axes[0].scatter(time, vx, s=1)
axes[1].scatter(time, vy, s=1)
axes[2].scatter(time, vz, s=1)

fig.subplots_adjust(
    left=0.055,   # Accommodates y-axis labels
    right=0.99,   # Minimal right margin
    top=0.98,     # Minimal top margin
    bottom=0.065, # Accommodates x-axis labels on bottom row
    hspace=0.08,  # Vertical spacing between rows
    wspace=0.18   # Horizontal spacing between columns
)
plt.show()


# dt = np.diff(time)
# time = time[0:-1]

# plt.figure()
# plt.scatter(time, dt, s=1)
# plt.xlabel('Timestamp (s)')
# plt.ylabel('dt (s)')