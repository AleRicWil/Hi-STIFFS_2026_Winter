import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



path = r'Hi-STIFFS_2026_Winter\Raw Data\2026-07-02\2026-07-02_162744_01_IMU.csv'

df = pd.read_csv(path, skiprows= 6)

time = df['Time (sec)'].to_numpy()
time_sort = np.sort(time)

dt = np.diff(time)
dt_sort = np.diff(time)

time = time[0:-1]
time_sort = time_sort[0:-1]

plt.figure()
plt.scatter(time, dt, s=1)
plt.xlabel('Timestamp (s)')
plt.ylabel('dt (s)')

plt.figure()
plt.scatter(time_sort, dt_sort, s=1)
plt.xlabel('Timestamp (s)')
plt.ylabel('dt (s)')

plt.figure()
plt.hist(dt, bins=3000, density=True)

plt.show()