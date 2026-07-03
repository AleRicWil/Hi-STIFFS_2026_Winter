import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



path = r'Hi-STIFFS_2026_Winter\Raw Data\2026-07-03\2026-07-03_121244_01_IMU.csv'

df = pd.read_csv(path, skiprows= 6)

time = df['Time (sec)'].to_numpy()
dt = np.diff(time)
time = time[0:-1]

plt.figure()
plt.scatter(time, dt, s=1)
plt.xlabel('Timestamp (s)')
plt.ylabel('dt (s)')

plt.show()