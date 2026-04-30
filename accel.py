import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


filename = r'Hi-STIFFS_2026_Winter\retry0.csv'

df = pd.read_csv(filename)
print(df)


fig, ax = plt.subplots(3,2, sharex=True)
ax[0,0].scatter(df['time (microseconds)'], df['ax'], s=0.5, label='ax')
ax[0,0].legend()
ax[1,0].scatter(df['time (microseconds)'], df['ay'], s=0.5, label='ay')
ax[1,0].legend()
ax[2,0].scatter(df['time (microseconds)'], df['az'], s=0.5, label='az')
ax[2,0].legend()

ax[0,1].scatter(df['time (microseconds)'], df['gx'], s=0.5, label='gx')
ax[0,1].legend()
ax[1,1].scatter(df['time (microseconds)'], df['gy'], s=0.5, label='gy')
ax[1,1].legend()
ax[2,1].scatter(df['time (microseconds)'], df['gz'], s=0.5, label='gz')
ax[2,1].legend()

plt.tight_layout()
plt.show()