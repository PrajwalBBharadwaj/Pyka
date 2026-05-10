import matplotlib.pyplot as plt
import numpy as np

data_1 = np.loadtxt('alt_estimate1.txt')
data_2 = np.loadtxt('alt_estimate2.txt')

fig, axs = plt.subplots(2, 1, figsize=(18, 12))

# Common colors
GPS_COLOR = 'blue'
ALT1_COLOR = 'green'
ALT2_COLOR = 'purple'
FUSION_COLOR = 'red'

# ---------- Plot 1 ----------
axs[0].plot(
    data_1[:, 0], data_1[:, 1],
    label='GPS Altitude',
    color=GPS_COLOR,
    linewidth=1.5
)

axs[0].plot(
    data_1[:, 0], data_1[:, 2],
    label='Altimeter 1 Altitude',
    color=ALT1_COLOR,
    linewidth=1.5
)

axs[0].plot(
    data_1[:, 0], data_1[:, 3],
    label='Altimeter 2 Altitude',
    color=ALT2_COLOR,
    linewidth=1.5
)

axs[0].plot(
    data_1[:, 0], data_1[:, 4],
    label='Sensor Fusion Estimate',
    color=FUSION_COLOR,
    linewidth=2.5
)

axs[0].set_title('Altitude Estimate 1')
axs[0].set_ylabel('Altitude')
axs[0].legend()
axs[0].grid(True)
axs[0].set_xlim(data_1[:, 0].min(), data_1[:, 0].max())

# ---------- Plot 2 ----------
axs[1].plot(
    data_2[:, 0], data_2[:, 1],
    label='GPS Altitude',
    color=GPS_COLOR,
    linewidth=1.5
)

axs[1].plot(
    data_2[:, 0], data_2[:, 2],
    label='Altimeter 1 Altitude',
    color=ALT1_COLOR,
    linewidth=1.5
)

axs[1].plot(
    data_2[:, 0], data_2[:, 3],
    label='Altimeter 2 Altitude',
    color=ALT2_COLOR,
    linewidth=1.5
)

axs[1].plot(
    data_2[:, 0], data_2[:, 4],
    label='Sensor Fusion Estimate',
    color=FUSION_COLOR,
    linewidth=2.5
)

axs[1].set_title('Altitude Estimate 2')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Altitude')
axs[1].legend()
axs[1].grid(True)
axs[1].set_xlim(data_2[:, 0].min(), data_2[:, 0].max())

plt.tight_layout()
plt.show()
