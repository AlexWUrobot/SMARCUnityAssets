import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from mpl_toolkits.mplot3d import Axes3D

# read log.csv in the same folder
df = pd.read_csv(os.path.join(os.path.dirname(__file__), 'log.csv'))
t = df['t']
x_s1 = df['x_s1']
x_s2 = df['x_s2']
x_s3 = df['x_s3']
x_s_d1 = df['x_s_d1']
x_s_d2 = df['x_s_d2']
x_s_d3 = df['x_s_d3']

propellers_rpms1 = df['propellers_rpms1']
propellers_rpms2 = df['propellers_rpms2']
propellers_rpms3 = df['propellers_rpms3']
propellers_rpms4 = df['propellers_rpms4']

rollRad = df['rollRad']
pitchRad = df['pitchRad']
yawRad = df['yawRad']

v_s1 = df['v_s1']
v_s2 = df['v_s2']
v_s3 = df['v_s3']
v_s_d1 = df['v_s_d1']
v_s_d2 = df['v_s_d2']
v_s_d3 = df['v_s_d3']

insideCount = df['insideCount']

########################################## Position

plt.figure(1)
plt.subplot(3, 1, 1)
plt.plot(t, x_s1, label='x_s1')
plt.plot(t, x_s_d1, label='x_s_d1')
plt.ylabel("x-axis meter")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, x_s2, label='x_s2')
plt.plot(t, x_s_d2, label='x_s_d2')
plt.ylabel("y-axis meter")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, x_s3, label='x_s3')
plt.plot(t, x_s_d3, label='x_s_d3')
plt.ylabel("z-axis meter")
plt.legend()


plt.figure(2)
plt.subplot(3, 1, 1)
plt.plot(t, x_s1-x_s_d1, label='x_error')
plt.ylabel("meter")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, x_s2-x_s_d2, label='y_error')
plt.ylabel("meter")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, x_s3-x_s_d3, label='z_error')
plt.ylabel("meter")
plt.xlabel("time (s)")
plt.legend()

########################################## Velocity

plt.figure(3)
plt.subplot(3, 1, 1)
plt.plot(t, v_s1, label='v_x')
plt.plot(t, v_s_d1, label='desired_v_x')
plt.ylabel("x-axis meter/sec")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, v_s2, label='v_y')
plt.plot(t, v_s_d2, label='desired_v_y')
plt.ylabel("y-axis meter/sec")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, v_s3, label='v_z')
plt.plot(t, v_s_d3, label='desired_v_z')
plt.ylabel("z-axis meter/sec")
plt.xlabel("time (s)")
plt.legend()

########################################## RPM

plt.figure(4)
plt.subplot(4, 1, 1)
plt.plot(t, propellers_rpms1, label='rotor 1')
plt.ylabel("rpm")
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(t, propellers_rpms2, label='rotor 2')
plt.ylabel("rpm")
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(t, propellers_rpms3, label='rotor 3')
plt.ylabel("rpm")
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(t, propellers_rpms4, label='rotor 4')
plt.ylabel("rpm")
plt.xlabel("time (s)")
plt.legend()

########################################## Attitude and Wind

plt.figure(5)
plt.subplot(4, 1, 1)
plt.plot(t, rollRad, label='roll')
plt.ylabel("Degree")
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(t, pitchRad, label='pitch')
plt.ylabel("Degree")
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(t, yawRad, label='yaw')
plt.ylabel("Degree")
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(t, insideCount, label='Rope inside wind')
plt.ylabel("Count")
plt.xlabel("time (s)")
plt.legend()


# Create a 3D plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot the trajectories
ax.plot(x_s1, x_s2, x_s3, color='blue', label='Source Trajectory')
#ax.plot(x_s_d1, x_s_d2, x_s_d3, color='red', label='Destination Trajectory', marker = 'o')
ax.scatter(x_s_d1, x_s_d2, x_s_d3, c='red', label='Destination Points')

# # Set labels and title
ax.set_title('3D Plot of Source and Destination Points')
ax.set_xlabel('X Axis (m)')
ax.set_ylabel('Y Axis (m)')
ax.set_zlabel('Z Axis (m)')

# Show legend
ax.legend()


plt.show()