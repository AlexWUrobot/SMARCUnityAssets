import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

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
plt.xlabel("time")
plt.legend()

plt.figure(3)
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
plt.xlabel("time")
plt.legend()

plt.show()