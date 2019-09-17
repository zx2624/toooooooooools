#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

ls = [l.split() for l in open(sys.argv[1])]
x_s = map(float, [l[1] for l in ls])
y_s = map(float, [l[2] for l in ls])
z_s =  map(float, [l[3] for l in ls])

roll_s = map(float, [l[4] for l in ls])
pitch_s = map(float, [l[5] for l in ls])
yaw_s = map(float, [l[6] for l in ls])


u = np.cos(yaw_s) * np.cos(pitch_s) 
v = np.sin(yaw_s) * np.cos(pitch_s) 
w = -np.sin(pitch_s)


fig = plt.figure()
ax = fig.gca(projection='3d')

# plt.quiver(x_s, y_s, cos_s, sin_s, units='width', color='r', label="loam")
# ax.quiver(x_s, y_s, z_s, u, v, w, length = 0.1)
ax.quiver(x_s, y_s, z_s, u, v, w, length = 0.1)

plt.show()
