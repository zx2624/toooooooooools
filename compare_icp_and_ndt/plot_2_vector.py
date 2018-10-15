#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

# ls = [l.split() for l in open(sys.argv[1])]
# x_s = map(float, [l[1] for l in ls])
# y_s = map(float, [l[2] for l in ls])
# yaw_s = map(float, [l[6] for l in ls])
# yaw_cov = map(float, [l[12] for l in ls])
x_good = []
y_good = []
yaw_good = []

with open(sys.argv[1]) as f:
	for l in f:
		line = l.split()
		x_good.append(float(line[1]))
		y_good.append(float(line[2]))
		yaw_good.append(float(line[6]))

cos_good = np.cos(yaw_good)
sin_good = np.sin(yaw_good)

# plt.quiver(x_s, y_s, cos_s, sin_s, units='width', color='r', label="odom")
plt.quiver(x_good, y_good, cos_good, sin_good, scale=4, scale_units='inches', color='r', label="icp")

plt.axis('equal')
plt.legend()
plt.show()
