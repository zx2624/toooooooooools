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

x_2 = []
y_2 = []
yaw_2 = []

with open(sys.argv[2]) as f:
	for l in f:
		line = l.split()
		x_2.append(float(line[1]))
		y_2.append(float(line[2]))
		yaw_2.append(float(line[6]))

cos_2 = np.cos(yaw_2)
sin_2 = np.sin(yaw_2)

# plt.quiver(x_s, y_s, cos_s, sin_s, units='width', color='r', label="odom")
plt.quiver(x_good, y_good, cos_good, sin_good, scale=4, scale_units='inches', color='r', label="all")
plt.quiver(x_2, y_2, cos_2, sin_2, scale=4, scale_units='inches', color='g', label="label")

plt.axis('equal')
plt.legend()
plt.show()
