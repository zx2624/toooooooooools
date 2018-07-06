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
x_bad = []
y_bad = []
yaw_bad = []

with open(sys.argv[1]) as f:
	for l in f:
		line = l.split()
		cov_x = float(line[7])
		cov_y = float(line[8])
		cov_z = float(line[9])
		cov_roll = float(line[10])
		cov_pitch = float(line[11])
		cov_yaw = float(line[12])
#if(cov_x > 3 or cov_y > 3 or cov_z > 3 or cov_yaw > 3 or cov_roll > 3 or cov_pitch > 3) :
		if(cov_x > 20 or cov_y > 20 or cov_z > 10000000) :
		  x_bad.append(float(line[1]))
		  y_bad.append(float(line[2]))
		  yaw_bad.append(float(line[6]))
		else :
			x_good.append(float(line[1]))
			y_good.append(float(line[2]))
			yaw_good.append(float(line[6]))

cos_good = np.cos(yaw_good)
sin_good = np.sin(yaw_good)

cos_bad = np.cos(yaw_bad)
sin_bad = np.sin(yaw_bad)

# plt.quiver(x_s, y_s, cos_s, sin_s, units='width', color='r', label="odom")
plt.quiver(x_good, y_good, cos_good, sin_good, scale=4, scale_units='inches', color='r', label="odom_good")
plt.quiver(x_bad, y_bad, cos_bad, sin_bad, scale=4, scale_units='inches', color='b', label="odom_bad")

plt.axis('equal')
plt.legend()
plt.show()
