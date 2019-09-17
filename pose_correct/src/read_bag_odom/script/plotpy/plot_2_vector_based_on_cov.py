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
		cov = float(line[12])
		if(cov > 5) :
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

x_good_2 = []
y_good_2 = []
yaw_good_2 = []
x_bad_2 = []
y_bad_2 = []
yaw_bad_2 = []

with open(sys.argv[2]) as f:
	for l in f:
		line = l.split()
		cov = float(line[12])
		if(cov > 5) :
			x_bad_2.append(float(line[1]))
			y_bad_2.append(float(line[2]))
			yaw_bad_2.append(float(line[6]))
		else :
			x_good_2.append(float(line[1]))
			y_good_2.append(float(line[2]))
			yaw_good_2.append(float(line[6]))

cos_good_2 = np.cos(yaw_good_2)
sin_good_2 = np.sin(yaw_good_2)

cos_bad_2 = np.cos(yaw_bad_2)
sin_bad_2 = np.sin(yaw_bad_2)

# plt.quiver(x_s, y_s, cos_s, sin_s, units='width', color='r', label="odom")
plt.quiver(x_good, y_good, cos_good, sin_good, scale=4, scale_units='inches',
		color='r', label="raw_odom_good")
plt.quiver(x_bad, y_bad, cos_bad, sin_bad, scale=4, scale_units='inches',
		color='b', label="raw_odom_bad")

plt.quiver(x_good_2, y_good_2, cos_good_2, sin_good_2, scale=4, scale_units='inches',
		color='g', label="new_odom_good")
plt.quiver(x_bad_2, y_bad_2, cos_bad_2, sin_bad_2, scale=4, scale_units='inches', 
		color='k', label="new_odom_bad")

plt.axis('equal')
plt.legend()
plt.show()
