#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

x_1 = []
y_1 = []
yaw_1 = []

with open(sys.argv[1]) as f:
	for l in f:
		line = l.split()
		x_1.append(float(line[1]))
		y_1.append(float(line[2]))
		yaw_1.append(float(line[6]))

cos_1 = np.cos(yaw_1)
sin_1 = np.sin(yaw_1)

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

x_3 = []
y_3 = []
yaw_3 = []

with open(sys.argv[3]) as f:
	for l in f:
		line = l.split()
		x_3.append(float(line[1]))
		y_3.append(float(line[2]))
		yaw_3.append(float(line[6]))

cos_3 = np.cos(yaw_3)
sin_3 = np.sin(yaw_3)

# plt.quiver(x_s, y_s, cos_s, sin_s, units='width', color='r', label="odom")
plt.quiver(x_1, y_1, cos_1, sin_1, scale=4, scale_units='inches', color='r', label="sicp")
plt.quiver(x_2, y_2, cos_2, sin_2, scale=4, scale_units='inches', color='g', label="gicp")
plt.quiver(x_3, y_3, cos_3, sin_3, scale=4, scale_units='inches', color='b', label="loam")

plt.axis('equal')
plt.legend()
plt.show()
