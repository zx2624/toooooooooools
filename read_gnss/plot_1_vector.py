#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

ls = [l.split() for l in open(sys.argv[1])]
x_s = map(float, [l[1] for l in ls])
y_s = map(float, [l[2] for l in ls])
yaw_s = map(float, [l[6] for l in ls])


cos_s = np.cos(yaw_s)
sin_s = np.sin(yaw_s)

plt.quiver(x_s, y_s, cos_s, sin_s, units='width', color='r', label="loam")

plt.axis('equal')
plt.legend()
plt.show()
