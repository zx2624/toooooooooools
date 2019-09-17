#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ls = [l.split() for l in open(sys.argv[1])]
# xs = map(float, [l[1] for l in ls])
# ys = map(float, [l[2] for l in ls])

ls = [l.split() for l in open(sys.argv[1])]
xs_new = map(float, [l[2] for l in ls])
ys_new = map(float, [l[3] for l in ls])
zs_new = map(float, [l[4] for l in ls])

# ls = [l.split() for l in open(sys.argv[3])]
# xs = map(float, [l[1] for l in ls])
# ys = map(float, [l[2] for l in ls])

#a,b=xs[0],ys[0]
#ga,gb=gxs[0],gys[0]

#for i in range(len(xs)):
    #xs[i] = xs[i]-a
    #ys[i] = ys[i]-b

#for i in range(len(gxs)):
    #gxs[i] =gxs[i]-ga
    #gys[i] = gys[i]-gb

# for i in range(len(xs_new)):
		# # xs_new[i] = xs_new[i]
		# ys_new[i] = ys_new[i] + 10

# plt.scatter(xs, ys, color = 'red', label="gnss")
fig = plt.figure()
ax = Axes3D(fig)

# plt.scatter(xs_new, ys_new, color = 'blue', label="loam")
ax.scatter(xs_new, ys_new, zs_new, c = 'r', marker='.')
# plt.plot(xs, ys, 'g', label="carto")
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()
