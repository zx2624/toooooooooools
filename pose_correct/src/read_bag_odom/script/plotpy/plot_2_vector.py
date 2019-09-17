#!/usr/bin/env python2.7
import sys
import numpy as np
import matplotlib.pyplot as plt

ls = [l.split() for l in open(sys.argv[1])]
x_s = map(float, [l[2] for l in ls])
y_s = map(float, [l[3] for l in ls])
yaw_s = map(float, [l[5] for l in ls])
print('=======')
ls = [l.split() for l in open(sys.argv[2])]
x_s_2 = map(float, [l[2] for l in ls])
y_s_2 = map(float, [l[3] for l in ls])
yaw_s_2 = map(float, [l[5] for l in ls])

# ls = [l.split() for l in open(sys.argv[2])]
# xs_new = map(float, [l[7] for l in ls])
# ys_new = map(float, [l[11] for l in ls])

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

# value = np.asarray(xs_good)
# plt.hist(value * 100 )
# plt.title("Gaussian Histogram")
# plt.xlabel("Value")
# plt.ylabel("Frequency")


cos_s = np.cos(yaw_s)
sin_s = np.sin(yaw_s)

plt.quiver(x_s, y_s, cos_s, sin_s, units='width', color='r', label="1")

cos_s_2 = np.cos(yaw_s_2)
sin_s_2 = np.sin(yaw_s_2)

plt.quiver(x_s_2, y_s_2, cos_s_2, sin_s_2, units='width',color="g", label="2")

# plt.scatter(xs, ys, color = 'red', label="gnss")

# plt.scatter(xs_new, ys_new, color = 'blue', label="loam")
# plt.plot(xs, ys, 'g', label="carto")
plt.axis('equal')
plt.legend()
plt.show()
