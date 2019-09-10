#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

first_dim = input("choose the x column: ")
second_dim = input("choose the y column: ")
def factor(x):
    x_ = float(x)
    if second_dim >= 7:
        return x_  * 100 
    if second_dim >=4 and second_dim <=6:
        return x_  * 100 * 57.32 #degree
    if second_dim <=3:
        return x_

ls = [l.split() for l in open(sys.argv[1])]
xs = map(float, [l[first_dim] for l in ls])
ys = map(factor, [l[second_dim] for l in ls])

ls = [l.split() for l in open(sys.argv[2])]
xs_2 = map(float, [l[first_dim] for l in ls])
ys_2 = map(factor, [l[second_dim] for l in ls])
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

plt.scatter(xs, ys, color = 'red', label="odom")
plt.scatter(xs_2, ys_2, color = 'green', label="odom")
# plt.scatter(xs_new, ys_new, color = 'blue', label="loam")
# plt.plot(xs, ys, 'g', label="carto")
plt.axis('equal')
plt.legend()
plt.show()
