#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

# ls = [l.split() for l in open(sys.argv[1])]
# xs = map(float, [l[1] for l in ls])
# ys = map(float, [l[2] for l in ls])

x_good = []
y_good = []
x_bad = []
y_bad = []

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
		if(cov_x > 20 or cov_y > 20 or cov_z > 200) :
		  x_bad.append(float(line[1]))
		  y_bad.append(float(line[2]))
		else :
			x_good.append(float(line[1]))
			y_good.append(float(line[2]))

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
print 'good gnss {}, bag gnss {}'.format(len(x_good), len(x_bad))
plt.scatter(x_good, y_good, color = 'red', label="good")
plt.scatter(x_bad, y_bad, color = 'blue', label="bad")
# plt.plot(xs, ys, 'g', label="carto")
plt.axis('equal')
plt.legend()
plt.show()
