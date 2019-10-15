#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt
import os


path = os.path.normpath(sys.argv[1] + '/') 
point = [13260.945, -957.605, -13.297]
x0 = 440804 + point[0]
y0 = 4425720 + point[1]
p0 = np.array([x0, y0])
# print('ls  ' + path)
files = os.popen('find  ' + path + ' -name *.txt').read().splitlines()
for file in files:
    if file[-4:] == '.txt':
        ls = [l.split() for l in open(file)]
        xs = map(float, [l[1] for l in ls])
        ys = map(float, [l[2] for l in ls])
        for i in range(0, len(xs)):
            p1 = np.array([xs[i], ys[i]])
            dis = np.linalg.norm(p0 - p1)
            if dis < 100:
                print(file.split('/'))[-1]
                break


        
# print(files)

# ls = [l.split() for l in open(sys.argv[1])]
# xs = map(float, [l[1] for l in ls])
# ys = map(float, [l[2] for l in ls])



# plt.scatter(xs, ys, color = 'red', label="odom")
# # plt.scatter(xs_new, ys_new, color = 'blue', label="loam")
# # plt.plot(xs, ys, 'g', label="carto")
# plt.axis('equal')
# plt.legend()
# plt.show()
