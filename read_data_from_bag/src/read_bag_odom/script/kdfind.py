#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt
import os
from sklearn.neighbors import KDTree


def float_list(str):
    num = []
    for num_str in str:
        num.append(float(num_str))
    return num


paths = []
paths.append(os.path.normpath(sys.argv[1] + '/'))
paths.append(os.path.normpath(sys.argv[2] + '/'))
points = []
infos = []
for path in paths:
    print(path)
    files = os.popen('find  ' + path + ' -name \*.txt').read().splitlines()
    points_one_day = []
    info_one_day = []
    for file in files:
        if file[-4:] == '.txt':
            ls = [l.split() for l in open(file)]
            info_one_day = info_one_day + [l for l in open(file)]
            # print([l[3:6] for l in ls])
            points_one_day = points_one_day + \
                ([float_list(l[1:4]) for l in ls])
    points.append(points_one_day)
    infos.append(info_one_day)

p0 = points[0]
p1 = points[1]
info0 = infos[0]
info1 = infos[1]
p1_array = np.array(p1)
kdtree = KDTree(p1_array, leaf_size=1)
x_offset = 440804.0
y_offset = 4425720.0
for p in p0:
    # print(p)
    dist, ind = kdtree.query(p, k=1)
    p_match = p1[ind[0][0]]
    # print(p_match)
    if abs(p[2] - p_match[2]) > 0.5:  # 
        print('------------')
        print(str(p[0]-x_offset) + ' ' +
              str(p[1] - y_offset) + ' ' + str(p[2]))
        info_line = map(float, [l for l in info1[ind[0][0]].split()])
        print(str(info_line[1]-x_offset) + ' ' +
              str(info_line[2] - y_offset) + ' ' + str(info_line[7]) + ' ' + str(info_line[8]) + ' ' + str(info_line[9]))

    # print(ind)

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
