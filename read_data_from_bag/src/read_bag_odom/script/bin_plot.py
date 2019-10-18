#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt
import os


path = os.path.normpath(sys.argv[1] + '/') 
# print('ls  ' + path)
data = []
files = os.popen('find  ' + path + ' -name \*.txt').read().splitlines()
for file in files:
    if file[-4:] == '.txt':
        ls = [l.split() for l in open(file)]
        xs = map(float, [l[9] for l in ls])
        data  = data + xs
print(len(data))
data = [i for i in data if i < 1]
print(len(data))
plt.hist(data, bins=1000, density=0, facecolor="blue", edgecolor="black", alpha=0.7)
plt.show()

        
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
