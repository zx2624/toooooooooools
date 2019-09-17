#!/usr/bin/env python2.7
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import glob

x_c = 440804 + 4121
y_c = 4425720 - 8273
col = input("choose the colum: ")
def rad(x , y):
    x_ = float(x) - x_c
    y_ = float(y) - y_c
    return math.atan2(y_, x_) * 57.32

def factor(x):
    x_ = float(x)
    if col >= 7:
        return x_  * 100 
    if col >=4 and col <=6:
        return x_  * 10 * 57.32 #degree
    if col <=3:
        return x_

directory1 = sys.argv[1]
directory2 = sys.argv[2]
files1 = glob.glob(directory1 + '/*.txt')
files2 = glob.glob(directory2 + '/*.txt')

rad_s_1=[]
rad_s_2=[]
things_1=[]
things_2=[]

for file in files1:
    ls = [l.split() for l in open(file)]
    rads = map(rad, [l[1] for l in ls], [l[2] for l in ls])
    rad_s_1 = rad_s_1 + rads
    things = map(factor, [l[col] for l in ls])
    things_1 = things + things_1

for file in files2:
    ls = [l.split() for l in open(file)]
    rads = map(rad, [l[1] for l in ls], [l[2] for l in ls])
    rad_s_2 = rad_s_2 + rads
    things = map(factor, [l[col] for l in ls])
    things_2 = things + things_2


plt.scatter(rad_s_1, things_1, color = 'red', label="odom")
plt.scatter(rad_s_2, things_2, color = 'green', label="odom")
# plt.scatter(y_s_2, pitch_2, color = 'green', label="odom")

# plt.scatter(xs_new, ys_new, color = 'blue', label="loam")
# plt.plot(xs, ys, 'g', label="carto")
plt.axis('equal')
plt.legend()
plt.show()