#! /usr/bin/env python

# Prerequisite
#(1) sudo apt-get install chromium-chromedriver
#(2) sudo pip install selenium
#(3) sudo pip install matplotlib
#(4) sudo pip install gmplot

from fnmatch import fnmatchcase
import rosbag
import sys
import getopt
import glob
import os
import time
from rosbag import Bag
# from selenium import webdriver
# import geodesy.utm as utm
import matplotlib.pyplot as plt
# from gmplot import gmplot
import subprocess
import time


path = sys.argv[1]
files = os.listdir(path)
outpath = os.path.normpath(path + '/' + 'out')
if not os.path.exists(outpath):
    print (outpath + ' does not exist!\n So we create it!')
    os.makedirs(outpath)
print(outpath)
for file in files:
    if file[-4:] == '.bag':
        big_file_path = os.path.normpath(path + '/' + file)
        print('dealing bag : ' + big_file_path)
        startimes = []
        endtimes = []
        bagprefix = file.split('.')[0]
        small_bag_path = os.popen('find ' + path + ' -name ' + bagprefix + 'small_0.bag').read().splitlines()
        print('all small bags are ')
        print(small_bag_path)
        if len(small_bag_path) == 0:
            print('no smallbags, begin to continue')
            continue

        matchedtopics = []
        topics = '*'
        outbag = Bag(os.path.normpath(outpath + '/' + file), 'w')

        print("===begin to write and get small bag===")
        for small in small_bag_path:
            print('===========  ' + small + '  =============')
            bag = Bag(small, 'r')
            startimes.append(bag.get_start_time())
            endtimes.append(bag.get_end_time())
            for topic, msg, t in bag:
                if any(fnmatchcase(topic, pattern) for pattern in topics):
                    if not topic in matchedtopics:
                        matchedtopics.append(topic)
                    # print("Write msg " + str(t.to_sec()))
                    outbag.write(topic, msg, t)   
        startimes.sort()
        endtimes.sort()         
        startimes.append(sys.float_info.max)
        endtimes.append(sys.float_info.max)
        print("===begin to write big bag===")
        with Bag(big_file_path, 'r') as ib:
            t_last = 0.0
            i = 0
            flop = True
            for topic, msg, t in ib:
                if t.to_sec() - t_last > 300:
                    print(t.to_sec())
                    t_last = t.to_sec()
                write_big = True
                if t.to_sec() > startimes[i] and t.to_sec() < endtimes[i]:
                    flop = True
                    continue
                if flop and t.to_sec() >= endtimes[i]:
                    flop = False
                    print('pass one ^.^')
                    i = i + 1
                if any(fnmatchcase(topic, pattern) for pattern in topics):
                    if not topic in matchedtopics:
                        matchedtopics.append(topic)
                    # print("Write msg " + str(t.to_sec()))
                    outbag.write(topic, msg, t)
        outbag.close()