#!/usr/bin/env python

import sys
import argparse
from fnmatch import fnmatchcase
import os
from rosbag import Bag

file_path = sys.argv[1]
print(file_path)
files = os.listdir(file_path)
files = files + (os.listdir(file_path+'/0bag'))
print(files)
for file1 in files:
    if file1[-4:] != '.bag':
        continue
    for file2 in files:
        if file2[-4:] != '.bag':
            continue
        
        if file1[0:-4] == file2[0:-11]:
            print('==============================')
            print('dealing ' + \
                file1 + ' ' + file2)
            file1_name = file_path + '/0bag/' + file1
            file2_name = file_path + '/' + file2
            fileout_name = file_path + '/tmp/' + file1.split('.')[0] + 'tmp.bag'
            com = 'python merge_bag.py ' + fileout_name + ' ' + file1_name + ' '  + file2_name
            print(com)
            os.system(com)