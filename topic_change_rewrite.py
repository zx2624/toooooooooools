#!/usr/bin/env python

import sys
import argparse
from fnmatch import fnmatchcase
import os
from rosbag import Bag

move_write_topic = '/pose_optimize/velodyne/odom/fusion' 
tmp_topic = '/pose_optimize/velodyne/odom/fusion_tmp1012'
input_topic = '/pose_optimize/velodyne/odom/fusion_1732'
# output_topic = '/pose_optimize/velodyne/odom/fusion'
file_path = sys.argv[1]
files = os.listdir(file_path)
for file in files:
    if file[-4:] == '.bag':
        print(file)
        file_in = os.path.normpath(file_path + '/' + file)
        file_out = os.path.normpath(file_path + '/' + 'tmp.bag')
        com_rename = 'rosrun rosbag topic_renamer.py ' \
            + move_write_topic + ' ' + file_in + ' ' \
                + tmp_topic + ' ' + file_out
        print('====================================')
        print('running command:  \n' + com_rename)
        os.system(com_rename)
        print('begin mv: ' + file)
        com_mv = 'mv ' + file_out + ' ' + file_in
        os.system(com_mv)
        # break
print('fist round done! ++++++++++++++++++++++++')

for file in files:
    if file[-4:] == '.bag':
        print(file)
        file_in = file_path + '/' + file
        file_out = file_path + '/' + 'tmp.bag'
        com_rename = 'rosrun rosbag topic_renamer.py ' \
            + input_topic + ' ' + file_in + ' ' \
                + move_write_topic + ' ' + file_out
        print('====================================')
        print('running command:  \n' + com_rename)
        os.system(com_rename)
        print('begin mv: ' + file)
        com_mv = 'mv ' + file_out + ' ' + file_in
        os.system(com_mv)
print('all done! ++++++++++++++++++++++++')
