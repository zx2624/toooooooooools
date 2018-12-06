#! /usr/bin/env python

import rosbag
import numpy as np
import sys
import getopt
import glob
import os
import time
import subprocess
import transforms3d.euler as euler
from  compare_odoms import plot_odom_and_save_image

#odom1 to odom2's extrinsic[x, y, z, roll, pitch, yaw]
odom1_to_odom2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# read messages 
def readmsg(bags, results_path, topic_name1, topic_name2, trans, rot):

	#store all bag's topic1 odom
	merge_odom1 = []
	#store all bag's topic1 odom
	merge_odom2 = []

	for bag_file in bags:

		#store topic1 odom
		odom1 = []
		#store topic1 odom
		odom2 = []

		# open bag
		print("Open bag: {}".format(bag_file))
		bag  = rosbag.Bag(bag_file)

		# loop over the topic to read evey message
		# for topic, msg, t in bag.read_messages(topics=topic_name):
		for topic, msg, t in bag.read_messages(topics=[topic_name1,topic_name2]):
			sec = t.to_sec()
			position = msg.pose.pose.position
			quaternion = msg.pose.pose.orientation
			
			#quaternion to euler
			quat = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
			rpy = euler.quat2euler(quat)

			if topic == topic_name1:
				odom1_trans = np.array([position.x, position.y, position.z])

				odom1_rot = euler.euler2mat(rpy[0], rpy[1], rpy[2])

				#new rotation 
				new_rot = np.dot(odom1_rot, rot) 
				
				#new translation
				new_trans = np.dot(odom1_rot, trans) + odom1_trans

				new_rpy = euler.mat2euler(new_rot)
				
				#insert odom to odom1
				odom1.append([sec, new_trans[0], new_trans[1], new_trans[2],
					new_rpy[0], new_rpy[1], new_rpy[2]])

				merge_odom1.append([sec, new_trans[0], new_trans[1], new_trans[2],
					new_rpy[0], new_rpy[1], new_rpy[2]])
			elif topic == topic_name2:
				#insert odom to odom2
				odom2.append([sec, position.x, position.y, position.z, rpy[0], 
					rpy[1], rpy[2]])
				merge_odom2.append([sec, position.x, position.y, position.z, rpy[0], 
					rpy[1], rpy[2]])

		#call plot and save image fuction
		if len(odom1) <= 0:
			print(bag_file + 'do not have' + topic_name1)
		elif len(odom2) <= 0:
			print(bag_file + 'do not have' + topic_name2)
		else:
			file_name_prefix = bag_file.split('/')[-1][:-4]
			plot_odom_and_save_image(odom1, odom2, file_name_prefix, results_path)

		#close bag file
		bag.close()
		# print odom1
		# print odom2
	#merge all bag
	file_name_prefix = 'merge'
	plot_odom_and_save_image(merge_odom1, merge_odom2, file_name_prefix,
		results_path)

def main(bags, path, topic_name1, topic_name2, trans, rot):

	readmsg(bags, path, topic_name1, topic_name2, trans, rot)

def usage():
	print("1. rosrun read_bag_plot_in_google_map.py -f /media/test.bag \
		/sensor/novatel/odom  /pose_optimize/velodyne/odom 0 0 0 0 0 0")
	print("2. rosrun read_bag_plot_in_google_map.py -d /media/test \
		/sensor/novatel/odom /pose_optimize/velodyne/odom 0 0 0 0 0 0")
	sys.exit()

def sort(path, bags):
	results = []
	bag_name_suffix_num = []
	bag_name = []

	for bag in bags:
		bag_name = bag.split('/')[-1]
		#bag_name_prefix = bag_name.split('_')[0]
		bag_name_suffix_num.append(int(bag_name.split('_')[1][:-4]))
	bag_name_suffix_num.sort()

	bag_name_prefix = bag_name.split('_')[0]

	if path[-1] != '/':
		path = path + '/'

	for i in range(len(bag_name_suffix_num)):
		results.append(path + bag_name_prefix + '_' +
			str(bag_name_suffix_num[i]) + '.bag')

	return results

if __name__ == '__main__':

	if len(sys.argv) < 11:
		usage()
	argv = sys.argv[1:]
	bags = []
	path = ''

	topic_name1 = ''
	topic_name2 = ''

	try:
		opts, args = getopt.getopt(argv, 'hf:d:')
	except getopt.GetoptError:
		usage()

	if len(args) < 2:
		usage()

	for opt, arg in opts:
		if opt == '-h':
			usage()
		elif opt == '-f':
			bags.append(arg)
			if not os.path.exists(arg):
				print 'arg' + ' is not exists!\n'
				usage()
			path = os.path.split(arg)[0]
		elif opt == '-d':
			if not os.path.exists(arg):
				print 'arg' + ' is not exists!\n'
				usage()
			bags = glob.glob(arg + '/*.bag')
			path = arg
		else:
			usage()
	print('path: {}'.format(path))

	#sort bags
	bags = sort(path, bags)
	if len(bags) == 0:
		print "The bags name have some problem!"
		sys.exit()
	for bag in bags:
		print "Bag: " + bag

	topic_name1 = args[0]
	topic_name2 = args[1]

	print args[2:]
	for idx in range(len(args[2:])):
		odom1_to_odom2[idx] = float(args[2:][idx])

	print('topic1 name: {}'.format(topic_name1))
	print('topic2 name: {}'.format(topic_name2))
	print('odom1 to odom2 extrinsic: {}'.format(odom1_to_odom2))

	trans = np.array([odom1_to_odom2[0], odom1_to_odom2[1],
		odom1_to_odom2[2]])

	rot = euler.euler2mat(odom1_to_odom2[3], odom1_to_odom2[4], odom1_to_odom2[5])

	print("odom1 to odom2 trans:\n {}".format(trans))
	print("odom1 to odom2 rot:\n {}".format(rot))

	results_path = ''
	if path[-1] == '/':
		results_path = path + 'odom_compare'
	else:
		results_path = path + '/odom_compare'

	if not os.path.exists(results_path):
		print results_path + ' is not exists!\n So we create it!'
		os.makedirs(results_path)

	main(bags, results_path, topic_name1, topic_name2, trans, rot)	
