#! /usr/bin/env python

# Prerequisite
#(1) sudo apt-get install chromium-chromedriver
#(2) sudo pip install selenium
#(3) sudo pip install matplotlib
#(4) sudo pip install gmplot

from util import sort
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

save_big = False
x_l = 440804 - 9076
x_r = 440804 - 9053
y_d = 4425720 - 11519
y_u = 4425720 - 11413
# Read one frame each SKIP_STEP frames
            
# read messages 
def readmsg(topic_name, bags, path):

	# set a flag
	Iter = 0
	bag_idx = -1

	for bag_file in bags:
		bag_idx += 1
		# open bag
		print("Open bag: {}".format(bag_file))
		bag  = rosbag.Bag(bag_file)

		end_time = 0.0
		is_first_frame = True
		prev = 0
		now = 0
		# loop over the topic to read evey message
		times = [] 
		last_e = 0
		last_n = 0
		last_t = 0
		for topic, msg, t in bag.read_messages(topics=topic_name):
			end_time = t.to_sec() 

			easting = msg.pose.pose.position.x
			northing = msg.pose.pose.position.y
			# altitude = msg.pose.pose.position.z
		#record start time
			if is_first_frame:
				is_first_frame = False
				# print(easting, "====", northing)
				if easting > x_l and easting < x_r and northing > y_d and northing < y_u:
					prev = 1
					print("=====================first in ====================")
					times.append(t.to_sec())
				continue
			if easting > x_l and easting < x_r and northing > y_d and northing < y_u:
				now = 1
			else:
				now = 0
			if(now != prev):
				# print("==============", easting, northing, altitude)
				# print(t.to_sec())
				times.append(t.to_sec())
			last_e = easting
			last_n = northing
			last_t = t.to_sec()
			prev = now
			Iter += 1
		if last_e > x_l and last_e < x_r and last_n > y_d and last_n < y_u:
			times.append(last_t)
		print("all times: ", times)
		bag.close()
		topics = "*"
		outbags = []

		savepath = path + '/' + 'cut2'# + name + '_' + str(i) + '.bag'
		if not os.path.exists(savepath):
			print (savepath + ' does not exist!\n So we create it!')
			os.makedirs(savepath)
		i = 0
		while i < len(times) / 2:
			name = bag_file.split('.')[0]
			name = name.split('/')[-1]
			name = savepath + '/' + name + 'small_' + str(i) + '.bag'
			print(name)
			outbags.append(Bag(name, 'w'))
			i = i + 1
		bag_big_name = (bag_file.split('.')[0]).split('/')[-1]
		bag_big_name = savepath + '/' + bag_big_name + '.bag'
		bag_big = rosbag.Bag(bag_big_name, 'w')
		print("===begin to write bag===")
		matchedtopics = []
		with Bag(bag_file, 'r') as ib:
			for topic, msg, t in ib:
				i = 0
				while i < len(times):
					if t.to_sec() >times[i] and t.to_sec() < times[i+1]:
						if any(fnmatchcase(topic, pattern) for pattern in topics):
							if not topic in matchedtopics:
								matchedtopics.append(topic)
							# print("Write msg " + str(t.to_sec()))
							outbags[i/2].write(topic, msg, t)
					elif save_big:
						if any(fnmatchcase(topic, pattern) for pattern in topics):
							if not topic in matchedtopics:
								matchedtopics.append(topic)
							# print("Write msg " + str(t.to_sec()))
							bag_big.write(topic, msg, t)
					i = i + 2
					# print(t.to_sec())

		for bag in outbags:
			bag.close()
		bag_big.close()
		print('bag ', bag_file, 'done')
	

def validtime(time, times):
	i = 0
	while i < len(times):
		if time > times[i] and time < times[i+1]:
			return True
		i = i + 2
	return False

def usage():
	print("1. rosrun read_bag_odom read_bag_plot_in_google_map.py -f /media/test.bag /sensor/novatel/odom")
	print("2. rosrun read_bag_odom read_bag_plot_in_google_map.py -d /media/test /sensor/novatel/odom")
	sys.exit()

if __name__ == '__main__':
	if len(sys.argv) < 4:
		usage()
	argv = sys.argv[1:]
	bags = []
	path = ''
	try:
		opts, args = getopt.getopt(argv, 'hf:d:')
	except getopt.GetoptError:
		usage()
	for opt, arg in opts:
		if opt == '-h':
			usage()
		elif opt == '-f':
			bags.append(arg)
			path = os.path.split(arg)[0]
		elif opt == '-d':
			bags = glob.glob(arg + '/*.bag')
			path = arg
		else:
			usage()

		#sort
		# bags = sort(path, bags) 
		if len(bags) == 0:
			print("The bags name have some problem!")
			sys.exit()

		for  bag_file in bags:
			print ('bags: ' + bag_file + '\n')
			# print(bag_file.split('.')[0], "===++===")
		# print('path: {}'.format(path))

		topic_name = args[0]
		print ("Will Read the topic: " + topic_name)
		readmsg(topic_name, bags, path)	
