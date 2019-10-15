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
import time

sufix = time.strftime("_%H%M", time.localtime())

R = [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9365079365079367, 0.8571428571428572, 0.7777777777777777, 0.6984126984126986, 0.6190476190476191, 0.53968253968254, 0.4603174603174605, 0.3809523809523814, 0.3015873015873018, 0.2222222222222223, 0.1428571428571432, 0.06349206349206415, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.03174603174603208, 0.08465608465608465, 0.1375661375661377, 0.1904761904761907, 0.2433862433862437, 0.2962962962962963, 0.3492063492063493, 0.4021164021164023, 0.4550264550264553, 0.5079365079365079, 0.5608465608465609, 0.6137566137566139, 0.666666666666667]
G = [ 0, 0.03968253968253968, 0.07936507936507936, 0.119047619047619, 0.1587301587301587, 0.1984126984126984, 0.2380952380952381, 0.2777777777777778, 0.3174603174603174, 0.3571428571428571, 0.3968253968253968, 0.4365079365079365, 0.4761904761904762, 0.5158730158730158, 0.5555555555555556, 0.5952380952380952, 0.6349206349206349, 0.6746031746031745, 0.7142857142857142, 0.753968253968254, 0.7936507936507936, 0.8333333333333333, 0.873015873015873, 0.9126984126984127, 0.9523809523809523, 0.992063492063492, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9841269841269842, 0.9047619047619047, 0.8253968253968256, 0.7460317460317465, 0.666666666666667, 0.587301587301587, 0.5079365079365079, 0.4285714285714288, 0.3492063492063493, 0.2698412698412698, 0.1904761904761907, 0.1111111111111116, 0.03174603174603208, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
B = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01587301587301582, 0.09523809523809534, 0.1746031746031744, 0.2539682539682535, 0.333333333333333, 0.412698412698413, 0.4920634920634921, 0.5714285714285712, 0.6507936507936507, 0.7301587301587302, 0.8095238095238093, 0.8888888888888884, 0.9682539682539679, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
x_l = 440804 + 14970
x_r = 440804 + 15045
y_d = 4425720 - 8002
y_u = 4425720 - 7889
# Read one frame each SKIP_STEP frames
SKIP_STEP = 3
            
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

		start_time = 0.0
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
			altitude = msg.pose.pose.position.z
		#record start time
			if is_first_frame:
				is_first_frame = False
				start_time = end_time
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
		i = 0
		while i < len(times) / 2:
			name = bag_file.split('.')[0]
			name = name.split('/')[-1]
			savepath = path + '/' + 'cut2' + sufix# + name + '_' + str(i) + '.bag'
			if not os.path.exists(savepath):
				print (savepath + ' does not exist!\n So we create it!')
				os.makedirs(savepath)
			name = savepath + '/' + name + '_' + str(i) + '.bag'
			print(name)
			outbags.append(Bag(name, 'w'))
			i = i + 1

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
					i = i + 2
					# print(t.to_sec())

		for bag in outbags:
			bag.close()
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
