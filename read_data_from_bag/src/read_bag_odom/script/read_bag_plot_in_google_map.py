#! /usr/bin/env python

# Prerequisite
#(1) sudo apt-get install chromium-chromedriver
#(2) sudo pip install selenium
#(3) sudo pip install matplotlib
#(4) sudo pip install gmplot

from transform_WGS84_GCJ02 import WGS2GCJ
import rosbag
import sys
import getopt
import glob
import os
import time
from selenium import webdriver
import geodesy.utm as utm
import matplotlib.pyplot as plt
from gmplot import gmplot
import subprocess


R = [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9365079365079367, 0.8571428571428572, 0.7777777777777777, 0.6984126984126986, 0.6190476190476191, 0.53968253968254, 0.4603174603174605, 0.3809523809523814, 0.3015873015873018, 0.2222222222222223, 0.1428571428571432, 0.06349206349206415, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.03174603174603208, 0.08465608465608465, 0.1375661375661377, 0.1904761904761907, 0.2433862433862437, 0.2962962962962963, 0.3492063492063493, 0.4021164021164023, 0.4550264550264553, 0.5079365079365079, 0.5608465608465609, 0.6137566137566139, 0.666666666666667]
G = [ 0, 0.03968253968253968, 0.07936507936507936, 0.119047619047619, 0.1587301587301587, 0.1984126984126984, 0.2380952380952381, 0.2777777777777778, 0.3174603174603174, 0.3571428571428571, 0.3968253968253968, 0.4365079365079365, 0.4761904761904762, 0.5158730158730158, 0.5555555555555556, 0.5952380952380952, 0.6349206349206349, 0.6746031746031745, 0.7142857142857142, 0.753968253968254, 0.7936507936507936, 0.8333333333333333, 0.873015873015873, 0.9126984126984127, 0.9523809523809523, 0.992063492063492, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9841269841269842, 0.9047619047619047, 0.8253968253968256, 0.7460317460317465, 0.666666666666667, 0.587301587301587, 0.5079365079365079, 0.4285714285714288, 0.3492063492063493, 0.2698412698412698, 0.1904761904761907, 0.1111111111111116, 0.03174603174603208, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
B = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01587301587301582, 0.09523809523809534, 0.1746031746031744, 0.2539682539682535, 0.333333333333333, 0.412698412698413, 0.4920634920634921, 0.5714285714285712, 0.6507936507936507, 0.7301587301587302, 0.8095238095238093, 0.8888888888888884, 0.9682539682539679, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
            
# read messages 
def readmsg(topic_name, bags, path_of_bag):
	# set arrays to store data
	gps_lat = []
	gps_lon = []
	# mix lat 
	min_lat = sys.float_info.max
	# max lat
	max_lat = sys.float_info.min

	#min lon
	min_lon = sys.float_info.max
	#max lon
	max_lon = sys.float_info.min

	# mix x 
	min_x = sys.float_info.max
	# max x
	max_x = sys.float_info.min

	#min y
	min_y = sys.float_info.max
	#max y
	max_y = sys.float_info.min

	# set a flag
	Iter = 0
	invalid_num = 0

	for bag_file in bags:
		# open bag
		print("Open bag: {}".format(bag_file))
		bag  = rosbag.Bag(bag_file)

		# loop over the topic to read evey message
		for topic, msg, t in bag.read_messages(topics=topic_name):
			sec         = t.to_sec() 
			easting = msg.pose.pose.position.x
			northing = msg.pose.pose.position.y
			altitude = msg.pose.pose.position.z

			print("easting:{} northing:{} altitude:{}".format(easting, northing, altitude))
			#convert easting, northing, altitude to latitude, longitude, altitude
			point = utm.UTMPoint(easting, northing, altitude, 50, 'S').toMsg()
			print("latitude:{} longitude:{} altitude:{}".format(point.latitude, point.longitude, point.altitude))
			print("")

		# transform from GCJ-2 to WGS-84
			ret = WGS2GCJ(point.latitude, point.longitude)
			if not ret[2]:
			  invalid_num+=1
			  continue

			gps_lat.append(ret[0])
			gps_lon.append(ret[1])

			if max_lat < ret[0]:
				max_lat = ret[0]
			if min_lat > ret[0]:
				min_lat = ret[0]

			if max_lon < ret[1]:
				max_lon = ret[1]
			if min_lon > ret[1]:
				min_lon = ret[1]

			if max_x < easting:
				max_x = easting
			if min_x > easting:
				min_x = easting

			if max_y < northing:
				max_y = northing
			if min_y > northing:
				min_y = northing


			Iter += 1

		bag.close()

	print("Read {} points!".format(Iter))
	print("Read {} invalid points!".format(invalid_num))

	# get center point's latitude, longitude
	center_lat = (max_lat + min_lat) / 2.0
	center_lon = (max_lon + min_lon) / 2.0
	print("center point lat:{} lon:{} ".format(center_lat, center_lon))

	# plot the data
	gmap_plot(gps_lat,gps_lon, path_of_bag, abs(max_x - min_x), abs(max_y - min_y), center_lat, center_lon)
	screenshot_html(path_of_bag)

# google map plotting
def gmap_plot(gps_lat, gps_lon, path, x_range, y_range, cen_lat, cen_lon):

	print("x_range: {:.2f} meters  y_range_: {:.2f} meters".format(x_range, y_range))

	# get scale 
	scale = get_scale(x_range, y_range)

	# Place map
	gmap = gmplot.GoogleMapPlotter(cen_lat, cen_lon, scale)

	cnt = len(gps_lat) - 1

	for a in range(cnt):
		#use 85 radix to instead 256

		# v1 = a / ( 85 * 85)
		# v2 = (a - v1 * 85 * 85) / 85
		# v3 = (a - v1 * 85 * 85 - v2 * 85)
		
		# v1 = v1 * 3 % 256 
		# v2 = v2 * 3 % 256 
		# v3 = v3 * 3 % 256 

		# print("{}".format((a / 32) % 256))
		v1 = (int)(255 * R[(a / 32) % 64 ]) % 256
		v2 = (int)(255 * G[(a / 32) % 64 ]) % 256
		v3 = (int)(255 * B[(a / 32) % 64 ]) % 256


		prefix = '0'
		rr = str(hex(v1))[2:]
		if len(rr) < 2:
		  rr = prefix + rr
		
		gg = str(hex(v2))[2:]
		if len(gg) < 2:
		  gg = prefix + gg
		
		bb = str(hex(v3))[2:]
		if len(bb) < 2:
		  bb = prefix + bb
		
		color = '#' + rr + gg + bb
		# gmap.plot(gps_lat[a : a + 2], gps_lon[a : a + 2], color, edge_width=10)
		gmap.plot(gps_lat[a : a + 2], gps_lon[a : a + 2], color, edge_width=1)

	# gmap.plot(gps_lat, gps_lon, 'cornflowerblue', edge_width=10)
	# gmap.plot(gps_lat, gps_lon, 'cornflowerblue', edge_width=10)
	#gmap.plot(gps_lat[1:3], gps_lon[1:3], '#000000', edge_width=10)
	#gmap.plot(gps_lat[500:], gps_lon[500:], '#ffff00', edge_width=10)

	file_path = path + '/google_map.html'
	print('Save html to {}'.format(file_path))
	# Draw
	gmap.draw(file_path)

def screenshot_html(path):

	options = webdriver.ChromeOptions()
	options.add_argument("--start-maximized")

	# for chrome browser
	driver = webdriver.Chrome('/usr/lib/chromium-browser/chromedriver', chrome_options = options)
	#TODO Add surport of Firefox browser 
	#driver = webdriver.Firefox()

	file_path = 'file://'
	file_path = file_path +  path + '/google_map.html'
	print("Open html file path:{}".format(file_path))
	driver.get(file_path)
	time.sleep(10)
	png_path = path + '/google_map.png'
	driver.save_screenshot(png_path)
	print('Save png to {}'.format(png_path))
	driver.quit()

def get_screen_width_and_height():
	 
	cmd = ['xrandr']
	cmd2 = ['grep', '*']
	p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
	p2 = subprocess.Popen(cmd2, stdin=p.stdout, stdout=subprocess.PIPE)
	p.stdout.close()
	 
	resolution_string, junk = p2.communicate()

	# resolution = resolution_string.split()[0]
	# width, height = resolution.split('x')
	width, height = 1440, 878

	print("Screen width, height [{}px, {}px]".format(width, height))
	return width, height

def get_scale(x_range, y_range):
	#reference https://www.cnblogs.com/xixihuang/p/5779262.html
	# from level 2 to level 22 meter/pixel
	#TODO need to validate the correctly
	scale_table = 71000, 36000, 17000, 9000, 4000, 2000, 568, 549, 278, 139, 69, 35, 17, 8, 4, 2.15, 1.07, 0.54, 0.27,  0.138, 0.0694
	#get width and height of the screen
	width, height = get_screen_width_and_height()
	width = float(width)
	height = float(height)
	idx = 0
	for value in scale_table:
		if width * value < x_range or height * value < y_range:
			break
		idx += 1
	print("Scale: {}".format(idx + 1))
	# return idx + 1
	return idx

def main(bags, path):
	# choose one topic  
	topic_name = '/sensor/novatel/odom'
#topic_name = '/sensor/gnss/odom'

	readmsg(topic_name, bags, path)

def usage():
	print("1. rosrun read_bag_plot_in_google_map.py -f /media/test.bag")
	print("2. rosrun read_bag_plot_in_google_map.py -d /media/test")

if __name__ == '__main__':
	if len(sys.argv) < 2:
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
		# print('bags: {}'.format(bags))
		print('path: {}'.format(path))
		# print("{} {} {}".format(len(R), len(G), len(B)))

		main(bags, path)	
