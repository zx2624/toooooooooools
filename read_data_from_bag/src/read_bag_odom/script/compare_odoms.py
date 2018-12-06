#!/usr/bin/env python
import sys
import numpy as np
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# f1 = open("odom1.txt", "w")
# f2 = open("odom2.txt", "w")

def plot_odom_and_save_image(odom1, odom2, file_name_prefix, path):

	i = 0
	j = 0
	cnt = 0
	MAX_TIME_DIFF = 0.05

	ts_idx = []
	odom1_new = []
	odom2_new = []

	#store first odom
	odom1_start = []
	odom2_start = []

	# for odom1_temp in odom1:
		# str1 = ''
		# for i in odom1_temp:
			# str1 += str(i) + ' '	
		# str1+='\n'
		# f1.write(str1)

	# for odom2_temp in odom2:
		# str2 = ''
		# for i in odom2_temp:
			# str2 += str(i) + ' '	
		# str2+='\n'
		# f2.write(str2)

	flag = False

	while i < len(odom1) and j < len(odom2):
		if odom1[i][0] - odom2[j][0] > MAX_TIME_DIFF :
			j+=1
			continue
		elif odom2[j][0] - odom1[i][0] > MAX_TIME_DIFF :
			i+=1
			continue

		ts_idx.append(cnt)

		odom1_temp = odom1[i]
		odom2_temp = odom2[j]

		if not flag:
			flag = True
			odom1_start = odom1_temp[1:4]
			odom2_start = odom2_temp[1:4]

		#sub first odom
		odom1_temp[1] -= odom1_start[0]
		odom1_temp[2] -= odom1_start[1]
		odom1_temp[3] -= odom1_start[2]

		#sub first odom
		odom2_temp[1] -= odom2_start[0]
		odom2_temp[2] -= odom2_start[1]
		odom2_temp[3] -= odom2_start[2]

		odom1_new.append(odom1_temp)
		odom2_new.append(odom2_temp)

		cnt+=1
		i+=1
		j+=1

	print("pairs size: {}".format(len(odom1_new)))
	print("odom1 size: {}".format(len(odom1)))
	print("odom2 size: {}".format(len(odom2)))

	compare_attr = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
	global_name = []

	cos_odom1 = np.cos([odom[6] for odom in odom1_new])
	sin_odom1 = np.sin([odom[6] for odom in odom1_new])

	cos_odom2 = np.cos([odom[6] for odom in odom2_new])
	sin_odom2 = np.sin([odom[6] for odom in odom2_new])

	plt.quiver([odom[1] for odom in odom1_new], [odom[2] for odom in odom1_new],
		cos_odom1, sin_odom1, color='r', label = 'Odom1')
	plt.quiver([odom[1] for odom in odom2_new], [odom[2] for odom in odom2_new],
		cos_odom2, sin_odom2, color='g', label = 'Odom2')

	subpath = 'traj/'

	if path[-1] != '/':
		path += '/'

	if not os.path.exists(path + subpath):
		print path + subpath + ' is not exists!\n So we create it!'
		os.makedirs(path + subpath)

	global_name = path + subpath + file_name_prefix + '.traj.png'

	print global_name

	plt.title('Odom1 Odom2 trajectory')
	plt.xlabel('X(m)')
	plt.ylabel('Y(m)')
	plt.grid(True)
	plt.axis('auto')
	# plt.legend('')

	plt.savefig(global_name)
	#clear
	plt.clf()

	for idx in range(len(compare_attr)):
		if idx < 3:
			plt.subplot(3, 1, idx + 1)
			plt.scatter(ts_idx, [i[idx + 1] for i in odom1_new] , color = 'red', label = '')
				#label='odom1_' + compare_attr[idx])
			plt.scatter(ts_idx, [i[idx + 1] for i in odom2_new] , color = 'green', label = '')
				# label='odom2_'+ compare_attr[idx])
			plt.title('Odom1 Odom2 ' + compare_attr[idx])
			plt.xlabel('index')
			plt.ylabel(compare_attr[idx] + '(m)')
			plt.grid(True)
		else:
			plt.subplot(3, 1, idx - 2)
			plt.scatter(ts_idx, [i[idx + 1] * 180.0 / 3.1415926 for i in odom1_new] , 
				color = 'red', label = '') #, label='odom1_' + compare_attr[idx])
			plt.scatter(ts_idx, [i[idx + 1] * 180.0 / 3.1415926 for i in odom2_new] ,
				color = 'green', label = '') #, label='odom2_'+ compare_attr[idx])
			plt.title('Odom1 Odom2 ' + compare_attr[idx])
			plt.xlabel('index')
			plt.ylabel(compare_attr[idx] + '(deg)')
			plt.grid(True)

		plt.axis('auto')
		plt.legend()
		# plt.show()


		if idx % 3 == 2:
			if idx == 2:
				subpath = 'XYZ/'
				if not os.path.exists(path + subpath):
					print path + subpath + ' is not exists!\n So we create it!'
					os.makedirs(path + subpath)

				png_name = file_name_prefix + '.XYZ.png'
			else:
				subpath = 'RPY/'
				if not os.path.exists(path + subpath):
					print path + subpath + ' is not exists!\n So we create it!'
					os.makedirs(path + subpath)

				png_name = file_name_prefix + '.RPY.png'

			global_name = path + subpath + png_name
			print global_name

			plt.tight_layout()
			plt.savefig(global_name)
			#clear
			plt.clf()
