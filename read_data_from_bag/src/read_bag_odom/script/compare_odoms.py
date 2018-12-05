#!/usr/bin/env python
import sys
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# f1 = open("odom1.txt", "w")
# f2 = open("odom2.txt", "w")

def plot_odom_and_save_image(odom1, odom2, bag_file, path):

	i = 0
	j = 0
	cnt = 0
	MAX_TIME_DIFF = 0.04

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

	save_one_on_each_frames = 1

	flag = False

	while i < len(odom1) and j < len(odom2):
		if odom1[i][0] - odom2[j][0] > MAX_TIME_DIFF :
			j+=1
			continue
		elif odom2[j][0] - odom1[i][0] > MAX_TIME_DIFF :
			i+=1
			continue
		else:
			if cnt % save_one_on_each_frames == 0:
				ts_idx.append(cnt / save_one_on_each_frames)

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
	for idx in range(len(compare_attr)):
		if idx < 3:
			plt.scatter(ts_idx, [i[idx + 1] for i in odom1_new] , color = 'red',
				label='odom1_' + compare_attr[idx])
			plt.scatter(ts_idx, [i[idx + 1] for i in odom2_new] , color = 'green',
				label='odom2_'+ compare_attr[idx])
		else:
			plt.scatter(ts_idx, [i[idx + 1] * 180.0 / 3.1415926 for i in odom1_new] ,
				color = 'red', label='odom1_' + compare_attr[idx])
			plt.scatter(ts_idx, [i[idx + 1] * 180.0 / 3.1415926 for i in odom2_new] ,
				color = 'green', label='odom2_'+ compare_attr[idx])

		plt.axis('auto')
		plt.legend()
		# plt.show()

		png_name = bag_file.split('/')[-1][:-4] + '.' + compare_attr[idx] + '.png'
		if path[-1] == '/':
			global_name = path + png_name
		else:
			global_name = path + '/' + png_name
		print global_name
		plt.savefig(global_name)
		#clear
		plt.clf()
