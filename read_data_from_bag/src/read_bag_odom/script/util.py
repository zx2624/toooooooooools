#!/usr/bin/env python

def sort(path, bags):
	results = []
	bag_name_suffix_num = []
	bag_name = []

	for bag in bags:
		bag_name = bag.split('/')[-1]
		bag_name_suffix_num.append(int(bag_name.split('_')[-1][:-4]))
	bag_name_suffix_num.sort()

	bag_name_prefix = ''
	split_str = bag_name.split('_')
	for i in range(len(split_str) - 1):
		bag_name_prefix += split_str[i]
		bag_name_prefix += '_'

	if path[-1] != '/':
		path = path + '/'

	for i in range(len(bag_name_suffix_num)):
		results.append(path + bag_name_prefix + 
			str(bag_name_suffix_num[i]) + '.bag')

	return results
