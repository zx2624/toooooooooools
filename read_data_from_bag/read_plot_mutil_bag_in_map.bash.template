#!/usr/bin/env bash
source devel/setup.bash

#You must set your bag path
bag_path=/media/horizon_ad/AD0008/DATA/20181203/sensor_20181203-141522_/
#Your must sey your bag prefix
bag_name_prefix=20181203-141522_

for((num=0;num<=11;num++))
do 
	bag_name=$bag_name_prefix$num
#echo "b: $num_str";

	rosrun read_bag_odom read_bag_plot_in_google_map.py -f $bag_path$bag_name.bag
	cd $bag_path
	mv google_map.html 	$bag_name_prefix.html
	mv google_map.png 	$bag_name_prefix.png

done
