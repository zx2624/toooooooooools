#!/usr/bin/env bash

bag_info="/media/horizon_ad/Elements/map_data/BeiJing/ShunYi/20181203/GPS_trajectory_all/bag_info.txt"
chassis_bag="/media/horizon_ad/Elements/map_data/BeiJing/ShunYi/20181203/chassis_bag/20181203_150429_pandora_.chassis_odom_2018-12-13-14-41-47.bag"

if [[ -f "$bag_info" ]]
then
    # read it
	while IFS=' ' read -r raw_bag start_stamp end_stamp
	    do
				echo "raw_bag: $raw_bag"
				echo "start_stamp: $start_stamp"
				echo "end_stamp: $end_stamp"
				~/siyuan/all_kinds_of_ros_pack/append_bag.py $raw_bag $chassis_bag $start_stamp $end_stamp
	done <"$bag_info"
fi

