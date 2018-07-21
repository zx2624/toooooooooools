#!/bin bash
for((num=2;num<=21;num++))
do 
	if [ $num -lt 10 ];
		then 
			num_str="0$num"
			echo "a: $num_str";
		else 
			num_str=$num
			echo "b: $num_str";
		fi;
	rosrun convert_kitti_lidar_data_to_rosbag convert_kitti_lidar_data_to_rosbag /media/horizon_ad/A2204A2C02CC88A3/siyuan/lidar_dataset/sequences/$num_str/velodyne /media/horizon_ad/A2204A2C02CC88A3/siyuan/eva_odometry/dataset/sequences/$num_str/times.txt /media/horizon_ad/A2204A2C02CC88A3/siyuan/lidar_dataset_bag/$num_str.bag
done
