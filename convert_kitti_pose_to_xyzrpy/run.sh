#!/bin bash
source devel/setup.bash
for((num=0;num<=10;num++))
do 
	if [ $num -lt 10 ];
		then 
			num_str="0$num"
			echo "a: $num_str";
		else 
			num_str=$num
			echo "b: $num_str";
		fi;
	rosrun convert_kitti_pose_to_xyzrpy convert_kitti_pose_to_xyzrpy /media/horizon_ad/A2204A2C02CC88A3/siyuan/eva_odometry/dataset/sequences/$num_str/times.txt /media/horizon_ad/A2204A2C02CC88A3/siyuan/lidar_dataset/poses/$num_str.txt /media/horizon_ad/A2204A2C02CC88A3/siyuan/eva_odometry/dataset/sequences/$num_str/calib.txt  /media/horizon_ad/A2204A2C02CC88A3/siyuan/lidar_dataset_pose_xyzrpy/$num_str.txt
done
