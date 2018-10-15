#!/bin bash
source devel/setup.bash
for((num=0;num<=21;num++))
do 
	if [ $num -lt 10 ];
		then 
			num_str="0$num"
			echo "a: $num_str";
		else 
			num_str=$num
			echo "b: $num_str";
		fi;
	rosrun convert_loam_pose_to_kitti_camera_pose convert_loam_pose_to_kitti_camera_pose /media/horizon_ad/A2204A2C02CC88A3/siyuan/loam_velodyne_hdl64/20180720_odom/$num_str.txt /media/horizon_ad/A2204A2C02CC88A3/siyuan/eva_odometry/dataset/sequences/$num_str/calib.txt  /media/horizon_ad/A2204A2C02CC88A3/siyuan/loam_velodyne_hdl64/20180720_odom_convert_to_cam/$num_str.txt
done
