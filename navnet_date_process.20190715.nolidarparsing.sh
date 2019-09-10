#!/usr/bin/env bash

START_BAG_SEQ=5
END_BAG_SEQ=5

bag_name="20190621-125345_"
Bag_path="/mnt/data-1/map_data/20190323/origin"
bag_fullin=${Bag_path}"/"${bag_name}

source devel/setup.bash


# mkdir ${Bag_path}"/g2o"
# mkdir ${Bag_path}"/frame"
# mkdir ${Bag_path}"/log"
#pwd
#echo "----------------------------------------------------------------------------------------"
#echo "run image_parsing........"
#echo "----------------------------------------------------------------------------------------"
#cd image_parsing/
#source devel/setup.bash
#pwd
#bash offline_ros_seg_det_track_path_1.sh ${START_BAG_SEQ} ${END_BAG_SEQ} ${bag_fullin} > ${Bag_path}"/log/1_image_parsing.txt" 2>&1
#bash offline_ros_seg_det_track_path_2.sh ${START_BAG_SEQ} ${END_BAG_SEQ} ${bag_fullin} > ${Bag_path}"/log/2_image_parsing.txt" 2>&1
#cd ..


#
#sleep 5
#echo "---------------------------------------------------------------------------------------"
#echo "run image_to_points......."
#echo "--------------------------------------------------------------------------------------"
#cd SOSO
#pwd
#bash run_horizon_dataset_rosbag.path.Aft_0420.sh ${START_BAG_SEQ} ${END_BAG_SEQ} ${bag_fullin} > ${Bag_path}"/log/2_image_to_points.txt" 2>&1
#cd ..


#sleep 5
#echo "----------------------------------------------------------------------------------------"
#echo "run interpolation_mapping........"
#echo "----------------------------------------------------------------------------------------"

############################# mapping package
# cd mapping
# pwd
# source devel/setup.bash


#rosrun interpolation_mapping interpolation_mapping src/interpolation_mapping/config.navnet.path.Aft_0420.yaml ${Bag_path} > ${Bag_path}"/log/3_interpolation_mapping.txt" 2>&1



# sleep 5
# echo "----------------------------------------------------------------------------------------"
# echo "run pose_optimize........"
# echo "----------------------------------------------------------------------------------------"
# rosrun pose_optimize pose_optimize src/pose_optimize/config.navnet.path.Aft_0420.yaml ${Bag_path} > ${Bag_path}"/log/4_pose_optimize.txt" 2>&1
# cd ..
############################# mapping package


#sleep 5
#echo "----------------------------------------------------------------------------------------"
#echo "run lidar_parsing........"
#echo "----------------------------------------------------------------------------------------"
#cd lidarpdparsing
#pwd
#source devel/setup.bash
#rosrun plot_image_pasring_results plot_image_pasring_results_no_pandora config.zx.yaml lidar_to_hugo_20190710.yaml ${Bag_path} > ${Bag_path}"/log/5_lidar_parsing.txt" 2>&1
#cd ..

# sleep 5
# echo "----------------------------------------------------------------------------------------"
# echo "run update lidar_parsing........"
# echo "----------------------------------------------------------------------------------------"
# cd update_lidar_parsing
# pwd
# source devel/setup.bash
# rosrun update_lidar_parsing update_lidar_parsing_node config.template.yaml ${Bag_path} > ${Bag_path}"/log/6_liadr_parsing.txt" 2>&1
# cd ..

# sleep 5
# echo "----------------------------------------------------------------------------------------"
# echo "run generate_hd_map........"
# echo "----------------------------------------------------------------------------------------"
# cd generatehdmap
# source devel/setup.bash
# rosrun generate_hd_map generate_hd_map_node config.template.yaml ${Bag_path} > ${Bag_path}"/log/8_generate_hd_map.txt" 2>&1
# cd ..
# pwd 

echo "END"
