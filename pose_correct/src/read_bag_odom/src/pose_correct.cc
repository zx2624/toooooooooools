//Author: siyuan.yu

#include  "read_gnss/read_gnss.h"

#include <fstream>
#include <iomanip>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <boost/algorithm/string.hpp>
#include <dirent.h>
#include <regex>

#include <Eigen/Eigen>
bool getAllBagFilesPath(std::string bag_dir,
	std::vector<std::string>& vec_bag_files_path) {
	//check if bag file path is right
	DIR *dp;
	struct dirent *dirp;
	if((dp = opendir(bag_dir.c_str())) == NULL){
		std::cout << "Can not open " << bag_dir << std::endl;
		return false;
	}
	if (bag_dir[bag_dir.size() - 1] != '/') {
		bag_dir += '/';
	}

	std::regex reg ("(.*)(.bag)");
	//Record each bag's start time
	std::vector<std::string> vec_bag_path_temp;

	uint64_t idx = 0;
	while((dirp = readdir(dp)) != NULL) {
		//DT_RRE means is regular file
		if(dirp->d_type == DT_REG) {
			if(std::regex_match(dirp->d_name, reg)) {
				std::string bag_name(dirp->d_name);
				std::string all_path = bag_dir + bag_name;
				// std::cout << "the name is " << bag_name << std::endl;
				vec_bag_files_path.push_back(bag_dir + bag_name);
				// std::cout << "the dir " << bag_dir << std::endl;
				// size_t pos_start = bag_name.find_last_of("_");
				// size_t pos_end = bag_name.find_last_of(".");

				// if (pos_start == -1 || pos_end == -1) {
				// 	std::cerr << "bag_name : " << bag_name << "is not valid!\n";
				// 	return false;
				// }

				// size_t num = std::stoi(bag_name.substr(pos_start + 1, 
				// 	pos_end - pos_start - 1));
				// //std::cout << "num: " << num << std::endl; 

				// pos_start = bag_name.find_last_of("/");
				// pos_end = bag_name.find_last_of("_");

				// //std::string pre = bag_name.substr(pos_start + 1);
				// std::string pre = bag_name.substr(pos_start + 1, pos_end - pos_start - 1);
				// //std::cout << "pre: " << pre << std::endl; 

				// vec_start_time_idx.push_back(TimeIdx(num, idx++, pre));
				// vec_bag_path_temp.push_back(all_path);
			}
		}
	}
}

namespace tools {

	void ReadGNSS::SaveOdomToFile() {
		std::vector<std::string> names;
		getAllBagFilesPath(bag_file_path_, names);
		for(auto s : names){
			std::cout << "bags are : \n" << std::endl;
			std::cout << s << std::endl;
		}
		for (auto bag_name : names)
		{
			bag_.open(bag_name, rosbag::bagmode::Read);
			std::cout << "open bag file success! \n";

			//shanghai base
			//double init_x = 327753;
			//double init_y = 3462160;
			//double init_z = 15;

			double init_x = 0;
			double init_y = 0;
			double init_z = 0;

			int cnt = 0;
			int cnt_good = 0;

			rosbag::View gnss_odom_view(bag_, rosbag::TopicQuery(gnss_topic_name_));
			std::vector<nav_msgs::Odometry> vec_odom;
			for (rosbag::View::iterator its = gnss_odom_view.begin();
				 its != gnss_odom_view.end(); its++)
			{
				nav_msgs::OdometryConstPtr od = its->instantiate<nav_msgs::Odometry>();

				double covariance[6] = {0};
				for (int row = 0; row < 6; row++)
					covariance[row] = od->pose.covariance[6 * row + row];

				Eigen::Quaterniond quat(od->pose.pose.orientation.w,
										od->pose.pose.orientation.x,
										od->pose.pose.orientation.y,
										od->pose.pose.orientation.z);

				//auto rpy_debug = quat.toRotationMatrix().eulerAngles(2, 1, 0);

				Eigen::Vector3d rpy_raw;
				tf::Matrix3x3(tf::Quaternion(od->pose.pose.orientation.x, od->pose.pose.orientation.y,
											 od->pose.pose.orientation.z,
											 od->pose.pose.orientation.w))
					.getRPY(rpy_raw(0), rpy_raw(1), rpy_raw(2));
				//get corret pose
				Eigen::Matrix3d mat_rpy =
					Eigen::AngleAxisd(rpy_raw(2), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd(rpy_raw(1), Eigen::Vector3d::UnitX()) *
					Eigen::AngleAxisd(rpy_raw(0), Eigen::Vector3d::UnitY()).matrix();
				Eigen::Quaterniond q_rpy(mat_rpy);
				nav_msgs::Odometry msg_rpy;
				msg_rpy = *od;
				msg_rpy.pose.pose.orientation.w = q_rpy.w();
				msg_rpy.pose.pose.orientation.x = q_rpy.x();
				msg_rpy.pose.pose.orientation.y = q_rpy.y();
				msg_rpy.pose.pose.orientation.z = q_rpy.z();
				vec_odom.push_back(msg_rpy);
				//get corret pose

			}
			bag_.close();
			bag_.open(bag_name, rosbag::bagmode::Append);
			for(auto msg : vec_odom){
				bag_.write("/sensor/novatel/odom_correct", msg.header.stamp, msg);
			}
			bag_.close();
			std::cout << "save Finished!\n";
		}
	}
}

void show_usgae() {
	std::cout << 
		"Usage: rosrun read_gnss read_gnss_node bag_file_path odom_topic_name"
		<< std::endl;
}
int main(int argc, char** argv) {
	if(argc < 3) {
		show_usgae();
		exit(-1);
	}
	std::cout << "bag_file_path: " << argv[1] << std::endl;

	tools::ReadGNSS read_gnss(argv[1], argv[2]);
	read_gnss.SaveOdomToFile();
	return 0;
}
