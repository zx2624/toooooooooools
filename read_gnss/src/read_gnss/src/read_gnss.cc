//Author: siyuan.yu

#include  "read_gnss/read_gnss.h"

#include <fstream>
#include <iomanip>

#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Eigen>
namespace tools {

	void ReadGNSS::SaveOdomToFile() {

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";
		//if(!bag_.isOpen()) {
			//std::cerr << bag_file_path_ << "not exist!\n";
			//exit(-1);
		//}

		std::ofstream ofs(gnss_odom_path_.c_str());
		if(!ofs.is_open()) {
			std::cerr << gnss_odom_path_ << "not exist!\n";
			exit(-1);
		}

		double init_x, init_y, init_z;
		bool is_first_frame = true;
		rosbag::View gnss_odom_view(bag_, rosbag::TopicQuery(gnss_topic_name_));

		for(rosbag::View::iterator its = gnss_odom_view.begin(); 
				its != gnss_odom_view.end();its++) {
			nav_msgs::OdometryConstPtr od = its->instantiate<nav_msgs::Odometry>();

			if(is_first_frame) {
				init_x= od->pose.pose.position.x; 
				init_y= od->pose.pose.position.y; 
				init_z= od->pose.pose.position.z; 
				is_first_frame = false;
			}
			if(std::isnan(od->pose.pose.orientation.x)) continue;
			Eigen::Quaternionf quat(
					od->pose.pose.orientation.w,
					od->pose.pose.orientation.x,
					od->pose.pose.orientation.y,
					od->pose.pose.orientation.z
					);
			Eigen::Vector3f rpy = quat.toRotationMatrix().eulerAngles(2, 1, 0);
			ofs << std::fixed << std::setprecision(6)
					<< od->header.stamp.toSec() << " " << od->pose.pose.position.x - init_x
					<< " " << od->pose.pose.position.y - init_y 
					<< " " << od->pose.pose.position.z - init_z
					<< " " << rpy(2) << " "
					<< " " << rpy(1) << " "
					<< " " << rpy(0) << " "
					<< std::endl;
			std::cout << std::fixed << std::setprecision(6)
					<< od->header.stamp.toSec() << " " << od->pose.pose.position.x - init_x
					<< " " << od->pose.pose.position.y - init_y 
					<< " " << od->pose.pose.position.z - init_z
					<< " " << rpy(2) << " "
					<< " " << rpy(1) << " "
					<< " " << rpy(0) << " "
					<< std::endl;
		}
		std::cout << "Init position: " << std::fixed
			<< init_x << " " << init_y << " " << init_z << std::endl;
		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";
	}
}
void show_usgae() {
	std::cout << "rosrun read_gnss read_gnss_node bag_file_path save_gnss_odom_path \n";
}
int main(int argc, char** argv) {
	if(argc < 3) {
		std::cout << "need bag file param!\n";
		show_usgae();
		exit(-1);
	}
	std::cout << "bag_file_path: " << argv[1] << std::endl
						<< "gnss_odom_path: "<< argv[2] << std::endl;
	tools::ReadGNSS read_gnss(argv[1], argv[2]);
	read_gnss.SaveOdomToFile();
	return 0;
}
