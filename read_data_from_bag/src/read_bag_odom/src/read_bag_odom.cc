//Author: siyuan.yu

#include  "read_gnss/read_gnss.h"

#include <fstream>
#include <iomanip>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>

#include <Eigen/Eigen>

namespace tools {

	void ReadGNSS::SaveOdomToFile() {

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";

		std::ofstream ofs_raw("raw_odom.txt");
		std::ofstream ofs("raw_odom_good.txt");

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

		for(rosbag::View::iterator its = gnss_odom_view.begin(); 
				its != gnss_odom_view.end();its++) {
			nav_msgs::OdometryConstPtr od = its->instantiate<nav_msgs::Odometry>();

			double covariance[6] = {0};
			for(int row = 0; row < 6; row++)
			    covariance[row] = od->pose.covariance[6 * row + row];

			Eigen::Quaterniond quat(od->pose.pose.orientation.w,
					od->pose.pose.orientation.x, 
					od->pose.pose.orientation.y, 
					od->pose.pose.orientation.z
					);
			
			//auto rpy_debug = quat.toRotationMatrix().eulerAngles(2, 1, 0);

			Eigen::Vector3d rpy_raw;
			tf::Matrix3x3(tf::Quaternion(od->pose.pose.orientation.x, od->pose.pose.orientation.y,
                 od->pose.pose.orientation.z,
                 od->pose.pose.orientation.w)).getRPY(rpy_raw(0), rpy_raw(1), rpy_raw(2));

			ofs_raw << std::fixed << std::setprecision(6)
					<< od->header.stamp.toSec() << " " << od->pose.pose.position.x - init_x
					<< " " << od->pose.pose.position.y  - init_y
					<< " " << od->pose.pose.position.z - init_z
					<< " " << rpy_raw(0) << " " << rpy_raw(1) << " " << rpy_raw(2) 
					//<< " " << rpy_debug(2) << " " << rpy_debug(1) << " " << rpy_debug(0) 
					<< " " << covariance[0] << " " << covariance[1] << " " << covariance[2]
					<< " " << covariance[3] << " " << covariance[4] << " " << covariance[5]
					<< std::endl;

			cnt++;

			if(covariance[0] > 3 ||
					covariance[1] > 3 ||
					//covariance[2] > 3 ||
					covariance[3] > 3 ||
					covariance[4] > 3 ||
					covariance[5] > 3 
					) {
				continue;
			}

			ofs << std::fixed << std::setprecision(6)
					<< od->header.stamp.toSec() << " " << od->pose.pose.position.x - init_x
					<< " " << od->pose.pose.position.y - init_y
					<< " " << od->pose.pose.position.z - init_z
					<< " " << rpy_raw(0) << " " << rpy_raw(1) << " " << rpy_raw(2)
					<< " " << covariance[0] << " " << covariance[1] << " " << covariance[2]
					<< " " << covariance[3] << " " << covariance[4] << " " << covariance[5]
					<< std::endl;
			cnt_good++;
		}

		std::cout << "odom num: " << cnt << std::endl;
		std::cout << "good odom num: " << cnt_good << std::endl;

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";
	}
}

void show_usgae() {
	std::cout << 
		"Usage: rosrun read_gnss read_gnss_node bag_file_path save_gnss_odom_path"
		<< std::endl;
}
int main(int argc, char** argv) {
	if(argc < 2) {
		show_usgae();
		exit(-1);
	}
	std::cout << "bag_file_path: " << argv[1] << std::endl;

	//tools::ReadGNSS read_gnss(argv[1]);
	//tools::ReadGNSS read_gnss(argv[1], "/localization/loam/odom");
	//tools::ReadGNSS read_gnss(argv[1], "/sensor/velodyne/odom");
	//tools::ReadGNSS read_gnss(argv[1], "/sensor/sick/odom");
	//tools::ReadGNSS read_gnss(argv[1], "/sensor/velodyne_cent/odom");
	//tools::ReadGNSS read_gnss(argv[1], "/hvo/keyframe/lidarodom");
	//tools::ReadGNSS read_gnss(argv[1], "/hvo/keyframe/dsoodom_opti");
	tools::ReadGNSS read_gnss(argv[1], "/pose_optimize/velodyne/odom");
	read_gnss.SaveOdomToFile();
	return 0;
}
