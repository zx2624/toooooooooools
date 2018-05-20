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
		//if(!bag_.isOpen()) {
			//std::cerr << bag_file_path_ << "not exist!\n";
			//exit(-1);
		//}

		std::ofstream ofs(gnss_odom_path_.c_str());
		std::ofstream ofs_raw("raw_gnss.txt");

		if(!ofs.is_open()) {
			std::cerr << gnss_odom_path_ << "not exist!\n";
			exit(-1);
		}

		bool is_first_frame = true;
		double last_x = 0;
		double last_y = 0;
		double last_z = 0;
		double last_time = 0;

		//shanghai base
		double init_x = 327753;
		double init_y = 3462160;
		double init_z = 15;

		int cnt = 0;

		rosbag::View gnss_odom_view(bag_, rosbag::TopicQuery(gnss_topic_name_));


		//Eigen::Matrix3d gnss_to_lidar = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
					 //* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
					 //* Eigen::AngleAxisd(90 * M_PI / 180, Eigen::Vector3d::UnitZ()).matrix();
		tf::Matrix3x3 gnss_to_lidar;
		gnss_to_lidar.setEulerYPR(90 * M_PI / 180, 0, 0);

		for(rosbag::View::iterator its = gnss_odom_view.begin(); 
				its != gnss_odom_view.end();its++) {
			nav_msgs::OdometryConstPtr od = its->instantiate<nav_msgs::Odometry>();

			double covariance[6] = {0};
			for(int row = 0; row < 6; row++)
			    covariance[row] = od->pose.covariance[6 * row + row];

			if(covariance[0] > 3 || covariance[1] > 3 || covariance[2] > 10) {
				continue;
			}

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
#ifdef DEBUG
			//std::cout << "[tf_rpy, eigen_rpy]: " << rpy_raw(0) << " " << rpy_raw(1) << " " << rpy_raw(2) 
									//<< " " << rpy_debug(2) << " " << rpy_debug(1) << " " << rpy_debug(0) << std::endl;
#endif

			
			if(is_first_frame) {
				last_x= od->pose.pose.position.x; 
				last_y= od->pose.pose.position.y; 
				last_z= od->pose.pose.position.z; 

				last_time = od->header.stamp.toSec();
				is_first_frame = false;
				continue;
			}
			else {
			//if(std::isnan(od->pose.pose.orientation.x)) continue;
				double x= od->pose.pose.position.x; 
				double y= od->pose.pose.position.y; 
				double z= od->pose.pose.position.z; 
				double time = od->header.stamp.toSec();

				double delta_x = x - last_x; 
				double delta_y = y - last_y; 
				double delta_z = z - last_z; 

				last_x = x;
				last_y = y;
				last_z = z;

				if(time - last_time > 0.3 || (fabs(delta_x) < 0.1 && fabs(delta_y) < 0.1)){
					last_time = time;
					continue;
				}

				last_time = time;

				if(od->pose.covariance[35] > 5) {

					cnt++;
					auto yaw = atan2(delta_y, delta_x);
					//auto yaw = -atan(delta_y / delta_x);
					//if (delta_y > 0) {
						 //yaw = -yaw - M_PI / 2;
					//} else {
							 //yaw = -yaw + M_PI * 3 / 2;
						 //}
					
					//if (yaw > 2 * M_PI || yaw < 0) {
						//yaw = 2 * M_PI;
					//}

					//std::cout << "[x, y, delta_x, delta_y, yaw]" << x - init_x << " " << y - init_y
						//<< " " << delta_y << " " << delta_x << " " << yaw << std::endl;
					//auto pitch = asin(delta_z / sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2)));
					auto pitch = 0;

					//Eigen::Matrix3d r0 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
								 //* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
								 //* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
					tf::Matrix3x3 r0;
					r0.setEulerYPR(yaw, pitch, 0);

					auto r1 = gnss_to_lidar.transpose() * r0;
					//rpy_raw = r1.eulerAngles(0, 1, 2);
					tf::Matrix3x3(r1).getRPY(rpy_raw(0), rpy_raw(1), rpy_raw(2));
				}

				ofs << std::fixed << std::setprecision(6)
						<< od->header.stamp.toSec() << " " << od->pose.pose.position.x - init_x
						<< " " << od->pose.pose.position.y - init_y
						<< " " << od->pose.pose.position.z - init_z
						<< " " << rpy_raw(0) << " " << rpy_raw(1) << " " << rpy_raw(2)
						<< " " << covariance[0] << " " << covariance[1] << " " << covariance[2]
						<< " " << covariance[3] << " " << covariance[4] << " " << covariance[5]
						<< std::endl;
			}
		}

		std::cout << "cnt: " << cnt << std::endl;
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
	//tools::ReadGNSS read_gnss(argv[1], argv[2]);
	//tools::ReadGNSS read_gnss(argv[1], argv[2], "/sensor/velodyne/odom");
	tools::ReadGNSS read_gnss(argv[1], argv[2], "/sensor/sick/odom");
	read_gnss.SaveOdomToFile();
	return 0;
}
