//Author: siyuan.yu

#include <ctime>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>


ros::Publisher pubCloud;
ros::Publisher pubOdom;
void select_points(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
									pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected) {
	source_selected->clear();
	for(unsigned int i = 0; i < source->points.size(); i++) {
		pcl::PointXYZI p = source->points[i];
		 source_selected->push_back(p);
	}
}
struct MFrameMsg{
	MFrameMsg()
  : bagID(0)
  , seq(0)
  , timestamp(0.0)
  , speed(0.0)
  , yaw_rate(0.0)
  , pose(Eigen::Matrix4f::Identity()){
}
  uint64_t bagID;
  uint64_t seq;
  double   timestamp;
  double   speed;
  double   yaw_rate;
	Eigen::Matrix4f pose;
	Eigen::Matrix<double, 6, 6> covariance;
};

void readMFrame(std::string& mframe_path, std::vector<MFrameMsg> &mframe) {
	ifstream file;
  file.open(mframe_path.c_str());
	if (file.is_open()) {
		MFrameMsg frame;
		while (!file.eof()) {
      file >> frame.bagID >> frame.seq >> frame.timestamp >> frame.speed >> 
		  frame.pose(0,0) >> frame.pose(0,1) >> frame.pose(0,2) >> frame.pose(0,3) >> 
			frame.pose(1,0) >> frame.pose(1,1) >> frame.pose(1,2) >> frame.pose(1,3) >> 
			frame.pose(2,0) >> frame.pose(2,1) >> frame.pose(2,2) >> frame.pose(2,3) >> 
		  frame.pose(3,0) >> frame.pose(3,1) >> frame.pose(3,2) >> frame.pose(3,3);
      mframe.push_back(frame);
		}
		mframe.pop_back();//file.eof() will double the last line, so delete back once
		cout << "****  OK: Read " << mframe.size() << " mframes in " << mframe_path << "  ****" << endl;
		file.close();
   } else {
		 cout << "****  ERROR: Don't exist mframe: " << mframe_path << "  ****" << endl;
	}
}
void MergePointcloud(std::string bag_file_path_ , std::string odom_path_  ) {
	rosbag::Bag bag_;
	bag_.open(bag_file_path_, rosbag::bagmode::Read);
	std::cout << "open bag file success! \n";

	std::fstream ofs(odom_path_.c_str());
	if(!ofs.is_open()) {
		std::cerr << odom_path_ << "not exist!\n";
		exit(-1);
	}

	std::vector<MFrameMsg> odom_frame;
	readMFrame(odom_path_, odom_frame);
	std::cout << "odom size: " << odom_frame.size() << std::endl;

	std::vector<rosbag::View::iterator> ptis;
	rosbag::View points_view(bag_, rosbag::TopicQuery("/sensor/velodyne/points"));
	for (rosbag::View::iterator pti = points_view.begin(); pti != points_view.end(); pti++) {
			//cout << "push back points iterator! \n"; 
	    ptis.push_back(pti);
	}
	std::cout << "pointcloud size: " << ptis.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr source_transformed(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr sum(new pcl::PointCloud<pcl::PointXYZI>);

	sensor_msgs::PointCloud2::Ptr ros_pcd(new sensor_msgs::PointCloud2);
	//for(uint64_t i = 0; i < odom_frame.size(); i++) {
	int i = 0, j = 0;
	nav_msgs::Odometry odom;
	odom.child_frame_id = "/velodyne_odom";
	odom.header.frame_id = "/velodyne_init";
	while(i < odom_frame.size() && j < ptis.size()) {
		ros_pcd = ptis[j]->instantiate<sensor_msgs::PointCloud2>();
		if (odom_frame[i].timestamp > (ros_pcd->header.stamp.toSec() + 0.01)) {
			j++;
		}
		else if ((odom_frame[i].timestamp + 0.1) < ros_pcd->header.stamp.toSec()) {
			i++;
		}
		else if ((odom_frame[i].timestamp - ros_pcd->header.stamp.toSec()) <= 0.1) {
		//pcl::fromROSMsg(*ros_pcd, *source);
		//select_points(source, source_selected);
		//Eigen::Matrix4f transform_to_init = odom_frame[i].pose;
		//pcl::transformPointCloud(*source, *source_transformed, transform_to_init);

		//pcl::toROSMsg(*source_transformed, *ros_pcd);
			odom.header.stamp.fromSec(odom_frame[i].timestamp);
			odom.pose.pose.position.x = odom_frame[i].pose(0,3);
			odom.pose.pose.position.y = odom_frame[i].pose(1,3);
			odom.pose.pose.position.z = odom_frame[i].pose(2,3);
			Eigen::Matrix3f mat;
			for (int j = 0; j < 3; j++) {
			    for (int k = 0; k < 3; k++) {
			        mat(j,k) = odom_frame[i].pose(j,k);
			      }
			  }
			Eigen::Quaternionf q(mat);
			odom.pose.pose.orientation.w = q.w();
			odom.pose.pose.orientation.x = q.x();
			odom.pose.pose.orientation.y = q.y();
			odom.pose.pose.orientation.z = q.z();

		pubCloud.publish(*ros_pcd);
		pubOdom.publish(odom);
		std::cout << "[frame " << i << " ]" << std::endl;
		i++;
		j++;
		usleep(20 * 1000);
		}
	}
	bag_.close();
	ofs.close();
}
int main(int argc, char** argv) {
	std::cout << "bag_file_path: " << argv[1] << std::endl
						<< "odom_path: "<< argv[2] << std::endl;
	ros::init(argc, argv, "transform_pointcloud");
	ros::NodeHandle nh; 
	pubCloud =	nh.advertise<sensor_msgs::PointCloud2> ("/sensor/velodyne/points", 1); 
	pubOdom = nh.advertise<nav_msgs::Odometry> ("/posegraph/odom", 1);

	MergePointcloud(std::string(argv[1]), std::string(argv[2]));

	return 0;
}
