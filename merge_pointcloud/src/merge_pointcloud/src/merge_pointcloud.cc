//Author: siyuan.yu

#include <ctime>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

void select_points(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected) {
 source_selected->clear();
  //for(unsigned int i = 0; i < source->points.size(); i++) {
 //for VLP32, decrease point num
  for(unsigned int i = 0; i < source->points.size(); i+=2) {
		if (fabs(source->points[i].x) < 1 &&
			fabs(source->points[i].y) < 1 &&
      fabs(source->points[i].z) < 1) {//for points in car
      continue;
		}
		if (fabs(source->points[i].x) > 30 ||
			fabs(source->points[i].y) > 30 ||
      fabs(source->points[i].z) > 30 ) {
      continue;
		}

		//TODO for 20180720 yuanboyuan data test
		if(source->points[i].z < -3 || source->points[i].z > -2) continue; 

		source->points[i].intensity =  255 * ( source->points[i].z - (-3)) / (-2 - (-3));
		//TODO end

		source_selected->push_back(source->points[i]);
	}
}

void MergePointcloud(std::string bag_file_path , std::string out_pcd_name,
											std::string out_ply_name) {
	rosbag::Bag bag_;
	bag_.open(bag_file_path, rosbag::bagmode::Read);
	std::cout << "open bag file success! \n";

	std::vector<rosbag::View::iterator> ptis;

	//rosbag::View points_view(bag_, rosbag::TopicQuery("/sensor/velodyne/points"));
	//rosbag::View odom_view(bag_, rosbag::TopicQuery("/sensor/velodyne/odom"));
	//rosbag::View points_view(bag_, rosbag::TopicQuery("/sensor/sick/points"));
	//rosbag::View odom_view(bag_, rosbag::TopicQuery("/sensor/sick/odom"));
	//rosbag::View points_view(bag_, rosbag::TopicQuery("/sensor/pandar/points"));
	//rosbag::View odom_view(bag_, rosbag::TopicQuery("/sensor/pandar/odom"));
	//rosbag::View points_view(bag_, rosbag::TopicQuery("/hvo/keyframe/pointcloud"));
	//rosbag::View odom_view(bag_, rosbag::TopicQuery("/hvo/keyframe/dsoodom_opti"));

	rosbag::View points_view(bag_, rosbag::TopicQuery("/velodyne_points"));
	rosbag::View odom_view(bag_, rosbag::TopicQuery("/pose_optimize/velodyne/odom"));

	if (points_view.size() <= 0 ) {
		std::cerr << "pointcloud size is 0!\n";
		return;
	}
	else {
		std::cout << "pointcloud size: " << points_view.size() << std::endl;
	}

	if (odom_view.size() <= 0 ) {
		std::cerr << "oodm size is 0!\n";
		return;
	}
	else {
		std::cout << "odom size: " << odom_view.size() << std::endl;
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr source_transformed(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr local_sum(new pcl::PointCloud<pcl::PointXYZI>);


	int64_t cnt = 0;

	const double EPS = 0.005; // 0.005 second

	rosbag::View::iterator odom_iter = odom_view.begin();
	rosbag::View::iterator pti = points_view.begin();
	while(odom_iter !=  odom_view.end() && pti != points_view.end()) {
		nav_msgs::OdometryConstPtr od = 
			odom_iter->instantiate<nav_msgs::Odometry>();
		sensor_msgs::PointCloud2ConstPtr pt 
			= pti->instantiate<sensor_msgs::PointCloud2>();

		if(od->header.stamp.toSec() - pt->header.stamp.toSec() > EPS) {
			pti++;
			continue;
		}
		else if(pt->header.stamp.toSec() - od->header.stamp.toSec() > EPS) {
			odom_iter++;
			continue;
		}

		pcl::fromROSMsg(*pt, *source);

		Eigen::Matrix4f transform_to_init = Eigen::Matrix4f::Identity();
		transform_to_init(0, 3) = od->pose.pose.position.x;
		transform_to_init(1, 3) = od->pose.pose.position.y;
		transform_to_init(2, 3) = od->pose.pose.position.z;
		//transform_to_init(0, 3) = od->pose.pose.position.x - 327753;
		//transform_to_init(1, 3) = od->pose.pose.position.y - 3462160;
		//transform_to_init(2, 3) = od->pose.pose.position.z - 15;
		transform_to_init.block<3, 3>(0, 0) = 
			Eigen::Quaternionf(
				od->pose.pose.orientation.w, 
				od->pose.pose.orientation.x, 
				od->pose.pose.orientation.y, 
				od->pose.pose.orientation.z 
					).toRotationMatrix();

		//select points of pointcloud
		select_points(source, source_selected);

		pcl::transformPointCloud(*source_selected, *source_transformed,
				transform_to_init);

		cout << "[ " << ++cnt << "/" << points_view.size() << " ] " <<  
			std::fixed << pt->header.stamp.toSec() << " points size: "
			<< source_transformed->points.size() << endl; 
		cout << endl << transform_to_init << endl;

		*local_sum += *source_transformed;
		odom_iter++;
		pti++;

		//if (cnt == 500) break;
	}

	bag_.close();

	if (local_sum->points.size() > 0) {
		std::cout << "points sum: " << local_sum->points.size() << std::endl;
		//save to pcd file
		//pcl::io::savePCDFileASCII(out_pcd_name, *local_sum);
		//std::cout << "Save pointcloud " <<
			//out_pcd_name << std::endl;
		//ply writer
		pcl::PLYWriter writer;
		std::cout << "Save ply " <<
			out_ply_name << std::endl;
		//defalut is ascii format
		writer.write(out_ply_name, *local_sum);
	}
	else 
		std::cout << "Local sum pointcloud size is 0\n";
}

int main(int argc, char** argv) {
	if(argc < 4) {
		std::cerr << "argv: in_bag out_pcd_name out_ply_name\n";
		return -1;
	}
	std::cout << "in_bag: " << argv[1] << std::endl
						<< "out_pcd_name: "<< argv[2] << std::endl
						<< "out_ply_name: "<< argv[3] << std::endl;

	if (0 != access(argv[1], F_OK)){
		std::cerr << argv[1] << "not exist!\n";
	}

	MergePointcloud(std::string(argv[1]), std::string(argv[2]), std::string(argv[3]));
	return 0;
}
