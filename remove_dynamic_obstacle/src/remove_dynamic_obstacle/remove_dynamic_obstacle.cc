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

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/common/transforms.h>

//BeiJing map base
const double X_ST = 439311;
const double Y_ST = 4426691;
const double Z_ST = 0;
const double EPS = 0.01;
const double resolution = 0.05;

void select_points(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
									pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected) {
	source_selected->clear();
	for(unsigned int i = 0; i < source->points.size(); i++) {
		pcl::PointXYZI p = source->points[i];
		 if ((fabs(p.x) < 2 && 
					 fabs(p.y) < 2 &&
						fabs(p.z) < 2)) {//remove the lidar
							continue;
		 }
		 source_selected->push_back(p);
	}
}

inline Eigen::Matrix4d getMatrix4dFromMsg(const nav_msgs::Odometry& msg) {
	Eigen::Quaterniond q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
	msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
	Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
	pose.block<3, 3>(0, 0) = q.toRotationMatrix();
	pose(0, 3) = msg.pose.pose.position.x;
	pose(1, 3) = msg.pose.pose.position.y;
	pose(2, 3) = msg.pose.pose.position.z;
	return pose;
}

int main(int argc, char** argv) {
	if (argc < 3) {
		std::cerr << "Usgae: rosrun remove_dynamic_obstacle remove_dynamic_obstacle "
			<< "bag_file pointcloud_topic_name odom_topic_name \n";
		return -1;
	}
		rosbag::Bag bag;

		std::cout << "Open bag file " << argv[1] << std::endl;
		bag.open(argv[1], rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";

		std::cout << "Read topic " << argv[2] << std::endl;
		rosbag::View points_view(bag, rosbag::TopicQuery(argv[2]));
		rosbag::View odom_view(bag, rosbag::TopicQuery(argv[3]));

		rosbag::View::iterator odom_iter = odom_view.begin();
		
		rosbag::View::iterator pti = points_view.begin();   


		bool is_first_frame = true;

		sensor_msgs::PointCloud2ConstPtr ros_pcd; 
		pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		int64_t count = 0;

		clock_t start = clock(), end = clock();
		//statistic time 
		std::vector<double> time_count;

		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addCoordinateSystem(3.0 ,"coor");
		viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		viewer.initCameraParameters();
		viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree (resolution);

		while(odom_iter != odom_view.end() && pti != points_view.end()) { 
			nav_msgs::OdometryConstPtr od = odom_iter->instantiate<nav_msgs::Odometry>();
			sensor_msgs::PointCloud2ConstPtr pt = pti->instantiate<sensor_msgs::PointCloud2>();

			if(od->header.stamp.toSec() - pt->header.stamp.toSec() > EPS) {
				pti++;
				continue;
			}
			else if(pt->header.stamp.toSec() - od->header.stamp.toSec() > EPS) {
				odom_iter++;
				continue;
			}

			//get pointcloud
			pcl::fromROSMsg(*pt, *source);
			count++;

			//delete points on car
			select_points(source, source_selected);

			//get pose
			auto mat = getMatrix4dFromMsg(*od);
			mat(0, 3) = mat(0, 3) - X_ST;
			mat(1, 3) = mat(1, 3) - Y_ST;
			mat(2, 3) = mat(2, 3) - Z_ST;

			start = clock();

			//transform pointcloud to world coordinate
			pcl::transformPointCloud (*source_selected, *source_selected, mat);

			// assign point cloud to octree
			octree.setInputCloud (source_selected);

			std::cout << "Input points size: " << source_selected->size() << std::endl;

			// add points from cloud to octree
			octree.addPointsFromInputCloud ();

			std::cout << "Leaf cnt: " << octree.getLeafCount() << std::endl;

			boost::shared_ptr<std::vector<int> > newPointIdxVector (new std::vector<int>);

			// get a vector of new points, which did not exist in previous buffer
			octree.getPointIndicesFromNewVoxels (*newPointIdxVector);

			std::cout << "Diffrent points size: " << newPointIdxVector->size() << std::endl;

			filtered_cloud->clear();
			filtered_cloud->points.resize(source_selected->points.size());

			for(size_t i = 0; i < source_selected->points.size(); i++) {
				pcl::PointXYZI temp = source_selected->points[i];

				pcl::PointXYZRGB p1;
				p1.x = temp.x;
				p1.y = temp.y;
				p1.z = temp.z;
				p1.rgb = 255 << 8;
				filtered_cloud->points[i] = p1;
			}

			for (std::vector<int>::iterator it = newPointIdxVector->begin (); 
				it != newPointIdxVector->end (); ++it) {
				filtered_cloud->points[*it].rgb = 255<<16;
			}

			end = clock();

			time_count.push_back((end - start) * 1000/CLOCKS_PER_SEC);//get millisecond


			viewer.removePointCloud("cloud");
			viewer.addPointCloud(filtered_cloud, "cloud");
			viewer.spinOnce(10);

			std::cout << "frame: " << count << std::endl;
			std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC << 
				"ms" << std::endl;

		}

		double sum_time = 0.0;
		for(unsigned int i = 0;i < time_count.size(); i++) {
			sum_time += time_count[i];
		}
		std::sort(time_count.begin(), time_count.end());
		std::cout << "average_time: " << sum_time / time_count.size() << "ms" << std::endl;
		std::cout << "middle_time: " << time_count[time_count.size()/2] << "ms" << std::endl;

		bag.close();

		return 0;
	}
