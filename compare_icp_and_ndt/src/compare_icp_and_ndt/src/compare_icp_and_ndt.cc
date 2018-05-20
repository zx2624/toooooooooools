//Author: siyuan.yu

#include  "compare_icp_and_ndt/compare_icp_and_ndt.h"

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
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

namespace tools {

	void select_points(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
										pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected) {
		source_selected->clear();
		for(unsigned int i = 0; i < source->points.size(); i++) {
			pcl::PointXYZI p = source->points[i];
			 if (p.z > 100 || p.z < 0 || 
						(fabs(p.x) < 3 && fabs(p.y) < 2 && fabs(p.z) < 2)) {//remove the lidar
				        continue;
			 }
			 source_selected->push_back(p);
		}
	}

	void CompareICPandNDT::SaveOdomToFileUsingNDT() {
		std::cout << "Using NDT!" << std::endl;

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";
		//if(!bag_.isOpen()) {
			//std::cerr << bag_file_path_ << "not exist!\n";
			//exit(-1);
		//}

		std::ofstream ofs(output_odom_path_.c_str());
		if(!ofs.is_open()) {
			std::cerr << output_odom_path_ << "not exist!\n";
			exit(-1);
		}

		rosbag::View points_view(bag_, rosbag::TopicQuery(points_topic_name_));
		bool is_first_frame = true;

		//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;
		//icp.setMaxCorrespondenceDistance(10);
		//icp.setMaximumIte

		// define voxelgrid filter
		pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
#ifdef CUDA_NDT
		gpu::GNormalDistributionsTransform ndt;
		std::cout << "Using gpu ndt!" << std::endl;
#else
		pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
		std::cout << "Using cpu ndt!" << std::endl;
#endif
	  ndt.setTransformationEpsilon(10e-6);
	  ndt.setStepSize(0.01);
		//ndt.setResolution(1);
	  ndt.setMaximumIterations(200);


		Eigen::Matrix4f transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform_start_to_now = Eigen::Matrix4f::Identity();

		sensor_msgs::PointCloud2ConstPtr ros_pcd; 
		pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
#ifdef CUDA_NDT
		Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
#else 
		pcl::PointCloud<pcl::PointXYZI> init_guess;
#endif

		int64_t count = 0;
		bool has_converged = false;
		int final_num_iteration = 0;
		double fitness_score = 0.0;

		clock_t start = clock(), end = clock();
		//statistic time 
		std::vector<double> time_count;

		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addCoordinateSystem(3.0 ,"coor");
		viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		viewer.initCameraParameters();
		viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for(rosbag::View::iterator its = points_view.begin(); 
				its != points_view.end();its++) {
			if(count++ == 1000) break;
			//if(count++%5 != 0) continue;
			//get a data instantiate
			ros_pcd = its->instantiate<sensor_msgs::PointCloud2>();
			pcl::fromROSMsg(*ros_pcd, *source);

			select_points(source, source_selected);

			if(is_first_frame) {
				is_first_frame = false;
				*target = *source_selected;

				// Apply voxelgrid filter
				voxel_grid_filter.setInputCloud(target);
				std::cout << "Before filter size: " << target->points.size() << std::endl;
				//voxel_grid_filter.filter(*target);
				std::cout << "After filter size: " << target->points.size() << std::endl;

				continue;
			}
			//if(!using_icp_) {
				voxel_grid_filter.setInputCloud(source_selected);
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;
				//voxel_grid_filter.filter(*source_selected);
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;

				ndt.setInputTarget(target);
				std::cout << "Target pointcloud size: " << target->points.size() << std::endl;

				ndt.setInputSource(source_selected);
				std::cout << "Source pointcloud size: " << source_selected->points.size() << std::endl;

				start = clock();
				ndt.align(init_guess);
				end = clock();
				time_count.push_back((end - start) * 1000/CLOCKS_PER_SEC);//get millisecond

				transform_source_to_target = ndt.getFinalTransformation();

				has_converged = ndt.hasConverged();
				fitness_score = ndt.getFitnessScore();
				final_num_iteration = ndt.getFinalNumIteration();
			
				transform_start_to_now = transform_start_to_now * transform_source_to_target;

				show_cloud->clear();
				for(size_t i = 0; i < target->points.size(); i++) {
					pcl::PointXYZI p = target->points[i];
		      pcl::PointXYZRGB p1;
		      p1.x = p.x;
		      p1.y = p.y;
		      p1.z = p.z;
		      p1.r = 0;
		      p1.g = 255;
		      p1.b = 0;
		      show_cloud->push_back(p1);
				}
				Eigen::Vector4f temp;
				for(size_t i = 0; i < source_selected->points.size(); i++) {
					pcl::PointXYZI p = source_selected->points[i];

					temp << p.x, p.y, p.z, 1;
					temp = transform_source_to_target * temp;

		      pcl::PointXYZRGB p1;
		      p1.x = temp(0);
		      p1.y = temp(1);
		      p1.z = temp(2);
		      p1.r = 255;
		      p1.g = 0;
		      p1.b = 0;
		      show_cloud->push_back(p1);
				}

				viewer.removePointCloud("cloud");
        viewer.addPointCloud(show_cloud, "cloud");
				viewer.spinOnce(10);

				//set the source to target for next loop
				*target = *source_selected;

				std::cout << "| has_converged | fitness_score | final_num_iteration| \n";
				std::cout << has_converged << " | "
									<< fitness_score << " | "
									<< final_num_iteration << " | \n";
			//}

			ofs << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " ";
			for(int i = 0; i < 4; i++)
				for(int j = 0; j < 4; j++)
					ofs << std::fixed << std::setprecision(6) 
							<< transform_start_to_now(i, j) << " ";
			ofs << std::endl;

			std::cout << "frame: " << count << std::endl;
			std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC << std::endl;

			std::cout << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " ";
			for(int i = 0; i < 4; i++)
				for(int j = 0; j < 4; j++)
					std::cout << std::fixed << std::setprecision(6) 
										//<< transform_start_to_now(i, j) << " ";
										<< transform_source_to_target(i, j) << " ";
			std::cout << std::endl;
			//set 100 times to loop
			//if(count == 100 * 5 +1) break;
		}
		double sum_time = 0.0;
		for(unsigned int i = 0;i < time_count.size(); i++) {
			sum_time += time_count[i];
		}
		std::sort(time_count.begin(), time_count.end());
		std::cout << "average_time: " << sum_time / time_count.size() << std::endl;
		std::cout << "middle_time: " << time_count[time_count.size()/2] << std::endl;

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";
	}
	void CompareICPandNDT::SaveOdomToFileUsingICP() {
		std::cout << "Using ICP!" << std::endl;

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";
		//if(!bag_.isOpen()) {
			//std::cerr << bag_file_path_ << "not exist!\n";
			//exit(-1);
		//}

		std::ofstream ofs(output_odom_path_.c_str());
		if(!ofs.is_open()) {
			std::cerr << output_odom_path_ << "not exist!\n";
			exit(-1);
		}

		rosbag::View points_view(bag_, rosbag::TopicQuery(points_topic_name_));
		bool is_first_frame = true;


		// define voxelgrid filter
		pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI, float> icp;
		icp.setMaxCorrespondenceDistance(10);
		icp.setMaximumIterations(200);
		icp.setTransformationEpsilon(1e-6);
	  icp.setEuclideanFitnessEpsilon(0);

		//ndt.setTransformationEpsilon(0.01);
		//ndt.setStepSize(0.1);
		//ndt.setResolution(1.0);
		//ndt.setMaximumIterations(30);

		//Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform_start_to_now = Eigen::Matrix4f::Identity();

		sensor_msgs::PointCloud2ConstPtr ros_pcd; 
		pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> aligned;

		int64_t count = 0;
		bool has_converged = false;
		int final_num_iteration = 0;
		double fitness_score = 0.0;

		clock_t start = clock(), end = clock();
		//statistic time 
		std::vector<double> time_count;

		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addCoordinateSystem(3.0 ,"coor");
		viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		viewer.initCameraParameters();
		viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for(rosbag::View::iterator its = points_view.begin(); 
				its != points_view.end();its++) {
			//if(count++%5 != 0) continue;
			if(count++ == 1000) break;
			//get a data instantiate
			ros_pcd = its->instantiate<sensor_msgs::PointCloud2>();
			pcl::fromROSMsg(*ros_pcd, *source);

			select_points(source, source_selected);

			if(is_first_frame) {
				is_first_frame = false;
				*target = *source_selected;

				// Apply voxelgrid filter
				voxel_grid_filter.setInputCloud(target);
				std::cout << "Before filter size: " << target->points.size() << std::endl;
				//voxel_grid_filter.filter(*target);
				std::cout << "After filter size: " << target->points.size() << std::endl;

				continue;
			}
			//if(!using_icp_) {
				voxel_grid_filter.setInputCloud(source_selected);
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;
				//voxel_grid_filter.filter(*source_selected);
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;

				icp.setInputTarget(target);
				std::cout << "Target pointcloud size: " << target->points.size() << std::endl;

				icp.setInputSource(source_selected);
				std::cout << "Source pointcloud size: " << source_selected->points.size() << std::endl;

				start = clock();
				icp.align(aligned);
				end = clock();
				time_count.push_back((end - start) * 1000/CLOCKS_PER_SEC);//get millisecond

				transform_source_to_target = icp.getFinalTransformation();

				has_converged = icp.hasConverged();
				fitness_score = icp.getFitnessScore();
				//final_num_iteration = icp.getFinalNumIteration();
			
				transform_start_to_now = transform_start_to_now * transform_source_to_target;

				show_cloud->clear();
				for(size_t i = 0; i < target->points.size(); i++) {
					pcl::PointXYZI p = target->points[i];
		      pcl::PointXYZRGB p1;
		      p1.x = p.x;
		      p1.y = p.y;
		      p1.z = p.z;
		      p1.r = 0;
		      p1.g = 255;
		      p1.b = 0;
		      show_cloud->push_back(p1);
				}
				Eigen::Vector4f temp;
				for(size_t i = 0; i < source_selected->points.size(); i++) {
					pcl::PointXYZI p = source_selected->points[i];

					temp << p.x, p.y, p.z, 1;
					temp = transform_source_to_target * temp;

		      pcl::PointXYZRGB p1;
		      p1.x = temp(0);
		      p1.y = temp(1);
		      p1.z = temp(2);
		      p1.r = 255;
		      p1.g = 0;
		      p1.b = 0;
		      show_cloud->push_back(p1);
				}

				viewer.removePointCloud("cloud");
        viewer.addPointCloud(show_cloud, "cloud");
				viewer.spinOnce(10);

				//set the source to target for next loop
				*target = *source_selected;

				std::cout << "| has_converged | fitness_score | final_num_iteration| \n";
				std::cout << has_converged << " | "
									<< fitness_score << " | "
									<< final_num_iteration << " | \n";
			//}

			ofs << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " ";
			for(int i = 0; i < 4; i++)
				for(int j = 0; j < 4; j++)
					ofs << std::fixed << std::setprecision(6) 
							<< transform_start_to_now(i, j) << " ";
			ofs << std::endl;

			std::cout << "frame: " << count << std::endl;
			std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC << std::endl;

			std::cout << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " ";
			for(int i = 0; i < 4; i++)
				for(int j = 0; j < 4; j++)
					std::cout << std::fixed << std::setprecision(6) 
										//<< transform_start_to_now(i, j) << " ";
										<< transform_source_to_target(i, j) << " ";
			std::cout << std::endl;
			//set 100 times to loop
			//if(count == 100 * 5 + 1 ) break;
		}
		double sum_time = 0.0;
		for(unsigned int i = 0;i < time_count.size(); i++) {
			sum_time += time_count[i];
		}
		std::sort(time_count.begin(), time_count.end());
		std::cout << "average_time: " << sum_time / time_count.size() << std::endl;
		std::cout << "middle_time: " << time_count[time_count.size()/2] << std::endl;

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";
	}
	void CompareICPandNDT::SaveLoamOdomToFile() {
		std::cout << "Save loam odom to file!" << std::endl;

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";
		//if(!bag_.isOpen()) {
			//std::cerr << bag_file_path_ << "not exist!\n";
			//exit(-1);
		//}

		std::ofstream ofs(output_odom_path_.c_str());
		if(!ofs.is_open()) {
			std::cerr << output_odom_path_ << "not exist!\n";
			exit(-1);
		}

		rosbag::View odoms_view(bag_, rosbag::TopicQuery("/localization/loam/odom"));

		int64_t count = 0;
		for(rosbag::View::iterator its = odoms_view.begin(); 
				its != odoms_view.end();its++) {
			if(count++ == 1000) break;
			//get a data instantiate
			nav_msgs::OdometryConstPtr od = its->instantiate<nav_msgs::Odometry>(); 


			ofs << std::fixed << std::setprecision(6)
					<< od->header.stamp.toSec() << " "
					<< od->pose.pose.position.x << " "
					<< od->pose.pose.position.y << " "
					<< od->pose.pose.position.z << " "
					<< od->pose.pose.orientation.x << " "
					<< od->pose.pose.orientation.y << " "
					<< od->pose.pose.orientation.z << " "
					<< od->pose.pose.orientation.w
					<< std::endl;

			std::cout << std::fixed << std::setprecision(6)
					<< od->header.stamp.toSec() << " "
					<< od->pose.pose.position.x << " "
					<< od->pose.pose.position.y << " "
					<< od->pose.pose.position.z << " "
					<< od->pose.pose.orientation.x << " "
					<< od->pose.pose.orientation.y << " "
					<< od->pose.pose.orientation.z << " "
					<< od->pose.pose.orientation.w
					<< std::endl;
		}

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";

	}
}
void show_usgae() {
	std::cout << "1:run icp method:\
								rosrun compare_icp_and_ndt compare_icp_and_ndt_node \
								bag_file_path save_odom_path -icp \n";
	std::cout << "2:run ndt method:\
								rosrun compare_icp_and_ndt compare_icp_and_ndt_node \
								bag_file_path save_odom_path -ndt \n";
	std::cout << "3:save loam odom:\
								rosrun compare_icp_and_ndt compare_icp_and_ndt_node \
								bag_file_path save_odom_path -loam \n";
}
int main(int argc, char** argv) {
	if(argc < 4) {
		std::cout << "need bag file param!\n";
		show_usgae();
		exit(-1);
	}
	std::cout << "bag_file_path: " << argv[1] << std::endl
						<< "output_odom_path: "<< argv[2] << std::endl;
#ifdef pandar
	tools::CompareICPandNDT compare_icp_and_ndt(argv[1], argv[2], "/sensor/pandar/points");
#else
	tools::CompareICPandNDT compare_icp_and_ndt(argv[1], argv[2]);
#endif

	if(strcmp(argv[3] , "-ndt") == 0) {
		clock_t start = clock();
		compare_icp_and_ndt.SaveOdomToFileUsingNDT();
		std::cout << "total time: " << (clock()- start) * 1000/CLOCKS_PER_SEC << std::endl;
	}
	else if(strcmp(argv[3] , "-icp") == 0) {
		clock_t start = clock();
		compare_icp_and_ndt.SaveOdomToFileUsingICP();
		std::cout << "total time: " << (clock()- start) * 1000/CLOCKS_PER_SEC << std::endl;
	}
	else if(strcmp(argv[3] , "-loam") == 0) compare_icp_and_ndt.SaveLoamOdomToFile();
	else {
		show_usgae();
		exit(-1);
	}
	return 0;
}
