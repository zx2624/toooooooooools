//Author: siyuan.yu

//#include  "compare_icp_and_ndt/compare_icp_and_ndt.h"
#include  <compare_icp_and_ndt/compare_icp_and_ndt.h>

#include <ctime>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <yaml-cpp/yaml.h> 

//#include <export/loc_export.h>

namespace tools {
		const int INVALID_LABEL_ID = 255;
    const int colorlist[24][3] = {
    {128, 64, 128},
    {244, 35, 232},
    {107, 142, 35},
    {152, 251, 152},
    {153, 153, 153},
    {220, 220, 0},
    {250, 170, 30},
    {200, 200, 128},
    {200, 200, 200},
    {220, 20, 60},
    {0, 0, 70},
    //{0, 0, 142},
    {0, 255, 0},
    {70, 70, 70},
    {190, 153, 153},
    {70, 130, 180},
    {0, 64, 64},
    {128, 128, 192},
    {192, 192, 0},
    {64, 192, 0},
    {128, 0, 192},
    {192, 192, 128},
    {255, 0, 0},
    {102, 102, 156},
    {0, 0, 230}
  };
	void Config::loadConfig(char *file) {
		YAML::Node yaml = YAML::LoadFile(file);

		bag_path = yaml["bag_path"].as<std::string>();

		output_odom_path = yaml["output_odom_path"].as<std::string>();
		lidar_point_topic = yaml["lidar_point_topic"].as<std::string>();
		lidar_label_topic = yaml["lidar_label_topic"].as<std::string>();

		method = yaml["method"].as<std::string>();

		do_vx_fil = yaml["do_voxel_filter"].as<bool>();
		vx_leaf_size = yaml["voxel_leaf_size"].as<double>();

		read_gt_odom = yaml["read_gt_odom"].as<bool>();
		gt_odom_topic = yaml["gt_odom_topic"].as<std::string>();

		max_test_frame_num = yaml["max_test_frame_num"].as<int>();

		label_size = yaml["label_size"].as<int>();

		for (auto i = 0; i < yaml["valid_label"].size(); i++) {
			auto label = static_cast<pcl::registration::HobotLabels>(
			yaml["valid_label"][i].as<int>());
			valid_label_vec.push_back(label);
		}

	}

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
			 //remove ground
			 //if (p.z < -1.5) //continue;
			 source_selected->push_back(p);
		}
	}

	inline bool isLabelValid(int label,
		const std::vector<pcl::registration::HobotLabels>& valid_label_vec) {
		for (auto i = 0; i < valid_label_vec.size(); i++) {
			if (label == valid_label_vec[i]) return true;
		}
		return false;
	}

	inline void setLabel(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
		pcl::PointCloud<pcl::Label>::Ptr label_pd, 
		const std::vector<pcl::registration::HobotLabels>& valid_label_vec) {

		auto cnt = 0;
		for(auto i = 0; i < source->points.size(); i++) {
			auto p = source->points[i];
			auto label = label_pd->points[i].label;

			//if (label == INVALID_LABEL_ID ) continue;
			if (!isLabelValid(label, valid_label_vec)) continue;

			p.intensity = label;
			source->points[cnt++] = p;
		}
		source->resize(cnt);
	}

	void CompareICPandNDT::SaveOdomToFileUsingNDT() {
		std::cout << "Using NDT!" << std::endl;

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";
		//if(!bag_.isOpen()) {
			//std::cerr << bag_file_path_ << "not exist!\n";
			//exit(-1);
		//}

		std::ofstream ofs((output_odom_path_ + "/ndt.txt").c_str());
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
	  ndt.setTransformationEpsilon(0.01);
	  ndt.setStepSize(0.1);
		ndt.setResolution(1);
	  ndt.setMaximumIterations(200);


		Eigen::Matrix4f transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f last_transform_source_to_target = Eigen::Matrix4f::Identity();
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

#ifdef VIEWER
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addCoordinateSystem(3.0 ,"coor");
		viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		viewer.initCameraParameters();
		viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#endif

		for(rosbag::View::iterator its = points_view.begin(); 
				its != points_view.end();its++) {
			if (count++ > max_test_frame_num_) break;;
			//if(count++ == 1000) break;
			if(count%5 != 0) continue;
			//get a data instantiate
			ros_pcd = its->instantiate<sensor_msgs::PointCloud2>();
			pcl::fromROSMsg(*ros_pcd, *source);

			select_points(source, source_selected);

			if(is_first_frame) {
				is_first_frame = false;
				*target = *source_selected;

				if (do_vx_fil_) {
					// Apply voxelgrid filter
					voxel_grid_filter.setInputCloud(target);
					std::cout << "Before filter size: " << target->points.size() << std::endl;
					voxel_grid_filter.filter(*target);
					std::cout << "After filter size: " << target->points.size() << std::endl;
				}

				continue;
			}
			//if(!using_icp_) {
			if (do_vx_fil_) {
				voxel_grid_filter.setInputCloud(source_selected);
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;
				voxel_grid_filter.filter(*source_selected);
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;
			}

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
#ifdef VIEWER
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
#endif

				//set the source to target for next loop
				*target = *source_selected;

				std::cout << "| has_converged | fitness_score | final_num_iteration| \n";
				std::cout << has_converged << " | "
									<< fitness_score << " | "
									<< final_num_iteration << " | \n";
			//}

			Eigen::Quaternionf quat(transform_start_to_now.block<3, 3>(0, 0));
			Eigen::Vector3d rpy(0, 0, 0);
			tf::Matrix3x3(tf::Quaternion(
						quat.x(),
						quat.y(),
						quat.z(),
						quat.w()
						)).getRPY(rpy(0), rpy(1), rpy(2));
			ofs << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_start_to_now(0, 3) << " "
					<< transform_start_to_now(1, 3) << " "
					<< transform_start_to_now(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			std::cout << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_start_to_now(0, 3) << " "
					<< transform_start_to_now(1, 3) << " "
					<< transform_start_to_now(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			std::cout << "frame: " << count << std::endl;
			std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC << 
				"ms" << std::endl;

			//set 100 times to loop
			//if(count == 100 * 5 +1) break;
		}
		double sum_time = 0.0;
		for(unsigned int i = 0;i < time_count.size(); i++) {
			sum_time += time_count[i];
		}
		std::sort(time_count.begin(), time_count.end());
		std::cout << "average_time: " << sum_time / time_count.size() << "ms" << std::endl;
		std::cout << "middle_time: " << time_count[time_count.size()/2] << "ms" << std::endl;

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";
	}

	void CompareICPandNDT::SaveOdomToFileUsingICP() {
		std::cout << "Using ICP!" << std::endl;

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";

		std::ofstream ofs((output_odom_path_ + "/ICP.txt").c_str());
		if(!ofs.is_open()) {
			std::cerr << output_odom_path_ << "not exist!\n";
			exit(-1);
		}

		rosbag::View points_view(bag_, rosbag::TopicQuery(points_topic_name_));
		rosbag::View label_points_view(bag_, rosbag::TopicQuery(label_points_topic_name_));

		bool is_first_frame = true;

		// define voxelgrid filter
		pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI, float> icp;

		icp.setMaxCorrespondenceDistance(5);
		icp.setMaximumIterations(50);
		icp.setTransformationEpsilon(0.01);
	  icp.setEuclideanFitnessEpsilon(0.1);

		Eigen::Matrix4f transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f last_transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform_start_to_now = Eigen::Matrix4f::Identity();

		sensor_msgs::PointCloud2ConstPtr ros_pcd; 
		sensor_msgs::PointCloud2ConstPtr ros_label_pcd; 

		pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> aligned;

		pcl::PointCloud<pcl::Label>::Ptr label_pd(new pcl::PointCloud<pcl::Label>);

		int64_t count = 0;
		bool has_converged = false;
		double fitness_score = 0.0;

		clock_t start = clock(), end = clock();
		//statistic time 
		std::vector<double> time_count;
#ifdef VIEWER
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addCoordinateSystem(3.0 ,"coor");
		viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		viewer.initCameraParameters();
		viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#endif

		for(rosbag::View::iterator its = points_view.begin(); 
				its != points_view.end();its++) {
			if (count++ > max_test_frame_num_) break;
			//get a data instantiate
			ros_pcd = its->instantiate<sensor_msgs::PointCloud2>();
			pcl::fromROSMsg(*ros_pcd, *source);

			select_points(source, source_selected);

			if(is_first_frame) {
				is_first_frame = false;
				*target = *source_selected;

				if (do_vx_fil_) {
					std::cout << "Before filter size: " << target->points.size() << std::endl;
					// Apply voxelgrid filter
					voxel_grid_filter.setInputCloud(target);
					voxel_grid_filter.filter(*target);
					std::cout << "After filter size: " << target->points.size() << std::endl;
				}

				continue;
			}

			if (do_vx_fil_) {
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;
				voxel_grid_filter.setInputCloud(source_selected);
				voxel_grid_filter.filter(*source_selected);
				std::cout << "After filter size: " << source_selected->points.size() << std::endl;
			}

				icp.setInputTarget(target);
				std::cout << "Target pointcloud size: " << target->points.size() << std::endl;

				icp.setInputSource(source_selected);
				std::cout << "Source pointcloud size: " << source_selected->points.size() << std::endl;

				start = clock();

				icp.align(aligned);
				end = clock();
				time_count.push_back((end - start) * 1000/CLOCKS_PER_SEC);//get millisecond

				transform_source_to_target = icp.getFinalTransformation();

				last_transform_source_to_target = transform_source_to_target;

				has_converged = icp.hasConverged();
				fitness_score = icp.getFitnessScore();
			
				transform_start_to_now = transform_start_to_now * transform_source_to_target;

#ifdef VIEWER
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
#endif

				//set the source to target for next loop
				*target = *source_selected;

				std::cout << "| has_converged | fitness_score | final_num_iteration| \n";
				std::cout << has_converged << " | "
									<< fitness_score << " | \n";

			Eigen::Quaternionf quat(transform_start_to_now.block<3, 3>(0, 0));
			Eigen::Vector3d rpy(0, 0, 0);
			tf::Matrix3x3(tf::Quaternion(
						quat.x(),
						quat.y(),
						quat.z(),
						quat.w()
						)).getRPY(rpy(0), rpy(1), rpy(2));

			ofs << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_start_to_now(0, 3) << " "
					<< transform_start_to_now(1, 3) << " "
					<< transform_start_to_now(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			std::cout << "T_0_i : "
					<< std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_start_to_now(0, 3) << " "
					<< transform_start_to_now(1, 3) << " "
					<< transform_start_to_now(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			quat = transform_source_to_target.block<3, 3>(0, 0);
			tf::Matrix3x3(tf::Quaternion(
						quat.x(),
						quat.y(),
						quat.z(),
						quat.w()
						)).getRPY(rpy(0), rpy(1), rpy(2));

			std::cout << "T_i_i+1 : "
					<< std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_source_to_target(0, 3) << " "
					<< transform_source_to_target(1, 3) << " "
					<< transform_source_to_target(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			std::cout << "frame: " << count << std::endl;
			std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC 
				<< "ms" << std::endl;

		}

		double sum_time = 0.0;
		for(unsigned int i = 0;i < time_count.size(); i++) {
			sum_time += time_count[i];
		}
		std::sort(time_count.begin(), time_count.end());
		std::cout << "average_time: " << sum_time / time_count.size() << "ms" 
			<< std::endl;
		std::cout << "middle_time: " << time_count[time_count.size()/2] << "ms"
			<< std::endl;

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";
	}

	void CompareICPandNDT::SaveOdomToFileUsingICPWithLabel() {
		std::cout << "Using Semantic ICP!" << std::endl;

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";

		std::ofstream ofs((output_odom_path_ + "/SemanticICP.txt").c_str());
		if(!ofs.is_open()) {
			std::cerr << output_odom_path_ << "not exist!\n";
			exit(-1);
		}

		rosbag::View points_view(bag_, rosbag::TopicQuery(points_topic_name_));
		rosbag::View label_points_view(bag_, rosbag::TopicQuery(label_points_topic_name_));

		bool is_first_frame = true;

		// define voxelgrid filter
		pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

		Eigen::Matrix4f transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f last_transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform_start_to_now = Eigen::Matrix4f::Identity();

		sensor_msgs::PointCloud2ConstPtr ros_pcd; 
		sensor_msgs::PointCloud2ConstPtr ros_label_pcd; 

		pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);

		pcl::PointCloud<pcl::Label>::Ptr label_pd(new pcl::PointCloud<pcl::Label>);

		pcl::PointCloud<pcl::PointXYZI> aligned;

		pcl::registration::CorrespondenceEstimationWithLabel<pcl::PointXYZI, pcl::PointXYZI>::Ptr
			corr_es_with_label(new 
			pcl::registration::CorrespondenceEstimationWithLabel<pcl::PointXYZI, pcl::PointXYZI>
			(label_size_, valid_label_vec_));

		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI, float> icp;

		icp.setMaxCorrespondenceDistance(5);
		icp.setMaximumIterations(50);
		icp.setTransformationEpsilon(0.01);
	  icp.setEuclideanFitnessEpsilon(0.1);

		//icp.setCorrespondenceEstimation(corr_es_with_label);

		int64_t count = 0;
		bool has_converged = false;
		double fitness_score = 0.0;

		clock_t start = clock(), end = clock();
		//statistic time 
		std::vector<double> time_count;
#ifdef VIEWER
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addCoordinateSystem(3.0 ,"coor");
		viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		viewer.initCameraParameters();
		viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#endif
		rosbag::View::iterator its = points_view.begin();
		rosbag::View::iterator label_its = label_points_view.begin();

		const double EPS = 0.01;
		while (its != points_view.end() && label_its != label_points_view.end()) {
			if (count++ > max_test_frame_num_) break;
			//get a data instantiate
			ros_pcd = its->instantiate<sensor_msgs::PointCloud2>();
			ros_label_pcd = label_its->instantiate<sensor_msgs::PointCloud2>();

			if (ros_pcd->header.stamp.toSec() - ros_label_pcd->header.stamp.toSec() > EPS) {
				label_its++;	
				continue;
			}
			else if (ros_pcd->header.stamp.toSec() - ros_label_pcd->header.stamp.toSec() < -EPS) {
				its++;	
				continue;
			}
			its++;
			label_its++;

			std::cout.precision(2);
			std::cout << std::fixed
				<< "pd stamp: " << ros_pcd->header.stamp.toSec()
				<< " pd label stamp: " << ros_label_pcd->header.stamp.toSec() << std::endl;

			pcl::fromROSMsg(*ros_pcd, *source);
			pcl::fromROSMsg(*ros_label_pcd, *label_pd);
			
			assert(source->size() == label_pd->size());

			//set Label to Intensity
			std::cout << "Before filter label: " << source->points.size() << std::endl;

			setLabel(source, label_pd, valid_label_vec_);

			std::cout << "After filter label: " << source->points.size() << std::endl;

			select_points(source, source_selected);

			if(is_first_frame) {
				is_first_frame = false;
				*target = *source_selected;

				if (do_vx_fil_) {
					std::cout << "Before filter size: " << target->points.size() << std::endl;
					// Apply voxelgrid filter
					voxel_grid_filter.setInputCloud(target);
					voxel_grid_filter.filter(*target);
					std::cout << "After filter size: " << target->points.size() << std::endl;
				}

			//corr_es_with_label->setInputTarget(target);

				continue;
			}
			if (do_vx_fil_) {
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;
				voxel_grid_filter.setInputCloud(source_selected);
				voxel_grid_filter.filter(*source_selected);
				std::cout << "After filter size: " << source_selected->points.size() << std::endl;
			}

				icp.setInputTarget(target);
				std::cout << "Target pointcloud size: " << target->points.size() << std::endl;

				icp.setInputSource(source_selected);
				std::cout << "Source pointcloud size: " << source_selected->points.size() << std::endl;

				start = clock();

				icp.align(aligned);

				end = clock();
				time_count.push_back((end - start) * 1000/CLOCKS_PER_SEC);//get millisecond

				transform_source_to_target = icp.getFinalTransformation();

				last_transform_source_to_target = transform_source_to_target;

				has_converged = icp.hasConverged();
				fitness_score = icp.getFitnessScore();
			
				transform_start_to_now = transform_start_to_now * transform_source_to_target;

				std::cout << "| has_converged | fitness_score | final_num_iteration| \n";
				std::cout << has_converged << " | "
									<< fitness_score << " | \n";

			Eigen::Quaternionf quat(transform_start_to_now.block<3, 3>(0, 0));
			Eigen::Vector3d rpy(0, 0, 0);
			tf::Matrix3x3(tf::Quaternion(
						quat.x(),
						quat.y(),
						quat.z(),
						quat.w()
						)).getRPY(rpy(0), rpy(1), rpy(2));

			ofs << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_start_to_now(0, 3) << " "
					<< transform_start_to_now(1, 3) << " "
					<< transform_start_to_now(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			std::cout << "T_0_i : "
					<< std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_start_to_now(0, 3) << " "
					<< transform_start_to_now(1, 3) << " "
					<< transform_start_to_now(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			quat = transform_source_to_target.block<3, 3>(0, 0);
			tf::Matrix3x3(tf::Quaternion(
						quat.x(),
						quat.y(),
						quat.z(),
						quat.w()
						)).getRPY(rpy(0), rpy(1), rpy(2));

			std::cout << "T_i_i+1 : "
					<< std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_source_to_target(0, 3) << " "
					<< transform_source_to_target(1, 3) << " "
					<< transform_source_to_target(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

#ifdef VIEWER
				show_cloud->clear();
				for(size_t i = 0; i < target->points.size(); i++) {
					pcl::PointXYZI p = target->points[i];

					auto label = static_cast<int>(p.intensity);

		      pcl::PointXYZRGB p1;
		      p1.x = p.x;
		      p1.y = p.y;
		      p1.z = p.z;
		      p1.r = tools::colorlist[label][0];
		      p1.g = tools::colorlist[label][1];
		      p1.b = tools::colorlist[label][2];

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
					p1.g = 255;
					p1.b = 255;
					show_cloud->push_back(p1);
				}

				viewer.removePointCloud("cloud");
        viewer.addPointCloud(show_cloud, "cloud");
				viewer.setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
					3, "cloud");

				viewer.removeText3D("delta_x");
				viewer.removeText3D("delta_y");
				viewer.removeText3D("delta_z");

				viewer.addText3D("delta_x: " + std::to_string(
					transform_source_to_target(0, 3)), pcl::PointXYZ(20, 20, 20), 1,
					0, 255, 255, "delta_x");
				viewer.addText3D("delta_y: " + std::to_string(
					transform_source_to_target(1, 3)), pcl::PointXYZ(22, 22, 22), 1,
					0, 255, 255, "delta_y");
				viewer.addText3D("delta_z: " + std::to_string(
					transform_source_to_target(2, 3)), pcl::PointXYZ(24, 24, 24), 1,
					0, 255, 255, "delta_z");

				viewer.spinOnce(10);
#endif

			//set the source to target for next loop
			*target = *source_selected;

			std::cout << "frame: " << count << std::endl;
			std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC 
				<< "ms" << std::endl;
		}

		double sum_time = 0.0;
		for(unsigned int i = 0;i < time_count.size(); i++) {
			sum_time += time_count[i];
		}
		std::sort(time_count.begin(), time_count.end());
		std::cout << "average_time: " << sum_time / time_count.size() << "ms" 
			<< std::endl;
		std::cout << "middle_time: " << time_count[time_count.size()/2] << "ms"
			<< std::endl;

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";
	}

	void CompareICPandNDT::SaveOdomToFileUsingGICP() {
		std::cout << "Using GICP!" << std::endl;

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";

		std::ofstream ofs((output_odom_path_ + "/GICP.txt").c_str());
		if(!ofs.is_open()) {
			std::cerr << output_odom_path_ << "not exist!\n";
			exit(-1);
		}

		rosbag::View points_view(bag_, rosbag::TopicQuery(points_topic_name_));
		bool is_first_frame = true;


		// define voxelgrid filter
		pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
		icp.setMaxCorrespondenceDistance(5);
		icp.setMaximumIterations(50);
		icp.setTransformationEpsilon(0.01);
	  icp.setEuclideanFitnessEpsilon(0.1);

		Eigen::Matrix4f transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f last_transform_source_to_target = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform_start_to_now = Eigen::Matrix4f::Identity();

		sensor_msgs::PointCloud2ConstPtr ros_pcd; 
		pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> aligned;

		int64_t count = 0;
		bool has_converged = false;
		//int final_num_iteration = 0;
		double fitness_score = 0.0;

		clock_t start = clock(), end = clock();
		//statistic time 
		std::vector<double> time_count;
#ifdef VIEWER
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addCoordinateSystem(3.0 ,"coor");
		viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		viewer.initCameraParameters();
		viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#endif

		for(rosbag::View::iterator its = points_view.begin(); 
				its != points_view.end();its++) {
			if (count++ > max_test_frame_num_) break;;
			//if(count++%5 != 0) continue;

			//get a data instantiate
			ros_pcd = its->instantiate<sensor_msgs::PointCloud2>();
			pcl::fromROSMsg(*ros_pcd, *source);

			select_points(source, source_selected);

			if(is_first_frame) {
				is_first_frame = false;
				*target = *source_selected;

				if (do_vx_fil_) {
					std::cout << "Before filter size: " << target->points.size() << std::endl;
					// Apply voxelgrid filter
					voxel_grid_filter.setInputCloud(target);
					voxel_grid_filter.filter(*target);
					std::cout << "After filter size: " << target->points.size() << std::endl;
				}

				continue;
			}
			//if(!using_icp_) {
			if (do_vx_fil_) {
				std::cout << "Before filter size: " << source_selected->points.size() << std::endl;
				voxel_grid_filter.setInputCloud(source_selected);
				voxel_grid_filter.filter(*source_selected);
				std::cout << "After filter size: " << source_selected->points.size() << std::endl;
			}

				icp.setInputTarget(target);
				std::cout << "Target pointcloud size: " << target->points.size() << std::endl;

				icp.setInputSource(source_selected);
				std::cout << "Source pointcloud size: " << source_selected->points.size() << std::endl;

				start = clock();

				icp.align(aligned);
				end = clock();
				time_count.push_back((end - start) * 1000/CLOCKS_PER_SEC);//get millisecond

				transform_source_to_target = icp.getFinalTransformation();

				last_transform_source_to_target = transform_source_to_target;

				has_converged = icp.hasConverged();
				fitness_score = icp.getFitnessScore();
			
				transform_start_to_now = transform_start_to_now * transform_source_to_target;

#ifdef VIEWER
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
#endif

				//set the source to target for next loop
				*target = *source_selected;

				std::cout << "| has_converged | fitness_score | final_num_iteration| \n";
				std::cout << has_converged << " | "
									<< fitness_score << " | \n";
			//}

			//ofs << std::fixed << std::setprecision(6)
					//<< ros_pcd->header.stamp.toSec() << " ";
			//for(int i = 0; i < 4; i++)
				//for(int j = 0; j < 4; j++)
					//ofs << std::fixed << std::setprecision(6) 
							//<< transform_start_to_now(i, j) << " ";
			//ofs << std::endl;
			Eigen::Quaternionf quat(transform_start_to_now.block<3, 3>(0, 0));
			Eigen::Vector3d rpy(0, 0, 0);
			tf::Matrix3x3(tf::Quaternion(
						quat.x(),
						quat.y(),
						quat.z(),
						quat.w()
						)).getRPY(rpy(0), rpy(1), rpy(2));

			ofs << std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_start_to_now(0, 3) << " "
					<< transform_start_to_now(1, 3) << " "
					<< transform_start_to_now(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			std::cout << "T_0_i : "
					<< std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_start_to_now(0, 3) << " "
					<< transform_start_to_now(1, 3) << " "
					<< transform_start_to_now(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			quat = transform_source_to_target.block<3, 3>(0, 0);
			tf::Matrix3x3(tf::Quaternion(
						quat.x(),
						quat.y(),
						quat.z(),
						quat.w()
						)).getRPY(rpy(0), rpy(1), rpy(2));

			std::cout << "T_i_i+1 : "
					<< std::fixed << std::setprecision(6)
					<< ros_pcd->header.stamp.toSec() << " "
				  << std::fixed << std::setprecision(3)
					<< transform_source_to_target(0, 3) << " "
					<< transform_source_to_target(1, 3) << " "
					<< transform_source_to_target(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			std::cout << "frame: " << count << std::endl;
			std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC 
				<< "ms" << std::endl;

			//set 100 times to loop
			//if(count == 100 * 5 + 1 ) break;
		}
		double sum_time = 0.0;
		for(unsigned int i = 0;i < time_count.size(); i++) {
			sum_time += time_count[i];
		}
		std::sort(time_count.begin(), time_count.end());
		std::cout << "average_time: " << sum_time / time_count.size() << "ms" 
			<< std::endl;
		std::cout << "middle_time: " << time_count[time_count.size()/2] << "ms"
			<< std::endl;

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";
	}

	//void CompareICPandNDT::SaveOdomToFileUsingLOAM_match() {
		//std::cout << "Using LOAM_match!" << std::endl;

		//bag_.open(bag_file_path_, rosbag::bagmode::Read);
		//std::cout << "open bag file success! \n";

		//std::ofstream ofs(output_odom_path_.c_str());
		//if(!ofs.is_open()) {
			//std::cerr << output_odom_path_ << "not exist!\n";
			//exit(-1);
		//}

		//rosbag::View points_view(bag_, rosbag::TopicQuery(points_topic_name_));
		//bool is_first_frame = true;

		////define loc param
		//LocParam param;
		//param.laser_type = "HDL32";
		//param.loc_mode = false;
		//param.use_gnss = false;
		//param.use_extrinsics = false;

		////init param
		//locStart(param);

		//Eigen::Matrix4f transform_source_to_target = Eigen::Matrix4f::Identity();
		//Eigen::Matrix4f last_transform_source_to_target = Eigen::Matrix4f::Identity();
		//Eigen::Matrix4f transform_start_to_now = Eigen::Matrix4f::Identity();

		//sensor_msgs::PointCloud2ConstPtr ros_pcd; 
		//pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
		//pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
		//pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
		//pcl::PointCloud<pcl::PointXYZI> aligned;

		//int64_t count = 0;
		//bool has_converged = false;
		////int final_num_iteration = 0;
		//double fitness_score = 0.0;

		//clock_t start = clock(), end = clock();
		////statistic time 
		//std::vector<double> time_count;
//#ifdef VIEWER
		//pcl::visualization::PCLVisualizer viewer("viewer");
		//viewer.addCoordinateSystem(3.0 ,"coor");
		//viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		//viewer.initCameraParameters();
		//viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//#endif

		//Eigen::Vector3d odom_r = Eigen::Vector3d::Zero();
		//Eigen::Vector3d odom_t = Eigen::Vector3d::Zero();

		//for(rosbag::View::iterator its = points_view.begin(); 
				//its != points_view.end();its++) {
			//count++;
			////if(count++%5 != 0) continue;
			////if(count++ == 1000) break;
			////get a data instantiate
			//ros_pcd = its->instantiate<sensor_msgs::PointCloud2>();
			//pcl::fromROSMsg(*ros_pcd, *source);

			//select_points(source, source_selected);

			//if(is_first_frame) {
				//is_first_frame = false;
				//*target = *source_selected;

				//continue;
			//}

			//std::cout << "Target pointcloud size: " << target->points.size() << std::endl;

			//std::cout << "Source pointcloud size: " << source_selected->points.size() << std::endl;

			//start = clock();
			//locRegistrationLaser(*target, *source_selected, odom_r, odom_t);
			//end = clock();

			//time_count.push_back((end - start) * 1000/CLOCKS_PER_SEC);//get millisecond

			//transform_source_to_target.block<3, 1>(0, 3) = odom_t.cast<float>();
			//transform_source_to_target.block<3, 3>(0, 0) = 
				//Eigen::AngleAxisf(float(odom_r(2)), Eigen::Vector3f::UnitZ()) * 
				//Eigen::AngleAxisf(float(odom_r(1)), Eigen::Vector3f::UnitY()) *
				//Eigen::AngleAxisf(float(odom_r(0)), Eigen::Vector3f::UnitX()).matrix();

			//last_transform_source_to_target = transform_source_to_target;

			//transform_start_to_now = transform_start_to_now * transform_source_to_target;

//#ifdef VIEWER
			//show_cloud->clear();
			//for(size_t i = 0; i < target->points.size(); i++) {
				//pcl::PointXYZI p = target->points[i];
				//pcl::PointXYZRGB p1;
				//p1.x = p.x;
				//p1.y = p.y;
				//p1.z = p.z;
				//p1.r = 0;
				//p1.g = 255;
				//p1.b = 0;
				//show_cloud->push_back(p1);
			//}
			//Eigen::Vector4f temp;
			//for(size_t i = 0; i < source_selected->points.size(); i++) {
				//pcl::PointXYZI p = source_selected->points[i];

				//temp << p.x, p.y, p.z, 1;
				//temp = transform_source_to_target * temp;

				//pcl::PointXYZRGB p1;
				//p1.x = temp(0);
				//p1.y = temp(1);
				//p1.z = temp(2);
				//p1.r = 255;
				//p1.g = 0;
				//p1.b = 0;
				//show_cloud->push_back(p1);
			//}
			//viewer.removePointCloud("cloud");
			//viewer.addPointCloud(show_cloud, "cloud");
			//viewer.spinOnce(10);
//#endif

			////set the source to target for next loop
			//*target = *source_selected;

			//Eigen::Quaternionf quat(transform_start_to_now.block<3, 3>(0, 0));
			//Eigen::Vector3d rpy(0, 0, 0);
			//tf::Matrix3x3(tf::Quaternion(
						//quat.x(),
						//quat.y(),
						//quat.z(),
						//quat.w()
						//)).getRPY(rpy(0), rpy(1), rpy(2));

			//ofs << std::fixed << std::setprecision(6)
					//<< ros_pcd->header.stamp.toSec() << " "
					//<< std::fixed << std::setprecision(3)
					//<< transform_start_to_now(0, 3) << " "
					//<< transform_start_to_now(1, 3) << " "
					//<< transform_start_to_now(2, 3) << " "
					//<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					//<< std::endl;

			//std::cout << "T_0_i : "
					//<< std::fixed << std::setprecision(6)
					//<< ros_pcd->header.stamp.toSec() << " "
					//<< std::fixed << std::setprecision(3)
					//<< transform_start_to_now(0, 3) << " "
					//<< transform_start_to_now(1, 3) << " "
					//<< transform_start_to_now(2, 3) << " "
					//<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					//<< std::endl;

			//std::cout << "T_i_i+1 : "
					//<< std::fixed << std::setprecision(6)
					//<< ros_pcd->header.stamp.toSec() << " "
					//<< std::fixed << std::setprecision(3)
					//<< transform_source_to_target(0, 3) << " "
					//<< transform_source_to_target(1, 3) << " "
					//<< transform_source_to_target(2, 3) << " "
					//<< odom_r(0) << " " << odom_r(1) << " " << odom_r(2)
					//<< std::endl;

			//std::cout << "frame: " << count << std::endl;
			//std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC 
				//<< "ms" << std::endl;

			////set 100 times to loop
			////if(count == 100 * 5 + 1 ) break;
		//}
		//double sum_time = 0.0;
		//for(unsigned int i = 0;i < time_count.size(); i++) {
			//sum_time += time_count[i];
		//}
		//std::sort(time_count.begin(), time_count.end());
		//std::cout << "average_time: " << sum_time / time_count.size() << "ms" 
			//<< std::endl;
		//std::cout << "middle_time: " << time_count[time_count.size()/2] << "ms"
			//<< std::endl;

		//bag_.close();
		//ofs.close();
		//std::cout << "save Finished!\n";
	//}

	void CompareICPandNDT::SaveGTOdomToFile() {
		std::cout << "Save loam odom to file!" << std::endl;

		bag_.open(bag_file_path_, rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";

		std::ofstream ofs((output_odom_path_ + "/GT.txt").c_str());
		if(!ofs.is_open()) {
			std::cerr << output_odom_path_ << "not exist!\n";
			exit(-1);
		}

		rosbag::View odoms_view(bag_, rosbag::TopicQuery(gt_odom_topic_));

		int64_t count = 0;
		bool is_first_frame = true;

		Eigen::Matrix4d init_odom_inv = Eigen::Matrix4d::Identity();

		for(rosbag::View::iterator its = odoms_view.begin(); 
				its != odoms_view.end();its++) {
			if(count++ == max_test_frame_num_) break;
			//get a data instantiate
			nav_msgs::OdometryConstPtr od = its->instantiate<nav_msgs::Odometry>(); 

			Eigen::Vector3d rpy(0, 0, 0);
			tf::Matrix3x3(tf::Quaternion(
						od->pose.pose.orientation.x, 
						od->pose.pose.orientation.y, 
						od->pose.pose.orientation.z, 
						od->pose.pose.orientation.w
						)).getRPY(rpy(0), rpy(1), rpy(2));

				Eigen::Matrix4d odom_mat = Eigen::Matrix4d::Identity();

				odom_mat(0, 3) = od->pose.pose.position.x;
				odom_mat(1, 3) = od->pose.pose.position.y;
				odom_mat(2, 3) = od->pose.pose.position.z;

				odom_mat.block<3, 3>(0, 0) = 
					(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())).matrix();

			if (is_first_frame) {
				init_odom_inv = odom_mat.inverse();
				is_first_frame = false;
			}

			odom_mat = init_odom_inv * odom_mat;

			Eigen::Quaterniond quat_temp;
			quat_temp = odom_mat.block<3, 3>(0, 0);

			tf::Matrix3x3(tf::Quaternion(
						quat_temp.x(), 
						quat_temp.y(), 
						quat_temp.z(), 
						quat_temp.w()
						)).getRPY(rpy(0), rpy(1), rpy(2));

			ofs << std::fixed << std::setprecision(6)
					<< od->header.stamp.toSec() << " "
					<< std::fixed << std::setprecision(3)
					<< odom_mat(0, 3) << " "
					<< odom_mat(1, 3) << " "
					<< odom_mat(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;

			std::cout << std::fixed << std::setprecision(6)
					<< od->header.stamp.toSec() << " "
					<< odom_mat(0, 3) << " "
					<< odom_mat(1, 3) << " "
					<< odom_mat(2, 3) << " "
					<< rpy(0) << " " << rpy(1) << " " << rpy(2)
					<< std::endl;
		}

		bag_.close();
		ofs.close();
		std::cout << "save Finished!\n";

	}
}
void show_usgae() {
	std::cout << "1: run icp method: rosrun compare_icp_and_ndt " << 
		"compare_icp_and_ndt_node bag_file_path save_odom_path -icp \n";
	std::cout << "2: run gicp method: rosrun compare_icp_and_ndt " << 
		"compare_icp_and_ndt_node bag_file_path save_odom_path -gicp \n";
	std::cout << "3: run ndt method: " <<
								"rosrun compare_icp_and_ndt compare_icp_and_ndt_node " <<
								"bag_file_path save_odom_path -ndt \n";
	std::cout << "4: save loam_match odom:" << 
								"rosrun compare_icp_and_ndt compare_icp_and_ndt_node "<<
								"bag_file_path save_odom_path -loam_match \n";
	std::cout << "5: save loam odom:" << 
								"rosrun compare_icp_and_ndt compare_icp_and_ndt_node "<<
								"bag_file_path save_odom_path -loam \n";
}

int main(int argc, char** argv) {
	if(argc < 2) {
		std::cerr 
			<< "rosrun compare_icp_and_ndt compare_icp_and_ndt_node config_file\n";
		exit(-1);
	}

	tools::Config config;
	config.loadConfig(argv[1]);

	tools::CompareICPandNDT compare_icp_and_ndt(config);

	clock_t start = clock();
	compare_icp_and_ndt.registration();
	std::cout << "total time: " << (clock()- start) * 1000/CLOCKS_PER_SEC << std::endl;

	if(config.read_gt_odom) {
		std::cout << "Read GT odom!\n";
		compare_icp_and_ndt.SaveGTOdomToFile();
	}

	return 0;
}
