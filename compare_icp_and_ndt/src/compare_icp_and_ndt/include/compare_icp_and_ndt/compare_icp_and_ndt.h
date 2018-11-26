#ifndef COMPARE_ICP_AND_NDT_
#define COMPARE_ICP_AND_NDT_

#include <string>
#include <rosbag/bag.h>

#include <fast_pcl/ndt_gpu/NormalDistributionsTransform.h>

namespace tools {
	class CompareICPandNDT {
	 public :
		 CompareICPandNDT(const std::string& bag_file_path, const std::string& output_odom_path, 
							const std::string& points_topic_name = "/sensor/velodyne/points",
							double voxel_leaf_size = 0.1) {
			bag_file_path_ = bag_file_path;
			output_odom_path_ = output_odom_path;
			//using_icp_ = using_icp;
			points_topic_name_ = points_topic_name;
			voxel_leaf_size_ = voxel_leaf_size;
		 };

		 void SaveOdomToFileUsingNDT();
		 void SaveOdomToFileUsingICP();
		 void SaveOdomToFileUsingGICP();
		 void SaveOdomToFileUsingLOAM_match();
		 void SaveLoamOdomToFile();
	 private :
		 rosbag::Bag bag_;
		 std::string bag_file_path_;
		 std::string output_odom_path_;
		 std::string points_topic_name_;
		 //bool using_icp_;
		 // Added for GPU ndt
		 //gpu::GNormalDistributionsTransform gpu_ndt_;

		 //voxel leaf size
		 double voxel_leaf_size_;

	};
}
#endif
