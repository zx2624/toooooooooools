#ifndef COMPARE_ICP_AND_NDT_
#define COMPARE_ICP_AND_NDT_

#include <compare_icp_and_ndt/correspondence_estimation_with_label.h>

#include <string>
#include <rosbag/bag.h>

#include <fast_pcl/ndt_gpu/NormalDistributionsTransform.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>


namespace tools {
	struct Config {
		std::string bag_path;
		std::string output_odom_path;
		std::string lidar_point_topic;
		std::string lidar_label_topic;

		std::string method;

		bool do_vx_fil;
		double vx_leaf_size;

		bool read_gt_odom;
		std::string gt_odom_topic;

		int max_test_frame_num;

		int label_size;
		std::vector<pcl::registration::HobotLabels> valid_label_vec;

		void loadConfig(char *file);
	};
	
	class CompareICPandNDT {
	 public :
		 CompareICPandNDT(const Config& config) {
			bag_file_path_ = config.bag_path;
			output_odom_path_ = config.output_odom_path;
			points_topic_name_ = config.lidar_point_topic;
			label_points_topic_name_ = config.lidar_label_topic;

			method_ = config.method;

			voxel_leaf_size_ = config.vx_leaf_size;
			do_vx_fil_ = config.do_vx_fil;
			
			read_gt_odom_ = config.read_gt_odom;
			gt_odom_topic_ = config.gt_odom_topic;

			max_test_frame_num_ = config.max_test_frame_num;
			label_size_ = config.label_size;
			valid_label_vec_ = config.valid_label_vec;
		 };

		 void registration() {
			 if (method_ == "NDT") {
				 regis_ = boost::make_shared<pcl::NormalDistributionsTransform<
						pcl::PointXYZI, pcl::PointXYZI> >();
				 re_odom_name_ = "/NDT.txt";

				 std::cout << "Using NDT!\n";
			 }
			 else if (method_ == "ICP") {
				 regis_ = boost::make_shared<pcl::IterativeClosestPoint<
						pcl::PointXYZI, pcl::PointXYZI> >();
				 re_odom_name_ = "/ICP.txt";

				 std::cout << "Using ICP!\n";
			 }
			 else if (method_ == "GICP") {
				 regis_ = boost::make_shared<pcl::GeneralizedIterativeClosestPoint<
						pcl::PointXYZI, pcl::PointXYZI> >();
				 re_odom_name_ = "/GICP.txt";

				 std::cout << "Using GICP!\n";
			 }
			 else if (method_ == "SemanticICP") {
				 regis_ = boost::make_shared<pcl::IterativeClosestPoint<
						pcl::PointXYZI, pcl::PointXYZI> >();
				 re_odom_name_ = "/SemanticICP.txt";

				 std::cout << "Using SemanticICP!\n";
			 }
			 else {
				 std::cerr << "Yout method: " << method_ << "is wrong!\n";
				 return;
			 }
			 SaveOdomToFile();
		 }

		 void SaveGTOdomToFile();

	 private :
		 void SaveOdomToFile();

		 //void SaveOdomToFileUsingLOAM_match();

		 rosbag::Bag bag_;

		 std::string bag_file_path_;
		 std::string output_odom_path_;
		 std::string points_topic_name_;
		 std::string label_points_topic_name_;
		
		 std::string method_;

		 //bool using_icp_;
		 // Added for GPU ndt
		 //gpu::GNormalDistributionsTransform gpu_ndt_;

		 //whether do voxel filtering
		 bool do_vx_fil_;
		 //voxel leaf size
		 double voxel_leaf_size_;

		 bool read_gt_odom_;
		 std::string gt_odom_topic_;

		 std::string re_odom_name_;

		 int max_test_frame_num_;
		 int label_size_;
		 std::vector<pcl::registration::HobotLabels> valid_label_vec_;
		 pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr regis_;
	};
}
#endif
