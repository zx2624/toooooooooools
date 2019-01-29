// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     cmp_cam_lidar_pd.cc
// ROLE:     TODO (some explanation)
// CREATED:  2019-01-25 15:16:42
// MODIFIED: 2019-01-29 17:22:20
#include <cmp_cam_lidar_pd/cmp_cam_lidar_pd.h>
#include <cmp_cam_lidar_pd/point_type.h>

#include <iostream>
#include <string>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h> 

#include <mapping_tools/mapping_tools.h>

#include <yaml-cpp/yaml.h>

#include <lidar_motion_compensation/lidar_motion_compensation.h>

typedef pcl::PointXYZRGBLCov PointCamEx;
//typedef horizon::mapping::evaluation::PointXYZRGBLC PointCam;
//TODO use x, y, z, r, g, b, normal[0], normal[1] to store x, y, z, r, g, b, label, cov
//typedef pcl::PointXYZRGBLC PointCam;
typedef pcl::PointXYZRGBNormal PointCam;

typedef pcl::Label PointLabel;
typedef pcl::PointXYZRGB PointRGB;

#ifdef Pandora
typedef pcl::PointXYZRGBL PointLidar;
#else
typedef pcl::PointXYZI PointLidarI;
//typedef horizon::mapping::evaluation::PointXYZL PointLidar;
typedef pcl::PointXYZRGBL PointLidar;
#endif

const unsigned char INVALID_LABEL = 255;
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
    {0, 0, 142},
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
 
class Config {
 public:
	std::string bag_path;

	std::string lidar_odom_topic;
	std::string lidar_point_topic;
	std::string lidar_label_topic;

	std::string cam_point_topic;

	int min_pts_at_line;
	int min_pts_at_plane;

	double cam_pt_cov;

	std::vector<horizon::mapping::evaluation::HobotLabels> cmp_label_vec;

	std::string result_file;

	double max_dis;

	double min_z_value;

	double leaf_size;

	int min_pts_each_voxel;
};

void load_config(char *file, Config& config) {
	YAML::Node yaml = YAML::LoadFile(file);

	config.bag_path = yaml["bag_path"].as<std::string>();

	config.lidar_odom_topic = yaml["lidar_odom_topic"].as<std::string>();
	config.lidar_point_topic = yaml["lidar_point_topic"].as<std::string>();
	config.lidar_label_topic = yaml["lidar_label_topic"].as<std::string>();

	config.cam_point_topic = yaml["cam_point_topic"].as<std::string>();

	config.min_pts_at_line = yaml["min_pts_at_line"].as<int>();
	config.min_pts_at_plane = yaml["min_pts_at_plane"].as<int>();

	config.cam_pt_cov = yaml["cam_pt_cov"].as<double>();

	for (auto i = 0; i < yaml["cmp_label"].size(); i++) {
		auto label = static_cast<horizon::mapping::evaluation::HobotLabels>(
			yaml["cmp_label"][i].as<int>());
		config.cmp_label_vec.push_back(label);
	}

	config.result_file = yaml["result_file"].as<std::string>();

	config.max_dis = yaml["max_dis"].as<double>();

	config.min_z_value = yaml["min_z_value"].as<double>();

	config.leaf_size = yaml["leaf_size"].as<double>();
	config.min_pts_each_voxel = yaml["min_pts_each_voxel"].as<double>();
}
int get_camera_params_no_pandora(std::string contents,
		Eigen::Matrix4d &ex_cam_lidar) {
  std::cout << "Parse Camera Calibration..." << std::endl;
  if (contents.empty()) {
    std::cout << "string is empty" << std::endl;
    return -1;
  }

  YAML::Node yn = YAML::LoadFile(contents);

  std::cout << "**************ParseCameraCalibration contents: \n" << contents << std::endl;

	if (yn["extrinsic"].IsDefined()) {
	// get camera
		for (auto i = 0; i < yn["extrinsic"].size(); i++) {	
			if (yn["extrinsic"][i]["from"].IsDefined() && 
				yn["extrinsic"][i]["from"].as<std::string>() == "/sensor/velodyne/points" &&
				yn["extrinsic"][i]["to"].IsDefined() && 
				yn["extrinsic"][i]["to"].as<std::string>() == "/sensor/hugo1/image1/compressed") {

				ex_cam_lidar(0, 3) = yn["extrinsic"][i]["translation"]["x"].as<double>();
				ex_cam_lidar(1, 3) = yn["extrinsic"][i]["translation"]["y"].as<double>();
				ex_cam_lidar(2, 3) = yn["extrinsic"][i]["translation"]["z"].as<double>();
				
				double roll = yn["extrinsic"][i]["rotation"]["r"].as<double>();
				double pitch = yn["extrinsic"][i]["rotation"]["p"].as<double>();
				double yaw = yn["extrinsic"][i]["rotation"]["y"].as<double>();

				ex_cam_lidar.block<3, 3>(0, 0) =
					Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();

				break;
			}
		} 
	}
	else {
		printf("invalid intrinsicFile content\n");
		return -1;
	}
  return 0;
}
void cacMeanAndStd(const std::vector<std::vector<double>>& mean_vec_vec, 
		std::vector<double>& mean_vec, std::vector<double>& std_vec, int& valid_pt_size) {
	auto cnt = 0;
	for (auto i = 0; i < mean_vec_vec.size(); i++) {
			if (std::isnan(mean_vec_vec[i][0]) ||
				std::isnan(mean_vec_vec[i][1]) ||
				std::isnan(mean_vec_vec[i][2])) {
				continue;
			}
			mean_vec[0] += mean_vec_vec[i][0];
			mean_vec[1] += mean_vec_vec[i][1];
			mean_vec[2] += mean_vec_vec[i][2];
			cnt++;
	}
	mean_vec[0] /= cnt;
	mean_vec[1] /= cnt;
	mean_vec[2] /= cnt;

	for (auto i = 0; i < mean_vec_vec.size(); i++) {
			if (std::isnan(mean_vec_vec[i][0]) ||
				std::isnan(mean_vec_vec[i][1]) ||
				std::isnan(mean_vec_vec[i][2])) {
				continue;
			}
			std_vec[0] += pow(mean_vec_vec[i][0] - mean_vec[0], 2);
			std_vec[1] += pow(mean_vec_vec[i][1] - mean_vec[1], 2);
			std_vec[2] += pow(mean_vec_vec[i][2] - mean_vec[2], 2);
	}
	std_vec[0] = sqrt(std_vec[0]) / cnt;
	std_vec[1] = sqrt(std_vec[1]) / cnt;
	std_vec[2] = sqrt(std_vec[2]) / cnt;
	LOG(INFO) << "Valid pair size: " << cnt;
	LOG(INFO) << "All pair size: " << mean_vec_vec.size();
	valid_pt_size = cnt;
}

void cmp_cam_lidar_pd(Config &config, const Eigen::Matrix4d ex_cam_lidar) {
	using namespace horizon::mapping;
	const double EPS = 0.01;
	const double TIME_EACH_LIDAR = 0.10;
	const double MAX_DIFF_TIME_LIDAR_CAM = TIME_EACH_LIDAR / 2;
	//store bags
	std::vector<std::string> vec_bag_files;
	//get all bags
	getAllBagFilesPath(config.bag_path, vec_bag_files);

	std::ofstream ofs(config.result_file);
	ofs << 
		"bag_file [mean_x, mean_y, mean_z] [std_x, std_y, std_z] [valid_size/all_size] [cov, max_dis, leaf_size, min_pts_each_voxel]\n"; 
#ifdef VIEW_POINTCLOUD
	pcl::PointCloud<PointRGB>::Ptr lidar_pd_rgb(new pcl::PointCloud<PointRGB>);
	pcl::PointCloud<PointRGB>::Ptr cam_pd_rgb(new pcl::PointCloud<PointRGB>);
	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.addCoordinateSystem(1.0 ,"coor");
	viewer.initCameraParameters();
	viewer.setCameraPosition(0.0, 0.0, 10.0, 0.0, 1.0, 0.0);
	//viewer.addArrow(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 10), 0, 255, 0, "s1");
	//viewer.addArrow(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 20), 0, 255, 0, "s2");
	viewer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 30), 0, 255, 0, "s3");
	viewer.addText3D("30 meter", pcl::PointXYZ(0, 0, 30), 1.0, 0, 255, 0, "t");
#endif

	for (auto &bag_file:vec_bag_files) {
		rosbag::Bag bag;
		bag.open(bag_file, rosbag::bagmode::Read);

		rosbag::View lidar_points_view(bag, rosbag::TopicQuery(config.lidar_point_topic));
		rosbag::View odoms_view(bag, rosbag::TopicQuery(config.lidar_odom_topic));
		rosbag::View label_view(bag, rosbag::TopicQuery(config.lidar_label_topic));
		rosbag::View cam_points_view(bag, rosbag::TopicQuery(config.cam_point_topic));

		rosbag::View::iterator lidar_points_view_iter = lidar_points_view.begin();
		rosbag::View::iterator label_view_iter = label_view.begin();
		rosbag::View::iterator odoms_view_iter = odoms_view.begin();
		rosbag::View::iterator cam_points_view_iter = cam_points_view.begin();

		//align lidar points and odoms
		while(lidar_points_view_iter != lidar_points_view.end() &&
			odoms_view_iter != odoms_view.end()) {
			auto pt = lidar_points_view_iter->instantiate<sensor_msgs::PointCloud2>();
			auto odom = odoms_view_iter->instantiate<nav_msgs::Odometry>();

			double pt_stamp = pt->header.stamp.toSec();
			double odom_stamp = odom->header.stamp.toSec();
			if (pt_stamp - odom_stamp > EPS) {
				odoms_view_iter++;
			}
			else if(odom_stamp - pt_stamp > EPS) {
				lidar_points_view_iter++;
			}
			else break;
		}

		std::deque<rosbag::View::iterator> lidar_points_it_vec;
		std::deque<rosbag::View::iterator> label_it_vec;

		std::vector<Eigen::Matrix4d> odom_vec;
		std::deque<Eigen::Matrix4d> delta_odom_vec;

		std::deque<double> lidar_stamp_vec;

		//align lidar points, odom and label
		{
			auto cnt = 0;
			while(label_view_iter != label_view.end() &&
				odoms_view_iter != odoms_view.end() &&
				lidar_points_view_iter != lidar_points_view.end()) {
				auto label = label_view_iter->instantiate<sensor_msgs::PointCloud2>();
				auto odom = odoms_view_iter->instantiate<nav_msgs::Odometry>();

				double label_stamp = label->header.stamp.toSec();
				double odom_stamp = odom->header.stamp.toSec();
				if (label_stamp - odom_stamp > EPS) {
					odoms_view_iter++;
					lidar_points_view_iter++;
					continue;
				}
				else if(odom_stamp - label_stamp > EPS) {
					label_view_iter++;
					continue;
				}
				
				lidar_points_it_vec.push_back(lidar_points_view_iter++);
				label_it_vec.push_back(label_view_iter++);

				auto odom_mat = getMatrix4dFromMsg(*odom);
				odom_vec.push_back(odom_mat);

				lidar_stamp_vec.push_back(odom->header.stamp.toSec());

				if (cnt > 0) {
					delta_odom_vec.push_back(odom_vec[cnt - 1].inverse() * odom_mat);
					std::cout << "\n" << delta_odom_vec[cnt - 1] << "\n";
				}

				cnt++;
				odoms_view_iter++;
			}
			if (delta_odom_vec.size() > 0) {
				//fill the first delta odom
				delta_odom_vec.push_front(delta_odom_vec[0]);
			}
		}
		
		std::cout << lidar_points_it_vec.size() << " " <<  label_it_vec.size() << " "
				<< odom_vec.size() << " " << delta_odom_vec.size() << " "
				<< lidar_stamp_vec.size() << "\n";
		//Thier size must equal
		CHECK(lidar_points_it_vec.size() == label_it_vec.size() &&
				label_it_vec.size() == odom_vec.size() &&
				odom_vec.size() == delta_odom_vec.size() &&
				delta_odom_vec.size() == lidar_stamp_vec.size()) << " ERROR!";

		std::deque<rosbag::View::iterator> cam_points_it_vec;
		std::deque<double> cam_stamp_vec;
		//used to record the aligned lidar data with cam data
		std::vector<int> lidar_aligned_vec;

		{
			auto lidar_idx = 0;

			while (lidar_idx < lidar_stamp_vec.size() &&
					cam_points_view_iter != cam_points_view.end()) {
				auto cam_pd = cam_points_view_iter->instantiate<sensor_msgs::PointCloud2>();
				auto cam_pd_stamp = cam_pd->header.stamp.toSec();

				if (cam_pd_stamp - lidar_stamp_vec[lidar_idx] > MAX_DIFF_TIME_LIDAR_CAM) {
					lidar_idx++;
					continue;
				}
				else if (lidar_stamp_vec[lidar_idx] - cam_pd_stamp > MAX_DIFF_TIME_LIDAR_CAM) {
					cam_points_view_iter++;
					continue;
				}
				cam_stamp_vec.push_back(cam_pd_stamp);
				cam_points_it_vec.push_back(cam_points_view_iter++);
				lidar_aligned_vec.push_back(lidar_idx);
			}
		}

		LOG(INFO) << "The max aligned num of Lidar data and Camera is:" 
			<< cam_stamp_vec.size();

		//store each frame's result
		std::vector<std::vector<double>> mean_vec_vec(cam_stamp_vec.size());

#ifdef OpenMP
	#pragma omp parallel for
#endif
		for (auto i = 0; i < cam_stamp_vec.size(); i++) {
			auto lidar_idx = lidar_aligned_vec[i];

			auto lidar_pd_msg = lidar_points_it_vec[lidar_idx]->
				instantiate<sensor_msgs::PointCloud2>();
			auto label_msg = label_it_vec[lidar_idx]->
				instantiate<sensor_msgs::PointCloud2>();

			auto cam_pd_msg = cam_points_it_vec[i]->
				instantiate<sensor_msgs::PointCloud2>();

			pcl::PointCloud<PointLidarI>::Ptr lidar_pd_raw(new pcl::PointCloud<PointLidarI>);
			pcl::PointCloud<PointLidar>::Ptr lidar_pd(new pcl::PointCloud<PointLidar>);
			pcl::PointCloud<PointLabel>::Ptr label_pd(new pcl::PointCloud<PointLabel>);
			pcl::PointCloud<PointCamEx>::Ptr cam_pd_ex(new pcl::PointCloud<PointCamEx>);
			pcl::PointCloud<PointCam>::Ptr cam_pd(new pcl::PointCloud<PointCam>);

			pcl::fromROSMsg(*lidar_pd_msg, *lidar_pd_raw); 
			pcl::fromROSMsg(*label_msg, *label_pd); 
			//pcl::fromROSMsg(*cam_pd_msg, *cam_pd); 
			pcl::fromROSMsg(*cam_pd_msg, *cam_pd_ex); 

			//tranform from PointCamEx to PointCam
			{
				auto cnt = 0;
				cam_pd->resize(cam_pd_ex->size());
				for (auto &temp : cam_pd_ex->points) {
					if (temp.z < config.min_z_value) continue;

					if (fabs(temp.x) < 2 && fabs(temp.y) < 2 && fabs(temp.z) < 2) continue;

					if (sqrt(pow(temp.x, 2) + pow(temp.y, 2) + pow(temp.z, 2)) > config.max_dis) continue;

					cam_pd->points[cnt].x = temp.x;
					cam_pd->points[cnt].y = temp.y;
					cam_pd->points[cnt].z = temp.z;
					cam_pd->points[cnt].rgb = temp.rgb;

					cam_pd->points[cnt].normal[0] = temp.label;
					cam_pd->points[cnt].normal[1] = temp.cov[2];
					cnt++;
				}
				cam_pd->resize(cnt);
			}

			CHECK(lidar_pd_raw->size() == label_pd->size()) 
				<< "Lidar point size != Label size";

			//do motion compensation
			//the mapping (ordered): cloud_out.points[i] = cloud_in.points[indices[i]]
			auto indices = horizon::mapping::LidarMotionCompensation::motionCompensation(
					delta_odom_vec[lidar_idx], lidar_pd_raw, TIME_EACH_LIDAR);

			//caculate lidar odom at the aligned cam time
			double ratio = (lidar_stamp_vec[lidar_idx] - cam_stamp_vec[i]) / TIME_EACH_LIDAR;

			std::cout.precision(2);
			std::cout << "[Lidar, Cam]: " << std::fixed << lidar_stamp_vec[lidar_idx]
				<< " " << cam_stamp_vec[i] << std::endl;
			LOG(INFO) << "[delta_odom_vec]: \n" << delta_odom_vec[lidar_idx];

			//transform from lidar to lidar at cam time
			Eigen::Matrix4d t_cam_lidar = Eigen::Matrix4d::Identity();
			Eigen::Quaterniond zero_quat(1, 0, 0, 0);
			Eigen::Quaterniond delta_quat;
			delta_quat = delta_odom_vec[lidar_idx].block<3, 3>(0, 0);

			if (ratio > 0) {
				zero_quat = zero_quat.slerp(ratio, delta_quat);

				t_cam_lidar.block<3, 3>(0, 0) = zero_quat.toRotationMatrix();
				t_cam_lidar(0, 3) = ratio * delta_odom_vec[lidar_idx](0, 3);
				t_cam_lidar(1, 3) = ratio * delta_odom_vec[lidar_idx](1, 3);
				t_cam_lidar(2, 3) = ratio * delta_odom_vec[lidar_idx](2, 3);
			}
			else {
				ratio = fabs(ratio);

				zero_quat = zero_quat.slerp(ratio, delta_quat);

				t_cam_lidar.block<3, 3>(0, 0) = zero_quat.toRotationMatrix();
				t_cam_lidar(0, 3) = ratio * delta_odom_vec[lidar_idx](0, 3);
				t_cam_lidar(1, 3) = ratio * delta_odom_vec[lidar_idx](1, 3);
				t_cam_lidar(2, 3) = ratio * delta_odom_vec[lidar_idx](2, 3);

				t_cam_lidar = t_cam_lidar.inverse().eval();
			}

			//set label
			{
				for (auto p_idx = 0; p_idx < lidar_pd_raw->size(); p_idx++) {
					auto temp = lidar_pd_raw->points[p_idx];
					//auto label = label_pd->points[p_idx];
					auto label = label_pd->points[indices[p_idx]];

					if (INVALID_LABEL == static_cast<unsigned char>(label.label)) continue;
					if (fabs(temp.x) < 2 && fabs(temp.y) < 2 && fabs(temp.z) < 2) continue;

					Eigen::Vector4d vec(temp.x, temp.y, temp.z, 1);
					//vec = t_cam_lidar.block<3, 3>(0, 0) * vec + t_cam_lidar.block<3, 1>(0, 0);

					auto rel = ex_cam_lidar * t_cam_lidar * vec;

					//if (sqrt(pow(rel(0), 2) + pow(rel(1), 2) + pow(rel(2), 2)) > config.max_dis) continue;

					PointLidar data;
					data.x = rel(0);
					data.y = rel(1);
					data.z = rel(2);
					data.label = label.label;

					lidar_pd->push_back(data);
				}
			}

			//compare cam and lidar pd
			evaluation::CmpCamLidarPd<PointCam, PointLidar> cmp(
				config.cmp_label_vec, config.cam_pt_cov, config.min_pts_at_line, 
				config.min_pts_at_plane, config.leaf_size, config.min_pts_each_voxel);

			cmp.setCamPd(cam_pd); 
			cmp.setLidarPd(lidar_pd);

			cmp.cacPoint2Line();
			auto mean_dis = cmp.getMeanDis();
			auto std_dis = cmp.getStdDis();

			LOG(INFO) << "[X mean, std]: " << mean_dis[0] << "(m) " << std_dis[0] << "(m)";
			LOG(INFO) << "[Y mean, std]: " << mean_dis[1] << "(m) " << std_dis[1] << "(m)";
			LOG(INFO) << "[Z mean, std]: " << mean_dis[2] << "(m) " << std_dis[2] << "(m)";

#ifdef VIEW_POINTCLOUD
			lidar_pd_rgb->clear();
			auto lidar_pd_vec = cmp.getLidarPdVec();

			PointRGB p_rgb;

			for (auto lidar_pd_vec_idx = 0; lidar_pd_vec_idx < lidar_pd_vec.size(); lidar_pd_vec_idx++) {
				for (auto p_idx = 0; p_idx < lidar_pd_vec[lidar_pd_vec_idx]->size(); p_idx++) {
					auto data = lidar_pd_vec[lidar_pd_vec_idx]->points[p_idx];

					auto label_int = static_cast<unsigned char>(data.label);
					p_rgb.x = data.x;
					p_rgb.y = data.y;
					p_rgb.z = data.z;

					p_rgb.r = colorlist[label_int][0];
					p_rgb.g = colorlist[label_int][1];
					p_rgb.b = colorlist[label_int][2];
					lidar_pd_rgb->push_back(p_rgb);
				}
			}

			cam_pd_rgb->clear();
			auto cam_pd_vec = cmp.getCamPdVec();

			for (auto cam_pd_vec_idx = 0; cam_pd_vec_idx < cam_pd_vec.size(); cam_pd_vec_idx++) {
				for (auto p_idx = 0; p_idx < cam_pd_vec[cam_pd_vec_idx]->size(); p_idx++) {
					auto data = cam_pd_vec[cam_pd_vec_idx]->points[p_idx];

					//auto label_int = static_cast<unsigned char>(data.label);
					auto label_int = static_cast<unsigned char>(data.normal[0]);

					p_rgb.x = data.x;
					p_rgb.y = data.y;
					p_rgb.z = data.z;

					//p_rgb.r = colorlist[label_int][0];
					//p_rgb.g = colorlist[label_int][1];
					//p_rgb.b = colorlist[label_int][2];

					//TODO for vis lane
					p_rgb.r = 255;
					p_rgb.g = 255;
					p_rgb.b = 255;

					cam_pd_rgb->push_back(p_rgb);
				}
			}

			viewer.removePointCloud("cam");
			viewer.removePointCloud("lidar");

			viewer.addPointCloud(cam_pd_rgb, "cam");
			viewer.addPointCloud(lidar_pd_rgb, "lidar");

			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
				3, "cam");

			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
				3, "lidar");
			//viewer.spinOnce(20);

			viewer.removeText3D("mean_x");
			viewer.removeText3D("mean_y");
			viewer.removeText3D("mean_z");

			viewer.addText3D("mean_x: " + std::to_string(mean_dis[0]), pcl::PointXYZ(25, 0, 30), 1, 
				0, 255, 255, "mean_x");
			viewer.addText3D("mean_y: " + std::to_string(mean_dis[1]), pcl::PointXYZ(25, 0, 25), 1, 
				0, 255, 255, "mean_y");
			viewer.addText3D("mean_z: " + std::to_string(mean_dis[2]), pcl::PointXYZ(25, 0, 20), 1, 
				0, 255, 255, "mean_z");

			viewer.spinOnce(20);
#endif

			mean_vec_vec[i] = mean_dis;
		}
		std::vector<double> mean_vec(3, 0);
		std::vector<double> std_vec(3, 0);
		int valid_size = 0;
		cacMeanAndStd(mean_vec_vec, mean_vec, std_vec, valid_size);

		//output to file
		ofs.precision(3);
		ofs << bag_file << std::fixed << 
			" [" << mean_vec[0] << ", " << mean_vec[1] << ", " << mean_vec[2] << "]" 
			" [" << std_vec[0] << ", " << std_vec[1] << ", " << std_vec[2] << "]" 
			" [" << valid_size << "/" << mean_vec_vec.size() << "]"
			" [" << config.cam_pt_cov << " " << config.max_dis << " "<< config.leaf_size << " " << config.min_pts_each_voxel
			<< "]\n"; 

		//output to cout

		std::cout.precision(3);
		std::cout << bag_file << std::fixed << 
			" [" << mean_vec[0] << ", " << mean_vec[1] << ", " << mean_vec[2] << "]" 
			" [" << std_vec[0] << ", " << std_vec[1] << ", " << std_vec[2] << "]" 
			" [" << valid_size << "/" << mean_vec_vec.size() << "]"
			" [" << config.cam_pt_cov << " " << config.max_dis << " "<< config.leaf_size << " " << config.min_pts_each_voxel
			<< "]\n"; 

		bag.close();
	}
	ofs.close();

}

int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);
	FLAGS_stderrthreshold = 0;

	if (argc < 3) {
		LOG(ERROR) << "rosrun cmp_cam_lidar_pd cmp_cam_lidar_pd_node config_file cam_param\n";
		exit(-1);
	}

	Config config;
	load_config(argv[1], config);

	Eigen::Matrix4d ex_cam_lidar = Eigen::Matrix4d::Identity();
	get_camera_params_no_pandora(std::string(argv[2]), ex_cam_lidar);

	cmp_cam_lidar_pd(config, ex_cam_lidar);
	return 0;
}
