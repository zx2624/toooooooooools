// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     cmp_cam_lidar_pd.cc
// ROLE:     TODO (some explanation)
// CREATED:  2019-01-25 15:16:42
// MODIFIED: 2019-01-25 18:35:32
#include <cmp_cam_lidar_pd/cmp_cam_lidar_pd.h>
#include <cmp_cam_lidar_pd/point_type.h>

#include <iostream>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>

#include <mapping_tools/mapping_tools.h>

#include <yaml-cpp/yaml.h>

class Config {
 public:
	std::string bag_path;

	std::string lidar_odom_topic;
	std::string lidar_point_topic;
	std::string lidar_label_topic;

	std::string cam_point_topic;
};

void load_config(char * file, Config& config) {
	YAML::Node yaml = YAML::LoadFile(file);

	config.bag_path = yaml["bag_path"].as<std::string>();

	config.lidar_odom_topic = yaml["lidar_odom_topic"].as<std::string>();
	config.lidar_point_topic = yaml["lidar_point_topic"].as<std::string>();
	config.label_point_topic = yaml["label_point_topic"].as<std::string>();

	config.cam_point_topic = yaml["cam_point_topic"].as<std::string>();
}

int main(int argc, char argv**) {
	if (argc < 3) {
		LOG(FATAL) << "rosrun cmp_cam_lidar_pd cmp_cam_lidar_pd_node config_file\n";
	}
	google::InitGoogleLogging(argv[0]);
	FLAGS_stderrthreshold = 0;

	return 0;
}
