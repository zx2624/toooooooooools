// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     cmp_cam_lidar_pd.cc
// ROLE:     TODO (some explanation)
// CREATED:  2019-01-21 13:50:53
// MODIFIED: 2019-01-24 15:13:26
#include <cmp_cam_lidar_pd/point_type.h>
#include <cmp_cam_lidar_pd/cmp_cam_lidar_pd.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

//#include <rosbag/bag.h>
//#include <rosbag/view.h>

//#include <sensor_msgs/CompressedImage.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <nav_msgs/Odometry.h>

//#include <pcl_conversions/pcl_conversions.h>
typedef horizon::mapping::evaluation::PointXYZRGBLCov PointCam;
typedef pcl::PointXYZRGBL PointLidar;
using namespace horizon::mapping::evaluation;

int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);
	FLAGS_stderrthreshold = 0;

	pcl::PointCloud<PointCam>::Ptr cam_cloud(new pcl::PointCloud<PointCam>);
	pcl::PointCloud<PointLidar>::Ptr lidar_cloud(new pcl::PointCloud<PointLidar>);

  // Fill in the cloud data
  cam_cloud->width  = 11 * 4;
  cam_cloud->height = 1;
  cam_cloud->points.resize(cam_cloud->width * cam_cloud->height);

  // Generate the data
	auto cnt = 0;
  for (size_t i = 0; i <= 10; ++i) {
		for (size_t j = 0; j <= 3; ++j, cnt++) {
			//cam_cloud->points[cnt].x = i * 0.1 + (rand () % 20) / 100.0 ;
			//cam_cloud->points[cnt].y = j * 0.1 + (rand () % 20) / 100.0 ;

			cam_cloud->points[cnt].x = i * 0.1;
			cam_cloud->points[cnt].y = j * 0.1;
			cam_cloud->points[cnt].z = 10;

			cam_cloud->points[cnt].label = 1;
			cam_cloud->points[cnt].cov[3] = 0.001;
			cam_cloud->points[cnt].r = 255;
			cam_cloud->points[cnt].g = 0;
			cam_cloud->points[cnt].b = 0;
		}
  }

  // Fill in the cloud data
  lidar_cloud->width  = 21 * 7;
  lidar_cloud->height = 1;
  lidar_cloud->points.resize(lidar_cloud->width * lidar_cloud->height);

  // Generate the data
	cnt = 0;
  for (size_t i = 0; i <= 20; ++i) {
		for (size_t j = 0; j <= 6; ++j, cnt++) {
			lidar_cloud->points[cnt].x = i * 0.05  + (rand () % 20) / 100.0;
			lidar_cloud->points[cnt].y = j * 0.05 + (rand () % 20) / 100.0;

			lidar_cloud->points[cnt].z = 0;
			lidar_cloud->points[cnt].label = 1;

			lidar_cloud->points[cnt].r = 0;
			lidar_cloud->points[cnt].g = 255;
			lidar_cloud->points[cnt].b = 0;
		}
  }

  // Set a few outliers
  //lidar_cloud->points[4].z = 3.0;
  //lidar_cloud->points[8].z = -3.0;
  //lidar_cloud->points[12].z = 2.0;

	std::vector<HobotLabels> label_vec;
	label_vec.push_back(lane);

	CmpCamLidarPd<PointCam, PointLidar> cmp(label_vec, 0.005, 20, 20);
	cmp.setCamPd(cam_cloud);
	cmp.setLidarPd(lidar_cloud);

	cmp.cacPoint2Line();
	auto mean_dis = cmp.getMeanDis();
	auto std_dis = cmp.getStdDis();

	LOG(INFO) << "[X mean, std]: " << mean_dis[0] << "m " << std_dis[0] << "m";
	LOG(INFO) << "[Y mean, std]: " << mean_dis[1] << "m " << std_dis[1] << "m";
	LOG(INFO) << "[Z mean, std]: " << mean_dis[2] << "m " << std_dis[2] << "m";

	cmp.cacPoint2Plane();
	mean_dis = cmp.getMeanDis();
	std_dis = cmp.getStdDis();

	LOG(INFO) << "[X mean, std]: " << mean_dis[0] << "m " << std_dis[0] << "m";
	LOG(INFO) << "[Y mean, std]: " << mean_dis[1] << "m " << std_dis[1] << "m";
	LOG(INFO) << "[Z mean, std]: " << mean_dis[2] << "m " << std_dis[2] << "m";

	pcl::io::savePCDFile("cam_pd.pcd", *cam_cloud);
	pcl::io::savePCDFile("lidar_pd.pcd", *lidar_cloud);

	//pcl::visualization::PCLVisualizer viewer("viewer");
	//viewer.addCoordinateSystem(3.0 ,"coor");
	//viewer.initCameraParameters();
	//viewer.setCameraPosition(0.0, 0.0, 10.0, 0.0, 0, 0.0);

	//viewer.addPointCloud(cam_cloud, "cloud_cam");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::
		//PCL_VISUALIZER_POINT_SIZE, 3, "cloud_cam");
	//viewer.addPointCloud(lidar_cloud, "lidar_cam");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::
		//PCL_VISUALIZER_POINT_SIZE, 3, "lidar_cam");
	//viewer.spinOnce(10);

	return 0;
}
