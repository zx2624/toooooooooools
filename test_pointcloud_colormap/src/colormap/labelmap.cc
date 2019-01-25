//Author: siyuan.yu


#include <ctime>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>


#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifdef Pandora
typedef pcl::PointXYZRGBL PointType;
#else
typedef pcl::PointXYZI PointType;
#endif
typedef pcl::Label PointLable;
typedef pcl::PointXYZRGB PointXYZRGB;

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
        
void select_points(pcl::PointCloud<PointType>::Ptr source,
									pcl::PointCloud<PointType>::Ptr source_selected) {
	source_selected->clear();
	for(unsigned int i = 0; i < source->points.size(); i++) {
		auto p = source->points[i];
		 if ((fabs(p.x) < 2 && 
					 fabs(p.y) < 2 &&
						fabs(p.z) < 2)) {//remove the lidar
							continue;
		 }
		 if (p.z < -5 || p.z > 30.0) {//remove ground
							continue;
		 }
		 source_selected->push_back(p);
	}
}

void labelmap(char* bag_file_path_, char* points_topic_name_, char* points_label_topoc_name_) {

	rosbag::Bag bag_;
	bag_.open(bag_file_path_, rosbag::bagmode::Read);
	std::cout << "open bag file success! \n";

	rosbag::View points_view(bag_, rosbag::TopicQuery(points_topic_name_));
	rosbag::View points_label_view(bag_, rosbag::TopicQuery(points_label_topoc_name_));

	pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointLable>::Ptr source_label(new pcl::PointCloud<PointLable>);
	pcl::PointCloud<PointType>::Ptr source_selected(new pcl::PointCloud<PointType>);

	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.addCoordinateSystem(3.0 ,"coor");
	viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
	viewer.initCameraParameters();
	viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, -1.0, 0.0);

	pcl::PointCloud<PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<PointXYZRGB>);

	rosbag::View::iterator its = points_view.begin();
	rosbag::View::iterator its_label = points_label_view.begin();

	const double EPS = 0.01;

	//align pointcloud and label pointcloud
	while(its != points_view.end() && its_label != points_label_view.end()) {
		sensor_msgs::PointCloud2ConstPtr pt1
			= its->instantiate<sensor_msgs::PointCloud2>();
		sensor_msgs::PointCloud2ConstPtr pt2
			= its_label->instantiate<sensor_msgs::PointCloud2>();

		double pt1_stamp = pt1->header.stamp.toSec();
		double pt2_stamp = pt2->header.stamp.toSec();

		if (pt1_stamp - pt2_stamp > EPS) {
			its_label++;
		}
		else if(pt2_stamp - pt1_stamp > EPS) {
			its++;
		}
		else break;
	}
		
	while(its != points_view.end() && its_label != points_label_view.end()) {
		//get a data instantiate
		auto raw_pcd = its->instantiate<sensor_msgs::PointCloud2>();
		auto label_pcd = its_label->instantiate<sensor_msgs::PointCloud2>();

		pcl::fromROSMsg(*raw_pcd, *source);
		pcl::fromROSMsg(*label_pcd, *source_label);

		//select_points(source, source_selected);

		show_cloud->resize(source->points.size());
		unsigned int cnt = 0;
		for(size_t i = 0; i < source->points.size(); i++) {
			auto p = source->points[i];

			auto label = source_label->points[i];

			pcl::PointXYZRGB p1;
			p1.x = p.x;
			p1.y = p.y;
			p1.z = p.z;

			auto idx = static_cast<unsigned char>(label.label);
			
			if (idx == INVALID_LABEL) continue;
			p1.r = colorlist[idx][0];
			p1.g = colorlist[idx][1];
			p1.b = colorlist[idx][2];

			show_cloud->points[cnt++] = p1;
		}
		show_cloud->resize(cnt);

		std::cout << "Get " << show_cloud->points.size() << " Points.\n";
		viewer.removePointCloud("cloud");
		viewer.addPointCloud(show_cloud, "cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			3, "cloud");
		viewer.spinOnce(100);

		its++;
		its_label++;
	}

	bag_.close();
}


int main(int argc, char** argv) {
	if(argc < 3) {
		exit(-1);
	}
	std::cout << "bag_file_path: " << argv[1] << std::endl;
	std::cout << "points_topic_name: " << argv[2] << std::endl;
	std::cout << "points_label_topic_name: " << argv[3] << std::endl;
	labelmap(argv[1], argv[2], argv[3]);

	return 0;
}
