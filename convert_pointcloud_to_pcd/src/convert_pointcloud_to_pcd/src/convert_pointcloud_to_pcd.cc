// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     src/convert_pointcloud_to_pcd.cc
// ROLE:     TODO (some explanation)
// CREATED:  2018-04-09 17:08:45
// MODIFIED: 2018-04-18 15:10:07
#include <iostream>
#include <string>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void select_points(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected) {
	source_selected->clear();
	int count = 0;
  for(unsigned int i = 0; i < source->points.size(); i++) {
     pcl::PointXYZI p = source->points[i];
		 //if(p.z < -1.6 || p.z > 100 
				//|| (fabs(p.x) < 3 && fabs(p.y) < 2 && fabs(p.z) < 2)) {
								 //continue;
			//}
		 if(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
			 count++;
			 continue;
		 }
		 //if(p.z < -2.65  || p.z > -2.2) continue;
		 source_selected->push_back(p);
	}
	std::cout << "nan point : " << count << std::endl;
}

int main(int argc, char** argv) {
	rosbag::Bag bag;

	bag.open(argv[1], rosbag::bagmode::Read);

	//rosbag::View points_view(bag, rosbag::TopicQuery("/sensor/pandar/points"));
	rosbag::View points_view(bag, rosbag::TopicQuery("/rslidar_points"));

	sensor_msgs::PointCloud2::Ptr ros_pcd(new sensor_msgs::PointCloud2);

	pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);

	for (rosbag::View::iterator pti = points_view.begin(); pti != points_view.end(); pti++) {
		ros_pcd = pti->instantiate<sensor_msgs::PointCloud2>();
		pcl::fromROSMsg(*ros_pcd, *source);
		select_points(source, source_selected);
		pcl::io::savePCDFileASCII(std::to_string(ros_pcd->header.stamp.toSec()) + ".pcd", *source_selected);
		std::cout << "Save " << source_selected->points.size() << " / " << source->points.size() << " points " 
			<< std::to_string(ros_pcd->header.stamp.toSec()) + ".pcd" << std::endl;
	}
	bag.close();
	return 0;
}

