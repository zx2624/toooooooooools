//Author: siyuan.yu

#include <ctime>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>


void select_points(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
									pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected) {
	source_selected->clear();
	for(unsigned int i = 0; i < source->points.size(); i++) {
		pcl::PointXYZI p = source->points[i];
		 //if (fabs(p.x) < 3 && fabs(p.y) < 2 && fabs(p.z) < 2) {//remove the lidar
							//continue;
		 //}
		 source_selected->push_back(p);
	}
}
struct MFrameMsg{
	MFrameMsg()
  : bagID(0)
  , seq(0)
  , timestamp(0.0)
  , speed(0.0)
  , yaw_rate(0.0)
  , pose(Eigen::Matrix4f::Identity()){
}
  uint64_t bagID;
  uint64_t seq;
  double   timestamp;
  double   speed;
  double   yaw_rate;
	Eigen::Matrix4f pose;
	Eigen::Matrix<double, 6, 6> covariance;
};

void readMFrame(std::string& mframe_path, std::vector<MFrameMsg> &mframe) {
	ifstream file;
  file.open(mframe_path.c_str());
	if (file.is_open()) {
		MFrameMsg frame;
		while (!file.eof()) {
      file >> frame.bagID >> frame.seq >> frame.timestamp >> frame.speed >> 
		  frame.pose(0,0) >> frame.pose(0,1) >> frame.pose(0,2) >> frame.pose(0,3) >> 
			frame.pose(1,0) >> frame.pose(1,1) >> frame.pose(1,2) >> frame.pose(1,3) >> 
			frame.pose(2,0) >> frame.pose(2,1) >> frame.pose(2,2) >> frame.pose(2,3) >> 
		  frame.pose(3,0) >> frame.pose(3,1) >> frame.pose(3,2) >> frame.pose(3,3);
      mframe.push_back(frame);
		}
		mframe.pop_back();//file.eof() will double the last line, so delete back once
		cout << "****  OK: Read " << mframe.size() << " mframes in " << mframe_path << "  ****" << endl;
		file.close();
   } else {
		 cout << "****  ERROR: Don't exist mframe: " << mframe_path << "  ****" << endl;
	}
}
//get min man 3D point
template<typename PointT>
void getMinMax3D(typename pcl::PointCloud<PointT>::Ptr cloud, 
                 Eigen::Vector4d& g_min_pt, Eigen::Vector4d& g_max_pt) {
  g_min_pt[0] = g_min_pt[1] = g_min_pt[2] = FLT_MAX;
  g_max_pt[0] = g_max_pt[1] = g_max_pt[2] = -FLT_MAX;
  if (cloud->is_dense) {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      g_min_pt[0] = std::min<double>(g_min_pt[0], cloud->points[i].x);
      g_max_pt[0] = std::max<double>(g_max_pt[0], cloud->points[i].x);
      g_min_pt[1] = std::min<double>(g_min_pt[1], cloud->points[i].y);
      g_max_pt[1] = std::max<double>(g_max_pt[1], cloud->points[i].y);
      g_min_pt[2] = std::min<double>(g_min_pt[2], cloud->points[i].z);
      g_max_pt[2] = std::max<double>(g_max_pt[2], cloud->points[i].z);        
    }
  } else {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      if (!pcl_isfinite(cloud->points[i].x) ||
          !pcl_isfinite(cloud->points[i].y) ||
          !pcl_isfinite(cloud->points[i].z)) {
          continue;
      }
      g_min_pt[0] = std::min<double>(g_min_pt[0], cloud->points[i].x);
      g_max_pt[0] = std::max<double>(g_max_pt[0], cloud->points[i].x);
      g_min_pt[1] = std::min<double>(g_min_pt[1], cloud->points[i].y);
      g_max_pt[1] = std::max<double>(g_max_pt[1], cloud->points[i].y);
      g_min_pt[2] = std::min<double>(g_min_pt[2], cloud->points[i].z);
      g_max_pt[2] = std::max<double>(g_max_pt[2], cloud->points[i].z);
    }      
  }
}
void genMap(pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud,
		double xy_res, double xy_size, const std::string out_dir) {
  
  Eigen::Vector4d g_min_pt, g_max_pt;                                         
  getMinMax3D<pcl::PointXYZI>(raw_cloud, g_min_pt, g_max_pt);
  
  double block_size = xy_res * xy_size;

  double min_x = floor(g_min_pt(0) / block_size) * block_size;
  double min_y = floor(g_min_pt(1) / block_size) * block_size;

  double max_x = ceil(g_max_pt(0) / block_size) * block_size;
  double max_y = ceil(g_max_pt(1) / block_size) * block_size;

	std::cout << min_x << " [x] " << max_x << " "
						<< min_y << " [y] " << max_y << "\n";

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vec_pointcloud;
	for (double x = min_x; x < max_x ; x += block_size) {
		for (double y = min_y; y < max_y; y += block_size) {
			//each block have a pointcloud
			pcl::PointCloud<pcl::PointXYZI>::Ptr 
				cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
			vec_pointcloud.push_back(cloud_ptr);
		}
	}
	std::cout << "Have " << vec_pointcloud.size() << " blocks!\n";

	int64_t count = 0;
	for(uint64_t i = 0; i < raw_cloud->points.size(); i++) {
		pcl::PointXYZI point = raw_cloud->points[i];
      if (!pcl_isfinite(point.x) ||
          !pcl_isfinite(point.y) ||
          !pcl_isfinite(point.z)) {
          continue;
      }
	  bool found = false;
		//vec_pointcloud's index
		int index = 0;
		for (double x = min_x; !found && x < max_x ; x += block_size) {
			for (double y = min_y; y < max_y; y += block_size) {
				if(point.x >= x && point.x < x + block_size
					&& point.y >= y && point.y < y + block_size) {
					found = true;
					vec_pointcloud[index]->push_back(point);
					break;
				}
				index++;
			}
		}
		if(!found) count++;
	}
	std::cerr << count << " points can't found block\n\n";
	int index = 0;
	pcl::VoxelGrid<pcl::PointXYZI> voxel; 
	double leaf_res = 0.04;
	voxel.setLeafSize(leaf_res, leaf_res, leaf_res);
		for (double x = min_x; x < max_x ; x += block_size) {
			for (double y = min_y; y < max_y; y += block_size) {
			if(vec_pointcloud[index]->points.size() <= 0){
				index++;
				continue;
			}
      std::ostringstream oss;
      //oss << std::setiosflags(std::ios::fixed) << std::setprecision(2)
          //<< xy_res_ << "_" << size_ << "_" << z_res_ << "_250_"
          //<< z_res_ * 2 << "_250" << "_map_" 
          //<< x << "_" << y << "_" << min_z << ".jpg";
			 oss << std::setiosflags(std::ios::fixed) << std::setprecision(2) 
					 << leaf_res << "_map_" << x << "_" << y << "_" << x + block_size
					 << "_" << y + block_size << ".pcd";
			 std::cout << "oss: " << oss.str() << "\n\n";
      
       std::string pcd_path = out_dir + oss.str();
       if (0 != access(pcd_path.c_str(), F_OK)) {
				 pcl::PointCloud<pcl::PointXYZI>::Ptr
					 filtered(new pcl::PointCloud<pcl::PointXYZI>); 
				 voxel.setInputCloud(vec_pointcloud[index]);
				 std::cout << "Before filtered: " << vec_pointcloud[index]->points.size() 
									<< std::endl;
				 voxel.filter(*filtered);
				 std::cout << "After filtered: " << filtered->points.size() << std::endl;
				 pcl::io::savePCDFileASCII(pcd_path, *filtered);
				 //pcl::io::savePCDFileASCII(pcd_path, *(vec_pointcloud[index]));
				 std::cout << "save " << filtered->points.size() 
									 << " point to " << pcd_path << std::endl << std::endl;    
       } 
			 else {
				 //load exist pcd file
				 pcl::PointCloud<pcl::PointXYZI>::Ptr 
					 exist_pcd(new pcl::PointCloud<pcl::PointXYZI>);
				 pcl::PointCloud<pcl::PointXYZI>::Ptr 
					 filtered(new pcl::PointCloud<pcl::PointXYZI>); 
				 pcl::io::loadPCDFile(pcd_path, *exist_pcd);
				 //merge new pointcloud
				 *exist_pcd += *vec_pointcloud[index];
				 voxel.setInputCloud(exist_pcd);
				 std::cout << "Before filtered: " << exist_pcd->points.size()
									<< std::endl;
				 voxel.filter(*filtered);
				 std::cout << "After filtered: " << filtered->points.size() << std::endl;
				 std::cout << "add " << filtered->points.size() 
									 << " point to " << pcd_path << std::endl << std::endl;    
				 pcl::io::savePCDFileASCII(pcd_path, *filtered);
			 }
			 index++;
			}
    }
}
void MergePointcloud(std::string bag_file_path_ , std::string odom_path_ ,
										std::string out_dir) {
	rosbag::Bag bag_;
	bag_.open(bag_file_path_, rosbag::bagmode::Read);
	std::cout << "open bag file success! \n";

	std::fstream ofs(odom_path_.c_str());
	if(!ofs.is_open()) {
		std::cerr << odom_path_ << "not exist!\n";
		exit(-1);
	}

	std::vector<MFrameMsg> odom_frame;
	readMFrame(odom_path_, odom_frame);
	std::cout << "odom size: " << odom_frame.size() << std::endl;

	std::vector<rosbag::View::iterator> ptis;
#ifdef Pandar
	rosbag::View points_view(bag_, rosbag::TopicQuery("/sensor/pandar/points"));
#else
	rosbag::View points_view(bag_, rosbag::TopicQuery("/sensor/velodyne/points"));
#endif
	for (rosbag::View::iterator pti = points_view.begin(); pti != points_view.end(); pti++) {
			//cout << "push back points iterator! \n"; 
	    ptis.push_back(pti);
	}
	//std::cout << "pointcloud size: " << ptis.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr source_transformed(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr sum_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr local_sum(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr local_sum_filtered(new pcl::PointCloud<pcl::PointXYZI>);

	//define voxel grid
	//pcl::VoxelGrid<pcl::PointXYZI> voxel; 
	//voxel.setLeafSize(0.1, 0.1, 0.1);

	sensor_msgs::PointCloud2::Ptr ros_pcd(new sensor_msgs::PointCloud2);
	int batch = 0;
	int round = 0;
	for(uint64_t i = 0; i < odom_frame.size(); i++) {
	//for(uint64_t i = 0; i < odom_frame.size() && i < 100; i++) {
		ros_pcd = ptis[odom_frame[i].seq]->instantiate<sensor_msgs::PointCloud2>();
		pcl::fromROSMsg(*ros_pcd, *source);
		select_points(source, source_selected);
		Eigen::Matrix4f transform_to_init = odom_frame[i].pose;
		pcl::transformPointCloud(*source_selected, *source_transformed, transform_to_init);
		//std::cout << "pointcloud size: " << source_transformed->points.size() << std::endl;
		*local_sum += *source_transformed;
		//when accumulate 500 frames, then call genMap function
		if(batch++ == 500) { 
			std::cout << "\n---------Start genMap round " << ++round << std::endl;
#ifdef DEBUG
			pcl::io::savePCDFileASCII(std::to_string(ros_pcd->header.stamp.toSec()) + ".pcd", *local_sum);
			std::cout << "Save pointcloud " << std::to_string(ros_pcd->header.stamp.toSec()) + ".pcd" << std::endl;
#else 
			genMap(local_sum, 0.1, 400, out_dir);
#endif
			std::cout << "\n---------End genMap:\n";
			local_sum->clear();
			batch = 0;
		}
		
		//*local_sum += *source_transformed;
		//if(local_sum->points.size() > 50000) {
			//voxel.setInputCloud(local_sum);
			//voxel.filter(*local_sum_filtered);
			//voxel.setInputCloud(source_transformed);
			//voxel.filter(*local_sum_filtered);
			//std::cout << "Before filtered size : " << source_transformed->points.size() 
								//<< " After filtered size :  " << local_sum_filtered->points.size()
								//<< "\n";
			//merge
			//*sum_filtered += *local_sum_filtered;
			//clear local_sum 
			//local_sum->clear();
		//}
	}
#ifndef DEBUG
	if(local_sum->points.size() > 0) {
			std::cout << "\n---------Start genMap:\n";
			genMap(local_sum, 0.1, 400, out_dir);
			std::cout << "\n---------End genMap:\n";
	}
#endif

	//pcl::visualization::PCLVisualizer viewer("viewer");
	//viewer.addCoordinateSystem(3.0 ,"coor");
	//viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
	//viewer.initCameraParameters();
	//viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//for(size_t i = 0; i < sum->points.size(); i++) {
		//pcl::PointXYZI p = sum->points[i];
		//pcl::PointXYZRGB p1;
		//p1.x = p.x;
		//p1.y = p.y;
		//p1.z = p.z;
		//p1.r = 0;
		//p1.g = p.intensity * 8 > 255 ? 255 : p.intensity * 8;
		//p1.b = 0 ;
		//show_cloud->push_back(p1);
	//}
	//viewer.addPointCloud(show_cloud, "cloud");
	//while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		//viewer.spinOnce ();
	//}
	bag_.close();
	ofs.close();
}
void voxelFilterEachPcdFile() {
}
int main(int argc, char** argv) {
	if(argc < 4) {
		std::cerr << "argv: bag_file_path_ odom_path out_path\n";
		return -1;
	}
	std::cout << "bag_file_path: " << argv[1] << std::endl
						<< "odom_path: "<< argv[2] << std::endl
						<< "out_path: "<< argv[3] << std::endl;
	if (0 != access(argv[1], F_OK)){
		std::cerr << argv[1] << "not exist!\n";
	}
	if (0 != access(argv[2], F_OK)){
		std::cerr << argv[2] << "not exist!\n";
	}
	if (0 != access(argv[3], F_OK)){
		std::cerr << argv[3] << "not exist!\n";
	}
	MergePointcloud(std::string(argv[1]), std::string(argv[2]), 
									std::string(argv[3]));
	return 0;
}
