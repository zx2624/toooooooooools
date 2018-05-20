// Copyright 2017 Horizon-Robotics Inc. All Rights Reserved.
//

#include <glog/logging.h>
#include <iostream>
#include <cstdio>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <dirent.h>                                                             
#include <sys/stat.h>                                                           
#include <sys/types.h>                                                          
#include <stdint.h>                                                             
#include <fcntl.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp> 
#include <fstream>
#include <sstream>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/boost.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <boost/foreach.hpp>
#include <opencv2/core/core.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/simple_filter.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <autodrive_msgs/VehicleStatus.h>

#include "lidar_mapping/define.hpp"

const int MAX_VALID = 250;

namespace horizon {
namespace mapping {

class LidarMapper {
  
public:                                                                         
  LidarMapper(const std::string &file_path);                                                                
  ~LidarMapper();
  
  bool readConfig(const std::string &file_path);                                                           
  void readFileList(std::string &in_dir, std::vector<std::string> &file_list);
  void selectPoints(PCloudVPtr cloud_raw, PCloudVPtr cloud_sel);
  double calRotTime(PointV point);
  void motionCompensation(double speed, double yaw_rate, PCloudVPtr cloud_in);

  void mapper();
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_ground, 
                const sensor_msgs::PointCloud2ConstPtr& cloud_altitude,
                const nav_msgs::OdometryConstPtr& pose_odom,
                const autodrive_msgs::VehicleStatusConstPtr& chassis);
  void registerCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_ground, 
                const sensor_msgs::PointCloud2ConstPtr& cloud_altitude,
                const nav_msgs::OdometryConstPtr& pose_odom,
                const autodrive_msgs::VehicleStatusConstPtr& chassis);
  void genMap();
  void mergeMap();

  void fillMean(cv::Mat &mat);                            
  void fillZero(cv::Mat &mat);                                                 
  void postProcess();

private:                                                                           
  PCloudVPtr ground_cloud_;
  PCloudVPtr altitude_cloud_;  

  std::string bag_dir_;
  std::string ground_topic_;
  std::string altitude_topic_;
  std::string pose_topic_;
  std::string chassis_topic_;
  
  float xy_res_;
  float z_res_;
  int size_;
  double x_base_;
  double y_base_;

  std::string out_dir_;
  std::string out_int_dir_;
  std::string out_ele_dir_;
  std::string out_alt_dir_;
  std::string out_config_path_;                                                  
};

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
public:
  void newMessage(const boost::shared_ptr<M const> &msg) {
    this->signalMessage(msg);
  }
};

LidarMapper::LidarMapper(const std::string &file_path) {
  bag_dir_ = "bag/";
  ground_topic_ = "/perception/ground_points";
  altitude_topic_ = "/perception/altitude_points";
  pose_topic_ = "/posegraph/odom";
  chassis_topic_ = "/vehicle/status";
  
  xy_res_ = 0.04;
  z_res_ = 0.04;
  size_ = 1000;
  x_base_ = 328000;
  y_base_ = 3462000;
  
  if (!readConfig(file_path)) {
    LOG(FATAL) << "read config error!";
  }

  out_dir_ = "map/";
  out_int_dir_ = out_dir_ + "intensity/";
  out_ele_dir_ = out_dir_ + "elevation/";
  out_alt_dir_ = out_dir_ + "altitude/";
  out_config_path_ = out_dir_ + "config.yaml";

  if (access(out_dir_.c_str(), 0) != 0) {                                            
    if(boost::filesystem::create_directories(out_dir_)) {
      LOG(FATAL) << "mkdir error!";                   
      return;                                                   
    }
  }
  if (access(out_int_dir_.c_str(), 0) != 0) {                                            
    if(boost::filesystem::create_directories(out_int_dir_)) {
      LOG(FATAL) << "mkdir error!";                   
      return;                                                   
    }
  }
  if (access(out_ele_dir_.c_str(), 0) != 0) {                                            
    if(boost::filesystem::create_directories(out_ele_dir_)) {
      LOG(FATAL) << "mkdir error!";                   
      return;                                                   
    }
  }
  if (access(out_alt_dir_.c_str(), 0) != 0) {                                            
    if(boost::filesystem::create_directories(out_alt_dir_)) {
      LOG(FATAL) << "mkdir error!";                   
      return;                                                   
    }
  }

  YAML::Node config;
  config["image_size"] = size_;
  config["xy_resolution"] = xy_res_;
  config["z_resolution"] = z_res_;
  config["x_base"] = x_base_;
  config["y_base"] = y_base_;
  std::ofstream fout(out_config_path_);
  fout << config;
  fout.close();
  
  ground_cloud_ = PCloudVPtr(new PCloudV());
  altitude_cloud_ = PCloudVPtr(new PCloudV());

  mapper();
  postProcess();
}

LidarMapper::~LidarMapper() {}

bool LidarMapper::readConfig(const std::string &file_path) {
  YAML::Node config = YAML::LoadFile(file_path);                                
  if (config["bag_dir"]) {
    bag_dir_ = config["bag_dir"].as<std::string>();
  } else {
    return false;
  }

  if (config["ground_topic"]) {
    ground_topic_ = config["ground_topic"].as<std::string>();
  }
  if (config["altitude_topic"]) {
    altitude_topic_ = config["altitude_topic"].as<std::string>();
  }
  if (config["pose_topic"]) {
    pose_topic_ = config["pose_topic"].as<std::string>();
  }
  if (config["chassis_topic"]) {
    chassis_topic_ = config["chassis_topic"].as<std::string>();
  }

  if (config["xy_res"]) {
    xy_res_ = config["xy_res"].as<float>();
  }
  if (config["z_res"]) {
    z_res_ = config["z_res"].as<float>();
  }
  if (config["size"]) {
    size_ = config["size"].as<int>();
  }
  if (config["x_base"]) {
    x_base_ = config["x_base"].as<double>();
  } 
  if (config["y_base"]) {
    y_base_ = config["y_base"].as<double>();
  } 

  return true;                                                                 
}

void LidarMapper::readFileList(std::string &in_dir, std::vector<std::string> &file_list) {
  DIR *dir = NULL;                                                               
  if ((dir = opendir(in_dir.c_str())) == NULL) {                                 
    LOG(FATAL) << in_dir << ": Open dir failed.";                              
  }                                                                              
  struct dirent *ptr;                                                            
  while (ptr = readdir(dir)) {                                                   
    if (ptr->d_name[0] == '.' || ptr->d_type == DT_DIR) {                      
      continue;                                                              
    }                                                                          
    std::string bag_name(ptr->d_name);
    if (bag_name.size() < 4 || bag_name.substr(bag_name.size() - 3) != "bag") {
      continue;
    }                                       
    file_list.push_back(bag_name);                                           
  }                                                                              
  sort(file_list.begin(), file_list.end());                                      
}

void LidarMapper::selectPoints(PCloudVPtr cloud_raw, PCloudVPtr cloud_sel) {                          
  *cloud_sel = *cloud_raw; 
  for (int i = 0; i < cloud_raw->points.size(); ++i) {
    PointV &point = cloud_raw->points[i];
    double r = sqrt(point.x * point.x + point.y * point.y);                       
    if (r < 5.0) continue;                                                    
    if (r > 20.0) continue;                                                   
    if (point.z < -2.0 || point.z > 8.2) continue; 
    cloud_sel->points.push_back(point);
  }
}

double LidarMapper::calRotTime(PointV point) {
  double rotation_period = 0.1;
  
  double angle = -atan2(point.y, point.x);
  if (point.y > 0) {                                           
    angle = -angle - M_PI / 2;                                                    
  } else {                                                                    
    angle = -angle + M_PI * 3 / 2;                                                
  }
  
  if (angle > 2 * M_PI || angle < 0) {
    angle = 2 * M_PI;
  }
  return (2 * M_PI - angle) / (2 * M_PI) * rotation_period;
}

void LidarMapper::motionCompensation(double speed, double yaw_rate, PCloudVPtr cloud_in) {
  double vx = speed;
  double vy = 0;
  for(size_t i = 0; i < cloud_in->points.size(); i++) {
    PointV &point = cloud_in->points[i];
    //linear velocity compensation
    double time_deta = calRotTime(point);
    point.x += vx * time_deta;
    point.y += vy * time_deta;
    //angular velocity compensation
    double dyaw = yaw_rate * time_deta;
    point.x = cos(dyaw) * point.x - sin(dyaw) * point.y;
    point.y = sin(dyaw) * point.x + cos(dyaw) * point.y;
  }
}

std::vector<sensor_msgs::PointCloud2ConstPtr> ground_v;
std::vector<sensor_msgs::PointCloud2ConstPtr> altitude_v;
std::vector<nav_msgs::OdometryConstPtr> odom_v;
std::vector<autodrive_msgs::VehicleStatusConstPtr> chassis_v;

void LidarMapper::mapper() {
  std::vector<std::string> bag_list;
  readFileList(bag_dir_, bag_list);

  for (size_t i = 0; i < bag_list.size(); i++) {
    std::string bag_path = bag_dir_ + "/" + bag_list[i];
    if (access(bag_path.c_str(), 0) != 0) {
      LOG(FATAL) << "File don't exist " << bag_path;
    }
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(ground_topic_);
    topics.push_back(altitude_topic_);
    topics.push_back(pose_topic_);
    topics.push_back(chassis_topic_);
    
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    BagSubscriber<sensor_msgs::PointCloud2> ground_sub;
    BagSubscriber<sensor_msgs::PointCloud2> altitude_sub;
    BagSubscriber<nav_msgs::Odometry> pose_sub;
    BagSubscriber<autodrive_msgs::VehicleStatus> chassis_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
        sensor_msgs::PointCloud2, nav_msgs::Odometry, autodrive_msgs::VehicleStatus> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(100), ground_sub, 
        altitude_sub, pose_sub, chassis_sub);
    sync.registerCallback(boost::bind(&LidarMapper::callback, this, _1, _2, _3, _4));
    
    int batch = 0;
    for (rosbag::View::iterator it = view.begin(); it != view.end() && ros::ok(); ++it) {
      if (it->getTopic() == ground_topic_) {
        sensor_msgs::PointCloud2ConstPtr ground = it->instantiate<sensor_msgs::PointCloud2>();
        // ground = it->instantiate<sensor_msgs::PointCloud2>();
        if (ground != NULL)
          ground_sub.newMessage(ground);
      }
      if (it->getTopic() == altitude_topic_) {
        sensor_msgs::PointCloud2ConstPtr altitude = it->instantiate<sensor_msgs::PointCloud2>();
        // altitude = it->instantiate<sensor_msgs::PointCloud2>();
        if (altitude != NULL)
          altitude_sub.newMessage(altitude);
      }
      if (it->getTopic() == pose_topic_) {
        nav_msgs::OdometryConstPtr pose = it->instantiate<nav_msgs::Odometry>();
        // pose = it->instantiate<nav_msgs::Odometry>();
        if (pose != NULL) {
          pose_sub.newMessage(pose);
          ++ batch;
        }
      }
      if (it->getTopic() == chassis_topic_) {
        autodrive_msgs::VehicleStatusConstPtr chassis = it->instantiate<autodrive_msgs::VehicleStatus>();
        // chassis = it->instantiate<autodrive_msgs::VehicleStatus>();
        if (chassis != NULL)
          chassis_sub.newMessage(chassis);
      }
      ros::spinOnce();

      if (batch == 100) {
        for (int k = 0; k < odom_v.size(); ++k) {
          registerCloud(ground_v[k], altitude_v[k], odom_v[k], chassis_v[k]);
        }
        ground_v.clear();
        altitude_v.clear();
        odom_v.clear();
        chassis_v.clear();

        genMap();
        batch = 0;
      }
    }

    for (int k = 0; k < odom_v.size(); ++k) {
      registerCloud(ground_v[k], altitude_v[k], odom_v[k], chassis_v[k]);
    }
    ground_v.clear();
    altitude_v.clear();
    odom_v.clear();
    chassis_v.clear();

    genMap();
    bag.close();
  }
}

void LidarMapper::callback(const sensor_msgs::PointCloud2ConstPtr& ground_msg, 
                           const sensor_msgs::PointCloud2ConstPtr& altitude_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg,
                           const autodrive_msgs::VehicleStatusConstPtr& chassis_msg) {
  ground_v.push_back(ground_msg);
  altitude_v.push_back(altitude_msg);
  odom_v.push_back(odom_msg);
  chassis_v.push_back(chassis_msg);
}

void LidarMapper::registerCloud(const sensor_msgs::PointCloud2ConstPtr& ground_msg, 
                           const sensor_msgs::PointCloud2ConstPtr& altitude_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg,
                           const autodrive_msgs::VehicleStatusConstPtr& chassis_msg) {
  Eigen::Quaterniond q = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, 
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, 
      odom_msg->pose.pose.orientation.z);
  Eigen::Vector3d t = Eigen::Vector3d(odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
  Eigen::Matrix4d pose;
  pose.setOnes();
  pose.topLeftCorner(3, 3) = q.toRotationMatrix();
  pose.topRightCorner(3, 1) = t;

  PCloudVPtr ground_raw = PCloudVPtr(new PCloudV);
  pcl::fromROSMsg(*ground_msg, *ground_raw);
  
  PCloudVPtr ground_sel = PCloudVPtr(new PCloudV); 
  selectPoints(ground_raw, ground_sel);
  // motionCompensation(chassis_msg->speed, chassis_msg->yaw_rate, ground_sel);

  PCloudVPtr ground_trans = PCloudVPtr(new PCloudV);
  pcl::transformPointCloud(*ground_sel, *ground_trans, pose);
  
  *ground_cloud_ += *ground_trans;


  PCloudVPtr altitude_raw  = PCloudVPtr(new PCloudV);
  pcl::fromROSMsg(*altitude_msg, *altitude_raw);
  
  PCloudVPtr altitude_sel  = PCloudVPtr(new PCloudV); 
  selectPoints(altitude_raw, altitude_sel);
  // motionCompensation(chassis_msg->speed, chassis_msg->yaw_rate, altitude_sel);

  PCloudVPtr altitude_trans = PCloudVPtr(new PCloudV);
  pcl::transformPointCloud(*altitude_sel, *altitude_trans, pose);
  
  *altitude_cloud_ += *altitude_trans;
}

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

void LidarMapper::genMap(void) {
  PCloudVPtr ground_cloud = ground_cloud_;
  
  Eigen::Vector4d g_min_pt, g_max_pt;                                         
  getMinMax3D<PointV>(ground_cloud, g_min_pt, g_max_pt);
  
  double block_size = xy_res_ * size_;
  double min_x = floor(g_min_pt(0) / block_size) * block_size;
  double min_y = floor(g_min_pt(1) / block_size) * block_size;
  double max_x = ceil(g_max_pt(0) / block_size) * block_size;
  double max_y = ceil(g_max_pt(1) / block_size) * block_size;
  LOG(INFO) << min_x << " [x] " << max_x << "   " << min_y << " [y] " << max_y;
  
  PCloudVPtr ground_filtered = PCloudVPtr(new PCloudV);
  pcl::VoxelGrid<PointV> g_voxel_grid;
  g_voxel_grid.setLeafSize(xy_res_, xy_res_, z_res_);
  g_voxel_grid.setDownsampleAllData(true);
  g_voxel_grid.setSaveLeafLayout(true);
  g_voxel_grid.setMinimumPointsNumberPerVoxel(4);
  g_voxel_grid.setInputCloud(ground_cloud);
  g_voxel_grid.filter(*ground_filtered);
  Eigen::Vector3i g_min_b = g_voxel_grid.getMinBoxCoordinates();
  Eigen::Vector3i g_max_b = g_voxel_grid.getMaxBoxCoordinates();
  LOG(INFO) << ground_cloud->points.size() << " -[ground]-> " << ground_filtered->points.size() 
      << "  " << g_min_b.transpose() << " " << g_max_b.transpose();

  PCloudVPtr altitude_cloud = altitude_cloud_;

  Eigen::Vector4d a_min_pt, a_max_pt;                                         
  getMinMax3D<PointV>(altitude_cloud, a_min_pt, a_max_pt);

  PCloudVPtr altitude_filtered = PCloudVPtr(new PCloudV);
  pcl::VoxelGrid<PointV> a_voxel_grid;
  a_voxel_grid.setLeafSize(xy_res_, xy_res_, z_res_);
  a_voxel_grid.setDownsampleAllData(true);
  a_voxel_grid.setSaveLeafLayout(true);
  a_voxel_grid.setMinimumPointsNumberPerVoxel(2);
  a_voxel_grid.setInputCloud(altitude_cloud);
  a_voxel_grid.filter(*altitude_filtered);
  Eigen::Vector3i a_min_b = a_voxel_grid.getMinBoxCoordinates();
  Eigen::Vector3i a_max_b = a_voxel_grid.getMaxBoxCoordinates();
  LOG(INFO) << altitude_cloud->points.size() << " -[altitude]-> " << altitude_filtered->points.size()
      << "  " << a_min_b.transpose() << " " << a_max_b.transpose();
  
  for (double x = min_x; x < max_x; x += block_size) {
    for (double y = min_y; y < max_y; y += block_size) {
      if (x > g_max_pt(0) || (x + block_size) < g_min_pt(0) || 
          y > g_max_pt(1) || (y + block_size) < g_min_pt(1)) {
        continue;
      }

      std::vector<Eigen::MatrixXd> mat(4);                                          
      for (int i = 0; i < mat.size(); ++i) {                                        
        mat[i].resize(size_, size_);                                                  
        mat[i].setZero();
      }
      mat[3].setConstant(-std::numeric_limits<double>::max());

      for (int ix = 0; ix < size_; ++ix) {
        for (int iy = 0; iy < size_; ++iy) {
          double xx = x + ix * xy_res_;
          double yy = y + iy * xy_res_;

          double zz = (g_min_pt(2) + g_max_pt(2)) / 2;
          if (xx < g_min_pt(0) || xx > g_max_pt(0) || yy < g_min_pt(1) || yy > g_max_pt(1)) {
            continue;
          }
          Eigen::Vector3i idx = g_voxel_grid.getGridCoordinates(xx, yy, zz);
          for (idx(2) = g_min_b(2); idx(2) < g_max_b(2); ++idx(2)) {
            int pid = g_voxel_grid.getCentroidIndexAt(idx);
            if (pid >= 0) {
              mat[0](iy, ix) += 1;
              mat[1](iy, ix) += ground_filtered->points[pid].intensity;
              mat[2](iy, ix) += ground_filtered->points[pid].z;
            }
          }

          zz = (a_min_pt(2) + a_max_pt(2)) / 2;
          if (xx < a_min_pt(0) || xx > a_max_pt(0) || yy < a_min_pt(1) || yy > a_max_pt(1)) {
            continue;
          }
          idx = a_voxel_grid.getGridCoordinates(xx, yy, zz);
          for (idx(2) = a_max_b(2) - 1; idx(2) >= a_min_b(2); --idx(2)) {
            int pid = a_voxel_grid.getCentroidIndexAt(idx);
            if (pid >= 0) {
              mat[3](iy, ix) = mat[3](iy, ix) < altitude_filtered->points[pid].z ?
                  altitude_filtered->points[pid].z : mat[3](iy, ix);
              break;
            }
          }

        }
      }

      int valid_num = 0;
      double min_ele = std::numeric_limits<double>::max();
      for (int ix = 0; ix < size_; ++ix) {
        for (int iy = 0; iy < size_; ++iy) {
          if (mat[0](iy, ix) > 0) {
            ++ valid_num;
            mat[1](iy, ix) /= mat[0](iy, ix);
            mat[2](iy, ix) /= mat[0](iy, ix);
            if (mat[2](iy, ix) < min_ele) min_ele = mat[2](iy, ix);
          }
        }
      }
      LOG(INFO) << "valid_num: " << valid_num;
      if (valid_num < 10) {
        continue;
      }
      // double min_z = floor(min_ele);
      double min_z = -6;
      
      std::ostringstream oss;
      oss << std::setiosflags(std::ios::fixed) << std::setprecision(2)
          << xy_res_ << "_" << size_ << "_" << z_res_ << "_250_"
          << z_res_ * 2 << "_250" << "_map_" 
          << x << "_" << y << "_" << min_z << ".jpg";
      // oss << std::setiosflags(std::ios::fixed) << std::setprecision(2) 
      //     << xy_res_ << "_map_" << x << "_" << y << "_" << x + block_size << "_" << y + block_size << ".png";
      LOG(INFO) << "oss: " << oss.str();
      
      std::string int_path = out_int_dir_ + oss.str();
      std::string ele_path = out_ele_dir_ + oss.str();
      std::string alt_path = out_alt_dir_ + oss.str();
      if (0 != access(int_path.c_str(), F_OK) || 0 != access(ele_path.c_str(), F_OK) 
          || 0 != access(alt_path.c_str(), F_OK)) {
        cv::Mat img_int, img_ele, img_alt;
        img_int.create(size_, size_, CV_8UC1);
        img_ele.create(size_, size_, CV_8UC1);
        img_alt.create(size_, size_, CV_8UC1);
        for (int ix = 0; ix < size_; ++ix) {
          for (int iy = 0; iy < size_; ++iy) {
            img_int.at<uchar>(iy, ix) = 255;
            img_ele.at<uchar>(iy, ix) = 255;
            img_alt.at<uchar>(iy, ix) = 255;
            // img_int.at<uchar>(ix, iy) = 0;
            // img_ele.at<uchar>(ix, iy) = 0;
            // img_alt.at<uchar>(ix, iy) = 0;
            if (mat[0](iy, ix) > 0) {
              img_int.at<uchar>(iy, ix) = static_cast<uchar>(mat[1](iy, ix) * 10);
              img_ele.at<uchar>(iy, ix) = static_cast<uchar>(mat[2](iy, ix) - min_z);
            } 
            if (mat[3](iy, ix) > -0.5 * std::numeric_limits<double>::max()) {
              img_alt.at<uchar>(iy, ix) = static_cast<uchar>(mat[3](iy, ix) - min_z);
            }
          }
        }
        cv::imwrite(int_path, img_int); 
        cv::imwrite(ele_path, img_ele); 
        cv::imwrite(alt_path, img_alt);
        LOG(INFO) << "save intensity image " << int_path;
        LOG(INFO) << "save elevation image " << ele_path;  
        LOG(INFO) << "save altitude image " << alt_path << std::endl;    
      } else {
        cv::Mat img_int = cv::imread(int_path, -1);
        cv::Mat img_ele = cv::imread(ele_path, -1);
        cv::Mat img_alt = cv::imread(alt_path, -1);
        float decay = 0.9;
        float weight = 1 - decay;
        for (int ix = 0; ix < size_; ++ix) {
          for (int iy = 0; iy < size_; ++iy) {
            if (mat[0](iy, ix) > 0 && img_int.at<uchar>(iy, ix) <= MAX_VALID) {
              img_int.at<uchar>(iy, ix) = static_cast<uchar>(
                  weight * mat[1](iy, ix) + decay * img_int.at<uchar>(iy, ix));
            } else if (mat[0](iy, ix) > 0) {
              img_int.at<uchar>(iy, ix) = static_cast<uchar>(mat[1](iy, ix));
            }

            if (mat[0](iy, ix) > 0 && img_ele.at<uchar>(iy, ix) <= MAX_VALID) {
              img_ele.at<uchar>(iy, ix) = static_cast<uchar>(
                  weight * mat[2](iy, ix) + decay * img_ele.at<uchar>(iy, ix));
            } else if (mat[0](iy, ix) > 0) {
              img_ele.at<uchar>(iy, ix) = static_cast<uchar>(mat[2](iy, ix));
            }

            if (mat[3](iy, ix) > -0.5 * std::numeric_limits<double>::max()
                && img_alt.at<uchar>(iy, ix) <= MAX_VALID) {
              img_alt.at<uchar>(iy, ix) = static_cast<uchar>(
                  weight * (mat[3](iy, ix) - min_z) + decay * img_alt.at<uchar>(iy, ix));
            } else if (mat[3](iy, ix) > -0.5 * std::numeric_limits<double>::max()) {
              img_alt.at<uchar>(iy, ix) = static_cast<uchar>(mat[3](iy, ix) - min_z);
            }
          }
        }
        cv::imwrite(int_path, img_int); 
        cv::imwrite(ele_path, img_ele); 
        cv::imwrite(alt_path, img_alt);
        LOG(INFO) << "merge intensity image " << int_path;
        LOG(INFO) << "merge elevation image " << ele_path;  
        LOG(INFO) << "merge altitude image " << alt_path << std::endl; 
      }
      
    }
  }

  // mergeMap();
  ground_cloud_->clear();
  altitude_cloud_->clear();
}

void LidarMapper::mergeMap() {
  DIR *dir = NULL;
  if ((dir = opendir(out_int_dir_.c_str())) == NULL) {                                 
    LOG(FATAL) << out_int_dir_ << ": Open dir failed.";                              
  }                                                                              
  struct dirent *ptr;                                                            
  while (ptr = readdir(dir)) {                                                   
    if (ptr->d_name[0] == '.' || ptr->d_type == DT_DIR) {                      
      continue;                                                              
    }                                                                          
    std::string file_name(ptr->d_name);
    if (file_name.size() < 4 || (file_name.substr(file_name.size() - 3) != "png" 
        && file_name.substr(file_name.size() - 3) != "jpg")) {
      continue;
    }                                       
    
    std::string r_int_path = out_int_dir_ + file_name;
    std::string r_ele_path = out_ele_dir_ + file_name;
    std::string r_alt_path = out_alt_dir_ + file_name;
    
    int s = file_name.find_last_of('_');
    std::istringstream iss(file_name.substr(s + 1, file_name.size() - 4));
    double z;
    iss >> z;
    for (int k = -2; k <= 2; ++k) {
      if (k == 0) continue;
      double zz = z + k;
      std::ostringstream oss;
      oss << file_name.substr(0, s + 1) 
          << std::setiosflags(std::ios::fixed) << std::setprecision(2) << zz 
          << file_name.substr(file_name.size() - 4);
      std::string int_path = out_int_dir_ + oss.str();
      std::string ele_path = out_ele_dir_ + oss.str();
      std::string alt_path = out_alt_dir_ + oss.str();
      if (0 == access(int_path.c_str(), F_OK) && 0 == access(ele_path.c_str(), F_OK) 
          && 0 == access(alt_path.c_str(), F_OK)) {
        cv::Mat int_img = cv::imread(int_path, -1);
        cv::Mat ele_img = cv::imread(ele_path, -1);
        cv::Mat alt_img = cv::imread(alt_path, -1);

        cv::Mat r_int_img = cv::imread(r_int_path, -1);
        cv::Mat r_ele_img = cv::imread(r_ele_path, -1);
        cv::Mat r_alt_img = cv::imread(r_alt_path, -1);

        int dg = static_cast<int>(k / z_res_); 
        if (k < 0) {
          cv::Mat temp;
          cv::Mat mask = (r_ele_img <= MAX_VALID);
          temp = cv::Mat::zeros(size_, size_, CV_8UC1);
          r_ele_img.copyTo(temp, mask);
          temp += -dg;
          mask = (temp >= -dg & temp <= MAX_VALID);
          temp.copyTo(r_ele_img, mask);

          mask = (r_alt_img <= MAX_VALID);
          temp = cv::Mat::zeros(size_, size_, CV_8UC1);
          r_alt_img.copyTo(temp, mask);
          temp += -dg;
          mask = (temp >= -dg & temp <= MAX_VALID);
          temp.copyTo(r_alt_img, mask);

          mask = ~(int_img <= MAX_VALID);
          r_int_img.copyTo(int_img, mask);
          mask = ~(ele_img <= MAX_VALID);
          r_ele_img.copyTo(ele_img, mask);
          mask = ~(alt_img <= MAX_VALID);
          r_alt_img.copyTo(alt_img, mask);

          cv::imwrite(int_path, int_img); 
          cv::imwrite(ele_path, ele_img); 
          cv::imwrite(alt_path, alt_img);

          remove(r_int_path.c_str());
          remove(r_ele_path.c_str());
          remove(r_alt_path.c_str());
          LOG(INFO) << oss.str() << " <<<--- " << file_name;
        } else {
          cv::Mat temp;
          cv::Mat mask = (ele_img <= MAX_VALID);
          temp = cv::Mat::zeros(size_, size_, CV_8UC1);
          ele_img.copyTo(temp, mask);
          temp += -dg;
          mask = (temp >= -dg & temp <= MAX_VALID);
          temp.copyTo(ele_img, mask);

          mask = (alt_img <= MAX_VALID);
          temp = cv::Mat::zeros(size_, size_, CV_8UC1);
          alt_img.copyTo(temp, mask);
          temp += -dg;
          mask = (temp >= -dg & temp <= MAX_VALID);
          temp.copyTo(alt_img, mask);

          mask = ~(r_int_img <= MAX_VALID);
          int_img.copyTo(r_int_img, mask);
          mask = ~(r_ele_img <= MAX_VALID);
          ele_img.copyTo(r_ele_img, mask);
          mask = ~(r_alt_img <= MAX_VALID);
          alt_img.copyTo(r_alt_img, mask);

          cv::imwrite(r_int_path, r_int_img); 
          cv::imwrite(r_ele_path, r_ele_img); 
          cv::imwrite(r_alt_path, r_alt_img);

          remove(int_path.c_str());
          remove(ele_path.c_str());
          remove(alt_path.c_str());
          LOG(INFO) << file_name << " <<<--- " << oss.str();
        } 
        dir = opendir(out_int_dir_.c_str());
        break;
      }
    }

  }  
}

void LidarMapper::fillMean(cv::Mat &mat) {                                                  
  cv::Mat mask = ~(mat <= MAX_VALID);                    
  int step = mat.cols;                                                          
  while (step > 1) {                                                            
    for (size_t h = 0; h <= mat.rows - step; h+= step) {                        
      for (size_t w = 0; w <= mat.cols - step; w+= step) {                      
        cv::Rect roi = cv::Rect(w, h, step, step);                              
        if (cv::sum(~mask(roi))[0] < 1) continue;                               
        uchar m = cv::mean(mat(roi), ~mask(roi))[0];                           
        mat(roi).setTo(m, mask(roi));                                           
      }                                                                         
    }                                                                           
    step /= 2;                                                                  
  }                                                                             
}

void LidarMapper::fillZero(cv::Mat &mat) {                                                  
  cv::Mat mask = ~(mat <= MAX_VALID);
  mat.setTo(0, mask);                                                                                               
}

void LidarMapper::postProcess() {
  DIR *dir = NULL;
  if ((dir = opendir(out_int_dir_.c_str())) == NULL) {                                 
    LOG(FATAL) << out_int_dir_ << ": Open dir failed.";                              
  }                                                                              
  struct dirent *ptr;                                                            
  while (ptr = readdir(dir)) {                                                   
    if (ptr->d_name[0] == '.' || ptr->d_type == DT_DIR) {                      
      continue;                                                              
    }                                                                          
    std::string file_name(ptr->d_name);
    if (file_name.size() < 4 || (file_name.substr(file_name.size() - 3) != "png" 
        && file_name.substr(file_name.size() - 3) != "jpg")) {
      continue;
    }                                       
    
    std::string int_path = out_int_dir_ + "/" + file_name;
    cv::Mat img_int = cv::imread(int_path, -1);
    fillZero(img_int);
    img_int *= 10;
    cv::imwrite(int_path, img_int);
  }  

  ///
  if ((dir = opendir(out_ele_dir_.c_str())) == NULL) {                                 
    LOG(FATAL) << out_ele_dir_ << ": Open dir failed.";                              
  }                                                                     
  while (ptr = readdir(dir)) {                                                   
    if (ptr->d_name[0] == '.' || ptr->d_type == DT_DIR) {                      
      continue;                                                              
    }                                                                          
    std::string file_name(ptr->d_name);
    if (file_name.size() < 4 || (file_name.substr(file_name.size() - 3) != "png" 
        && file_name.substr(file_name.size() - 3) != "jpg")) {
      continue;
    }                                       
    
    std::string ele_path = out_ele_dir_ + "/" + file_name;
    cv::Mat img_ele = cv::imread(ele_path, -1);
    fillMean(img_ele);
    cv::imwrite(ele_path, img_ele);
  }  
}

} // namespace mapping
} // namespace horizon

int main(int argc, char **argv) {
  ::google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "lidar_mapper");

  if (argc != 2) {
    LOG(FATAL) << "usage: rosrun lidar_mapper config.yaml";
  } 

ros::start();
  std::string path = argv[1];
  horizon::mapping::LidarMapper mapper(path);

  return 0;
}
