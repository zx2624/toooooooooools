#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "serial/serial.h"
#include "geodesy/utm.h"

serial::Serial *rece_serial = NULL;

bool split(const std::string &s, char delim, std::vector<std::string> &elems) {
  elems.clear();
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return true;
}

inline double degree2rad(double degree) {
  return degree / 180.0 * 3.1415926;
}

int main(int argc, const char **argv) {
  ros::init(argc, argv, "novatel_inspvaxa");

  ros::NodeHandle nh("~");

  std::string rece_port = "/dev/ttyS0";
  int rece_baud = 115200;
  nh.getParam("rece_port", rece_port);
  nh.getParam("rece_baud", rece_baud);
  std::cout << "port: " << rece_port << "   baud: " << rece_baud << std::endl;
  rece_serial = new serial::Serial(rece_port, rece_baud, serial::Timeout::simpleTimeout(1000));
  rece_serial->write("LOG INSPVAXA ONTIME 0.1\r\n");

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/sensor/novatel/odom", 1);
  tf::TransformBroadcaster *odom_tf = new tf::TransformBroadcaster();

  while (ros::ok()) {
    std::string rece_str;
    size_t len = rece_serial->readline(rece_str);
      std::cout <<rece_str<< std::endl;

    if (len < 10) {
      continue;
    }
    printf("recv: %s", rece_str.c_str());

    std::vector<std::string> secs;
    split(rece_str, ';', secs);
    if (secs.size() != 2) {
      continue;
    }
    std::vector<std::string> elems;
    split(secs[1], ',', elems);

    std::string solution_status;
    double latitude, longitude, height, undulation;
    double north_velocity, east_velocity, up_velocity;
    double roll, pitch, azimuth;
    double latitude_std_dev, longitude_std_dev, height_std_dev;
    double north_velocity_std_dev, east_velocity_std_dev, up_velocity_std_dev;
    double roll_std_dev, pitch_std_dev, azimuth_std_dev;

    solution_status = elems[0];
    std::istringstream(elems[2]) >> latitude;
    std::istringstream(elems[3]) >> longitude;
    std::istringstream(elems[4]) >> height;
    std::istringstream(elems[5]) >> undulation;
    std::istringstream(elems[6]) >> north_velocity;
    std::istringstream(elems[7]) >> east_velocity;
    std::istringstream(elems[8]) >> up_velocity;
    std::istringstream(elems[9]) >> roll;
    std::istringstream(elems[10]) >> pitch;
    std::istringstream(elems[11]) >> azimuth;
    
    std::istringstream(elems[12]) >> latitude_std_dev;
    std::istringstream(elems[13]) >> longitude_std_dev;
    std::istringstream(elems[14]) >> height_std_dev;
    std::istringstream(elems[15]) >> north_velocity_std_dev;
    std::istringstream(elems[16]) >> east_velocity_std_dev;
    std::istringstream(elems[17]) >> up_velocity_std_dev;
    std::istringstream(elems[18]) >> roll_std_dev;
    std::istringstream(elems[19]) >> pitch_std_dev;
    std::istringstream(elems[20]) >> azimuth_std_dev;
 
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
    //odom.child_frame_id  = "novatel";
    odom.child_frame_id  = "chassis_base";
    geographic_msgs::GeoPoint gp;
    gp.latitude = latitude;
    gp.longitude = longitude;
    gp.altitude = height;
    geodesy::UTMPoint pt(gp);
    odom.pose.pose.position.x = pt.easting;
    odom.pose.pose.position.y = pt.northing;
    odom.pose.pose.position.z = pt.altitude;

    double yaw = -(azimuth - 90);

    tf::Quaternion qua;
    qua.setRPY(degree2rad(roll), degree2rad(pitch), degree2rad(yaw));
    odom.pose.pose.orientation.x = qua.x(); 
    odom.pose.pose.orientation.y = qua.y();
    odom.pose.pose.orientation.z = qua.z();
    odom.pose.pose.orientation.w = qua.w();

 std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(9) 
              << "wgs84_position: (" << longitude << ", " << latitude << ", " << height << ")   ";
    std::cout << pt.zone << pt.band << "   ";
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3) 
              << "utm_position: (" << pt.easting << ", " << pt.northing << ", " << pt.altitude << ")   degree: (" << roll << ", " << pitch << ", " << yaw << ")"<< std::endl;  
    for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0;
    odom.pose.covariance[0] = latitude_std_dev * latitude_std_dev;
    odom.pose.covariance[7] = longitude_std_dev * longitude_std_dev;
    odom.pose.covariance[14] = height_std_dev * height_std_dev;
    odom.pose.covariance[21] = degree2rad(roll_std_dev) * degree2rad(roll_std_dev);
    odom.pose.covariance[28] = degree2rad(pitch_std_dev) * degree2rad(pitch_std_dev);
    odom.pose.covariance[35] = degree2rad(azimuth_std_dev) * degree2rad(azimuth_std_dev);
    if(solution_status.compare("INS_SOLUTION_GOOD") == 0)
    {
      //INS_SOLUTION_GOOD
      odom.pose.covariance[1] = 0;
    }
    else if(solution_status.compare("INS_ALIGNING") == 0)
    {
      odom.pose.covariance[1] = 1;
    }
    else if(solution_status.compare("INS_HIGH_VARIANCE") == 0)
    {
      odom.pose.covariance[1] = 2;
    }
    else if(solution_status.compare("INS_INACTIVE") == 0)
    {
      odom.pose.covariance[1] = 3;
    }
    else if(solution_status.compare("INS_SOLUTION_FREE") == 0)
    {
      odom.pose.covariance[1] = 6;
    }
    else if(solution_status.compare("INS_ALIGNMENT_COMPLETE") == 0)
    {
      odom.pose.covariance[1] = 7;
    }
    else if(solution_status.compare("DETERMINING_ORIENTATION") == 0)
    {
      odom.pose.covariance[1] = 8;
    }
    else if(solution_status.compare("WAITING_INITIALPOS") == 0)
    {
      odom.pose.covariance[1] = 9;
    }
    else
    {
      odom.pose.covariance[1] = 10;
    }
    
    odom.twist.twist.linear.x = east_velocity;
    odom.twist.twist.linear.y = north_velocity;
    odom.twist.twist.linear.z = up_velocity;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    for (int i = 0; i < 36; ++i) odom.twist.covariance[i] = 0;
    odom.twist.covariance[0] = east_velocity_std_dev * east_velocity_std_dev;
    odom.twist.covariance[7] = north_velocity_std_dev * north_velocity_std_dev;
    odom.twist.covariance[14] = up_velocity_std_dev * up_velocity_std_dev;
    
    odom_pub.publish(odom);
  }