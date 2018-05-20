cmake_minimum_required(VERSION 2.8.3)
project(lidar_mapping)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wreorder -Wsign-compare ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  sensor_msgs
  geometry_msgs  
  std_msgs
  tf
  tf_conversions
  message_filters
  nav_msgs
  pcl_conversions
  velodyne_pointcloud
  message_generation
  robot_localization
  auto_common
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
add_definitions(${OpenCV_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "+++ ======>LOG] OpenCV Include: " ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
message(STATUS "+++ ======>LOG] OpenCV Library: " ${OpenCV_LIBRARY_DIRS})
message(STATUS "+++ ======>LOG] OpenCV Libraries: " ${OpenCV_LIBRARIES})

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
message(STATUS "+++ ======>LOG] PCL Include: " ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
message(STATUS "+++ ======>LOG] PCL Library: " ${PCL_LIBRARY_DIRS})
message(STATUS "+++ ======>LOG] PCL Libraries: " ${PCL_LIBRARIES})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Boost COMPONENTS date_time system filesystem thread program_options random REQUIRED)

#add_message_files(FILES ndt_stat.msg)
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS message_runtime std_msgs velodyne_pointcloud
#  DEPENDS system_lib
)

###########
## Build ##
###########

FILE(GLOB INC_FILES "include/lidar_mapping/*.hpp")
SET(INC_DIRS "include")
SET(SRC_DIRS "src")
SET(COMMON_SRC_FILES
  src/filter.cpp
  src/fusion_optimize.cpp
  src/global_optimize.cpp
  src/hungarian.cpp
  src/local_optimize.cpp
  src/odometry.cpp
  src/parsing.cpp
  src/polation.cpp
  src/util.cpp
  src/visual.cpp
)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${INC_DIRS})

ADD_EXECUTABLE(lidar_mapping 
  src/lidar_mapping_main.cpp
  src/lidar_mapping.cpp
  ${COMMON_SRC_FILES} ${INC_FILES}
)
TARGET_LINK_LIBRARIES(lidar_mapping
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  glog gflags
  boost_system boost_filesystem
)

ADD_EXECUTABLE(odom_lidar_mapping 
  src/odom_lidar_mapping_main.cpp
  src/odom_lidar_mapping.cpp
  ${COMMON_SRC_FILES} ${INC_FILES}
)
TARGET_LINK_LIBRARIES(odom_lidar_mapping
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  glog gflags
  boost_system boost_filesystem boost_serialization
)

ADD_EXECUTABLE(odom_velodyne_mapping                                               
  src/odom_velodyne_mapping_main.cpp                                               
  src/odom_velodyne_mapping.cpp                                                    
  ${COMMON_SRC_FILES} ${INC_FILES}                                              
)                                                                               
TARGET_LINK_LIBRARIES(odom_velodyne_mapping                                        
  ${catkin_LIBRARIES}                                                           
  ${PCL_LIBRARIES}                                                              
  ${OpenCV_LIBS}                                                                
  yaml-cpp                                                                      
  glog gflags                                                                   
  boost_system boost_filesystem boost_serialization                             
)

ADD_EXECUTABLE(offline_velodyne_mapping                                               
  src/offline_velodyne_mapping_main.cpp                                               
  src/offline_velodyne_mapping.cpp                                                    
  ${COMMON_SRC_FILES} ${INC_FILES}                                              
)                                                                               
TARGET_LINK_LIBRARIES(offline_velodyne_mapping                                        
  ${catkin_LIBRARIES}                                                           
  ${PCL_LIBRARIES}                                                              
  ${OpenCV_LIBS}                                                                
  yaml-cpp                                                                      
  glog gflags                                                                   
  boost_system boost_filesystem boost_serialization                             
)

ADD_EXECUTABLE(lidar_mapper                                             
  src/lidar_mapper.cpp                                               
  ${INC_FILES}                                              
)                                                                               
TARGET_LINK_LIBRARIES(lidar_mapper                                     
  ${catkin_LIBRARIES}                                                           
  ${PCL_LIBRARIES}                                                              
  ${OpenCV_LIBS}                                                                
  yaml-cpp                                                                      
  glog gflags                                                                   
  boost_system boost_filesystem boost_serialization                             
) 
