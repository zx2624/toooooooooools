cmake_minimum_required (VERSION 2.8.12)

project(pcltool CXX)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
add_compile_options(-std=c++11 -Wall -Wpedantic)

find_package(PCL REQUIRED COMPONENTS  io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIBS ${PCL_LIBRARIES} glog gflags)

add_executable(pcd2ply pcd2ply.cpp)
target_link_libraries(pcd2ply ${LIBS})

add_executable(pcd2asc pcd2asc.cpp)
target_link_libraries(pcd2asc ${LIBS})

add_executable(transform_pcd transform_pcd.cpp)
target_link_libraries(transform_pcd ${LIBS})
