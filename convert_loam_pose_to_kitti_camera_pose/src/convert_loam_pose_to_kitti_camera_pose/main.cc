// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     main.cc
// ROLE:     TODO (some explanation)
// CREATED:  2018-07-18 19:11:40
// MODIFIED: 2018-07-27 13:25:03
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <tf/tf.h>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
	if (argc < 4) {
		cout << "usage: rosrun convert_loam_pose_to_kitti_camera_pose "
			<< "convert_loam_pose_to_kitti_camera_pose "
			<< "lidar_pose_file calib_file output_xyzrpy_file\n";
		return -1;
	}
	ifstream ifs;

	//load lidar to camera transform
	ifs.open(argv[2]);
	Matrix4d  camera_to_lidar = Matrix4d::Identity();
	for (auto i = 0; i < 5; i++) {
		string str;
		getline(ifs, str);
		if (i == 4) {
			stringstream ss(str);
			string temp;
			ss >> temp;
			for (auto row = 0; row < 3; row++)
				for (auto col = 0; col < 4; col++) {
					ss >> camera_to_lidar(row, col);
				}
		}
	}
	ifs.close();

	cout << "camera_to_lidar:\n" << camera_to_lidar << "\n";

	//load loam odom
	ifs.open(argv[1]);
	//output time xyz rpy file
	ofstream ofs;
	ofs.open(argv[3]);
	//stamp
	double stamp = 0;
	//translation
	double x = 0 , y = 0, z = 0;
	//rpy
	double roll = 0, pitch = 0, yaw = 0;

	size_t i = 0;
	Matrix4d lidar = Matrix4d::Identity();
	while(ifs >> stamp >> x >> y >> z >> roll >> pitch >> yaw
			) {
		Matrix3d rotation =
			Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();
		lidar.block<3, 3>(0, 0) = rotation;
		lidar(0, 3) = x;
		lidar(1, 3) = y;
		lidar(2, 3) = z;

		Matrix4d camera = camera_to_lidar * lidar * camera_to_lidar.inverse();
		double roll = 0, pitch = 0, yaw = 0;

		Quaterniond quat(lidar.block<3, 3>(0, 0));
		tf::Matrix3x3(tf::Quaternion(
					quat.x(),
					quat.y(),
					quat.z(),
					quat.w()
					)).getRPY(roll, pitch, yaw);

		ofs << stamp << " ";
		for (auto row = 0; row < 3; row++) {
			for (auto col = 0; col < 4; col++) {
				ofs << camera(row, col) << " ";
			}
		}
		ofs << endl;
		i++;
	}
	ifs.close();
	ofs.close();
	cout << "Write " << i << " pose!\n";

	return 0;
}

