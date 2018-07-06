#include <iostream>

#include <tf/tf.h>

#include <Eigen/Eigen>

int main(int argc, char** argv) {

	double roll = -0.0005;
	double pitch = 0.05;
	double yaw = -1.57;

	Eigen::Matrix3d mat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
												Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
												Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();
	std::cout << "Eigen matrix from rpy:\n" << mat << "\n\n";

	tf::Matrix3x3 tf_mat;
	tf_mat.setRPY(roll, pitch, yaw);

	std::cout << "tf matrix from rpy:\n";
	for(int i = 0; i < 3; i++) {
		auto row = tf_mat.getRow(i);
		std::cout << row.x() << " " << row.y() << " " << row.z() << std::endl; 
	}
	auto rpy = mat.eulerAngles(2, 1, 0);
	std::cout << "\n rpy from eigen matrix: " << rpy(2) << " " << rpy(1) 
		<< " " << rpy(0) << std::endl;
	Eigen::Vector3d rpy_tf;
	tf_mat.getRPY(rpy_tf(0), rpy_tf(1), rpy_tf(2));
	std::cout << "\n rpy from tf matrix: " << rpy_tf << std::endl;

	Eigen::Quaterniond quat(mat);
	std::cout << "Quat from eigen matrix: " << quat.x() << " " 
		<< quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
	tf::Quaternion tf_quat;
	tf_mat.getRotation(tf_quat);
	std::cout << "Quat from tf matrix: " << tf_quat.x() << " "
		<< tf_quat.y() << " " << tf_quat.z() << " " << tf_quat.w() << std::endl;
	
	mat = quat.toRotationMatrix();
	std::cout << "Mat from eigen quat:\n" << mat << "\n\n";

	tf_mat.setRotation(tf_quat);
	std::cout << "Mat from tf quat:\n";
	for(int i = 0; i < 3; i++) {
		auto row = tf_mat.getRow(i);
		std::cout << row.x() << " " << row.y() << " " << row.z() << std::endl; 
	}

	Eigen::Affine3d af = Eigen::Affine3d::Identity();
	af.rotate(mat);
	af.translate(Eigen::Vector3d(0, 0, 0));

	af = Eigen::Affine3d::Identity() * af;

	auto rpy_af = af.rotation().eulerAngles(2, 1, 0);
	std::cout << "\n rpy from affine matrix: " << rpy_af(2) << " " << rpy_af(1) 
		<< " " << rpy_af(0) << std::endl;

	//std::cout << "------------------------------\n";
	//double quat_x = 0.014831;
	//double quat_y = 0.000107;
	//double quat_z = -0.999830;
	//double quat_w = 0.010926;

	//tf::Quaternion tf_quat(quat_x, quat_y, quat_z, quat_w);
	//tf::Matrix3x3 tf_mat(tf_quat);
	//std::cout << "\n mat from tf quat: "  << std::endl;
	//for(int i = 0; i < 3; i++) {
		//auto row = tf_mat.getRow(i);
		//std::cout << row.x() << " " << row.y() << " " << row.z() << std::endl; 
	//}
	//Eigen::Vector3d rpy_tf;
	//tf_mat.getRPY(rpy_tf(0), rpy_tf(1), rpy_tf(2));
	//std::cout << "\n rpy from tf matrix: " << rpy_tf << std::endl;

	//Eigen::Quaterniond eigen_quat(quat_w, quat_x, quat_y, quat_z);
	//Eigen::Matrix3d eigen_mat = eigen_quat.toRotationMatrix();
	//std::cout << "matrix from eigen_quat:\n" << eigen_mat << "\n\n";
	//auto rpy = eigen_mat.eulerAngles(0, 1, 2);
	//std::cout << "\n rpy from eigen matrix: " << rpy(0) << " " << rpy(1) 
		//<< " " << rpy(2) << std::endl;

	return 0;
}
