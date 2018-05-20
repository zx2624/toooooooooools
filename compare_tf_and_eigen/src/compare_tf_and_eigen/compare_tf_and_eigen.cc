#include <iostream>

#include <tf/tf.h>

#include <Eigen/Eigen>

int main(int argc, char** argv) {

	double roll = -0.23;
	double pitch = 1.09;
	double yaw = 2.98;

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

	return 0;
}
