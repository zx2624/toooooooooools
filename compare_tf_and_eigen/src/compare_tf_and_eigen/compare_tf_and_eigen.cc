#include <iostream>

#include <tf/tf.h>

#include <Eigen/Eigen>

int main(int argc, char** argv) {

#if 1
	double roll = 1.55559;
	double pitch = -0.00006;
	double yaw = -1.563;

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
#endif

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

	//Eigen::Quaternionf	quat1(-0.0526022, 0.763397, 0.0382588, -0.642646);
	//Eigen::Matrix4f t1 = Eigen::Matrix4f::Identity();
	//Eigen::Matrix3f r1(quat1);
	//t1.block<3, 3>(0, 0) = r1;
	//t1(0, 3) = -5.2008;
	//t1(1, 3) = 85.5356;
	//t1(2, 3) = -77.1247;

	//Eigen::Quaternionf	quat2(-0.0506864, 0.779315, 0.0718127, -0.620437);
	//Eigen::Matrix4f t2 = Eigen::Matrix4f::Identity();
	//Eigen::Matrix3f r2(quat2);
	//t2.block<3, 3>(0, 0) = r2;
	//t2(0, 3) = 1.0318;
	//t2(1, 3) = 85.323;
	//t2(2, 3) = -76.0615;

	//auto t1_2 = t1.inverse() * t2;
	//Eigen::Quaternionf quat1_2(t1_2.block<3, 3>(0, 0));
	//std::cout << "[pos, quat]: "
		//<< t1_2(0, 3) << " " << t1_2(1, 3) << " " << t1_2(2, 3) << " "
		//<< quat1_2.x() << " " << quat1_2.y() << " "
		//<< quat1_2.z() << " " << quat1_2.w() << "\n"; 

	return 0;
}
