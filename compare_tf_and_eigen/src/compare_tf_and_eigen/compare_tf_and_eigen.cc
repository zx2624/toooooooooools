#include <iostream>

#include <tf/tf.h>

#include <Eigen/Eigen>

int main(int argc, char** argv) {

	Eigen::Quaterniond quat_zero(1, 0, 0, 0);

	Eigen::Matrix3d mat = Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitZ()) *
												Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
												Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()).matrix();
	Eigen::Quaterniond quat_big;
	quat_big = mat;
	std::cout << "Quat_big from eigen matrix: " << quat_big.x() << " " 
		<< quat_big.y() << " " << quat_big.z() << " " << quat_big.w() << std::endl;
	auto quat = quat_zero.slerp(0, quat_big);
	std::cout << "Quat from eigen matrix: " << quat.x() << " " 
		<< quat.y() << " " << quat.z() << " " << quat.w() << std::endl;

	double roll = 0, pitch = 0, yaw = 0;
	tf::Matrix3x3(tf::Quaternion(
				quat.x(), quat.y(), quat.z(), quat.w()
	)).getRPY(roll, pitch, yaw);

	std::cout << "RPY: " << roll << " " << pitch << " " << yaw << std::endl;

#if 0
	double roll = 0;
	double pitch = 0;
	double yaw = 0;

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

#if 0
	Eigen::Matrix4d pre_odom;
	pre_odom << 
		0.627307, -0.778757, 0.004787, -2.394344, 
		0.778771, 0.627300, -0.002955, 1.020830, 
		-0.000701, 0.005582, 0.999984, 232.669088, 
		0.000000, 0.000000, 0.000000, 1.000000;

		//0.627, -0.779, 0.001, -2.361,
		//0.779, 0.627, -0.001, 0.988,
		//0.000, 0.002, 1.000, 232.661,
		//0.000, 0.000, 0.000, 1.000;

	Eigen::Matrix4d next_odom;
	next_odom << 
		0.620740, -0.784006, 0.004139, -2.708991, 
		0.784016, 0.620737, -0.002135, 1.252860, 
		-0.000895, 0.004570, 0.999989, 232.661787, 
		0.000000, 0.000000, 0.000000, 1.000000;
		//0.627, -0.779, 0.003, -2.370,
		//0.779, 0.627, -0.000, 0.997,
		//-0.001, 0.002, 1.000, 232.655,
		//0.000, 0.000, 0.000, 1.000;

	Eigen::Matrix4d mat_icp;
	mat_icp << 
		0.999999, 0.001222, 0.000741, -0.056108, 
		-0.001222, 0.999999, -0.000418, 0.064697, 
		-0.000741, 0.000418, 1.000000, -0.001257, 
		0.000000, 0.000000, 0.000000, 1.000000;
		 //1.000, 0.000, -0.002, 0.370,
		//-0.000, 1.000, -0.001, 0.291,
		//0.002, 0.001, 1.000, 0.009,
		//0.000, 0.000, 0.000, 1.000;
	auto temp_mat = pre_odom.inverse() * mat_icp * next_odom;
	std::cout << pre_odom.inverse() * mat_icp * next_odom << "\n\n";
	std::cout << temp_mat<< "\n\n";
	std::cout << pre_odom.inverse() * next_odom << "\n\n";

#endif
	//double roll = 0.111;
	//double pitch = 0;
	//double yaw = 90 * M_PI / 180.0;
	//Eigen::Matrix3d rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
												//Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
												//Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();
	//std::cout << "rotation:\n" <<  rotation << std::endl;
	//std::cout << "rotation(float):\n" <<  rotation.cast<float>() << std::endl;


	//Eigen::Vector3d translation(-0.156, -0.78, -0.8);
	//std::cout << "translation:\n" <<  translation<< std::endl;

	//translation = -rotation.transpose() * translation;
	//std::cout << "translation:\n" <<  translation.cast<float>() << std::endl;
	//std::cout << "translation(float):\n" <<  translation.cast<float>() << std::endl;
	return 0;
}
