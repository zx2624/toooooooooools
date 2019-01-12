// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     plot_image_pasring_results.cc
// ROLE:     TODO (some explanation)
// CREATED:  2019-01-07 17:20:44
// MODIFIED: 2019-01-11 20:31:46
#include <stdio.h>
#include<stdarg.h>

#include <iostream>
#include <memory>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <yaml-cpp/yaml.h>

#include <mapping_tools/mapping_tools.h>

typedef pcl::PointXYZRGBL PointType;
typedef pcl::Label PointLabel;
typedef pcl::PointXYZRGB PointRGB;

//class id num
const int CLASSID_NUM = 16;

//invalid_class_id
const unsigned char INVALID_CLASS_ID = 255;

//max point num of each frame pointcloud
const int MAX_POINTS_NUM = 150000;

//const float MASK_VALUE = 0.7;
const float MASK_VALUE = 0;

const float MIN_DISTANCE = 0.0;
const float MAX_DISTANCE = 100.0;

const int colorlist[24][3] = {
    {128, 64, 128},
    {244, 35, 232},
    {107, 142, 35},
    {152, 251, 152},
    {153, 153, 153},
    {220, 220, 0},
    {250, 170, 30},
    {200, 200, 128},
    {200, 200, 200},
    {220, 20, 60},
    {0, 0, 70},
    {0, 0, 142},
    {70, 70, 70},
    {190, 153, 153},
    {70, 130, 180},
    {0, 64, 64},
    {128, 128, 192},
    {192, 192, 0},
    {64, 192, 0},
    {128, 0, 192},
    {192, 192, 128},
    {255, 0, 0},
    {102, 102, 156},
    {0, 0, 230}
  };

//colormap HSV
static const float R[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9523809523809526f, 0.8571428571428568f, 0.7619047619047614f, 0.6666666666666665f, 0.5714285714285716f, 0.4761904761904763f, 0.3809523809523805f, 0.2857142857142856f, 0.1904761904761907f, 0.0952380952380949f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09523809523809557f, 0.1904761904761905f, 0.2857142857142854f, 0.3809523809523809f, 0.4761904761904765f, 0.5714285714285714f, 0.6666666666666663f, 0.7619047619047619f, 0.8571428571428574f, 0.9523809523809523f, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static const float G[] = { 0, 0.09523809523809523f, 0.1904761904761905f, 0.2857142857142857f, 0.3809523809523809f, 0.4761904761904762f, 0.5714285714285714f, 0.6666666666666666f, 0.7619047619047619f, 0.8571428571428571f, 0.9523809523809523f, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9523809523809526f, 0.8571428571428577f, 0.7619047619047619f, 0.6666666666666665f, 0.5714285714285716f, 0.4761904761904767f, 0.3809523809523814f, 0.2857142857142856f, 0.1904761904761907f, 0.09523809523809579f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const float B[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09523809523809523f, 0.1904761904761905f, 0.2857142857142857f, 0.3809523809523809f, 0.4761904761904762f, 0.5714285714285714f, 0.6666666666666666f, 0.7619047619047619f, 0.8571428571428571f, 0.9523809523809523f, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9523809523809526f, 0.8571428571428577f, 0.7619047619047614f, 0.6666666666666665f, 0.5714285714285716f, 0.4761904761904767f, 0.3809523809523805f, 0.2857142857142856f, 0.1904761904761907f, 0.09523809523809579f, 0};
  
class CameraCalibration {
 public:
  //  intrinsic
  cv::Mat cameraK;
  cv::Mat cameraD;
  //  extrinsic
  std::vector<double> cameraT;  //  x y z
  std::vector<double> cameraR;  //  q0(w) q1(x) q2(y) q3(z)
};

class Config {
 public:
	std::string bag_path;

	std::string raw_point_topic;
	std::string label_point_topic;

	std::vector<std::string> raw_image_topic_vec;
	std::vector<std::string> parsing_image_topic_vec;
};

void convert_cv_vec(const double array[], cv::Mat &rvec, cv::Mat &tvec) {
	rvec = (cv::Mat_<double>(3, 1) << array[0], array[1], array[2]);
	tvec = (cv::Mat_<double>(3, 1) << array[3], array[4], array[5]);
}

void convert_camera_matrix(const double array[], cv::Mat &camera_matrix) {
	camera_matrix = (cv::Mat_<double>(3, 3) << array[0], 0.0, array[2], 0.0,
		array[1], array[3], 0.0, 0.0, 1.0);
}

void convert_camera_dist(const double array[], cv::Mat &camera_dist) {
	camera_dist = (cv::Mat_<double>(5, 1) << array[0], array[1], array[2],
		array[3], array[4]);
}

void convert_rot(const double array[], double rr[]) {
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << array[0], array[1], array[2]);
  cv::Mat rot;
  cv::Rodrigues(rvec, rot);
  for (int i = 0; i < 9; ++i) {
    rr[i] = rot.at<double>(i);
  }
}

void convert_rot_trans(const double array[], double rr[], double tt[]) {
  convert_rot(array, rr);
  for (int i = 0; i < 3; ++i) {
		tt[i] = array[3 + i];
	}
}
bool convert_inv_extrinsics(const double extrinsics_src[],
		                            double extrinsics_inv[]) {
	double rr[9];
	double tt[3];
	convert_rot_trans(extrinsics_src, rr, tt);
	extrinsics_inv[0] = -extrinsics_src[0];
	extrinsics_inv[1] = -extrinsics_src[1];
	extrinsics_inv[2] = -extrinsics_src[2];
	extrinsics_inv[3] = -rr[0] * tt[0] - rr[3] * tt[1] - rr[6] * tt[2];
	extrinsics_inv[4] = -rr[1] * tt[0] - rr[4] * tt[1] - rr[7] * tt[2];
	extrinsics_inv[5] = -rr[2] * tt[0] - rr[5] * tt[1] - rr[8] * tt[2];
}

void ShowManyImages(std::string title, int nArgs, ...) {
	using namespace cv;
	using namespace std;
	int size;
	int i;
	int m, n;
	int x, y;

	// w - Maximum number of images in a row
	// h - Maximum number of images in a column
	int w, h;

	// scale - How much we have to resize the image
	float scale;
	int max;

	// If the number of arguments is lesser than 0 or greater than 12
	// return without displaying
	if(nArgs <= 0) {
			printf("Number of arguments too small....\n");
			return;
	}
	else if(nArgs > 14) {
			printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
			return;
	}
	// Determine the size of the image,
	// and the number of rows/cols
	// from number of arguments
	else if (nArgs == 1) {
			w = h = 1;
			size = 300;
	}
	else if (nArgs == 2) {
			w = 2; h = 1;
			size = 300;
	}
	else if (nArgs == 3 || nArgs == 4) {
			w = 2; h = 2;
			size = 300;
	}
	else if (nArgs == 5 || nArgs == 6) {
			w = 3; h = 2;
			//size = 200;
			size = 600;
	}
	else if (nArgs == 7 || nArgs == 8) {
			w = 4; h = 2;
			size = 200;
	}
	else {
			w = 4; h = 3;
			size = 150;
	}

	// Create a new 3 channel image
	Mat DispImage = Mat::zeros(Size(100 + size*w, 60 + size*h), CV_8UC3);

	// Used to get the arguments passed
	va_list args;
	va_start(args, nArgs);

	// Loop for nArgs number of arguments
	for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
			// Get the Pointer to the IplImage
			Mat img = va_arg(args, Mat);

			// Check whether it is NULL or not
			// If it is NULL, release the image, and return
			if(img.empty()) {
					printf("Invalid arguments");
					return;
			}

			// Find the width and height of the image
			x = img.cols;
			y = img.rows;

			// Find whether height or width is greater in order to resize the image
			max = (x > y)? x: y;

			// Find the scaling factor to resize the image
			scale = (float) ( (float) max / size );

			// Used to Align the images
			if( i % w == 0 && m!= 20) {
					m = 20;
					n+= 20 + size;
			}

			// Set the image ROI to display the current image
			// Resize the input image and copy the it to the Single Big Image
			Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
			Mat temp; resize(img,temp, Size(ROI.width, ROI.height));
			temp.copyTo(DispImage(ROI));
	}

	// Create a new window, and show the Single Big Image
	namedWindow( title, 1 );
	imshow( title, DispImage);
	cv::waitKey(1);

	// End the number of arguments
	va_end(args);
}

bool project_cloud_to_image_colored(pcl::PointCloud<PointType>::Ptr cloud,
														//pcl::PointCloud<PointLabel>& label_cloud,
														std::vector<std::vector<unsigned char>>& label_vec_vec,
                            const double intrinsic[], const double distortion[],
                            const double extrinsic[],
                            cv::Mat &image, const cv::Mat &parsing_image) {
  if (cloud->empty() || image.empty()) {
    return false;
  }
  cv::Mat rvec;
  cv::Mat tvec;
  convert_cv_vec(extrinsic, rvec, tvec);
  cv::Mat camera_k;
  convert_camera_matrix(intrinsic, camera_k);
  cv::Mat camera_d;
  convert_camera_dist(distortion, camera_d);

  int point_count = cloud->points.size();
  std::vector<cv::Point3f> pt3d_vec;
  std::vector<int> pt3d_idx;
  std::vector<float> pt3d_dis;

  pt3d_vec.reserve(point_count);
  pt3d_idx.reserve(point_count);
  pt3d_dis.reserve(point_count);

  double rr[9];
  double tt[3];
  convert_rot_trans(extrinsic, rr, tt);
  double xx[3];
  double yy[3];
  float img_pt[2];
  int width = image.cols;
  int height = image.rows;
  float ratio = 0.5;
  float min_w = -ratio * width;
  float max_w = (1 + ratio) * width;
  float min_h = -ratio * height;
  float max_h = (1 + ratio) * height;

  for (int i = 0; i < point_count; ++i) {
    xx[0] = cloud->points[i].x;
    xx[1] = cloud->points[i].y;
    xx[2] = cloud->points[i].z;
    if (std::isnan(xx[0]) || std::isnan(xx[1]) || std::isnan(xx[2])) {
      continue;
    }


    yy[2] = rr[6] * xx[0] + rr[7] * xx[1] + rr[8] * xx[2] + tt[2];
    if (yy[2] < 1.0) {
      continue;
    }
		auto yy2_backup = yy[2];
    yy[2] = 1.0 / yy[2];
    yy[0] = rr[0] * xx[0] + rr[1] * xx[1] + rr[2] * xx[2] + tt[0];
    img_pt[0] = yy[0] * yy[2] * intrinsic[0] + intrinsic[2];
    if (img_pt[0] < min_w || img_pt[0] > max_w) {
      continue;
    }
    yy[1] = rr[3] * xx[0] + rr[4] * xx[1] + rr[5] * xx[2] + tt[1];
    img_pt[1] = yy[1] * yy[2] * intrinsic[1] + intrinsic[3];
    if (img_pt[1] < min_h || img_pt[1] > max_h) {
      continue;
    }

		//Judge if the point which has been transformed to camera
		//coordinate is too close or too far.
		auto dis = sqrt(pow(yy[0], 2) + pow(yy[1], 2) + pow(yy2_backup, 2));
		if (dis < MIN_DISTANCE || dis > MAX_DISTANCE) continue;

    pt3d_vec.push_back(cv::Point3f(xx[0], xx[1], xx[2]));
    pt3d_idx.push_back(i);
		pt3d_dis.push_back(dis);
  }

  if (pt3d_vec.empty()) {
    return false;
  }

  std::vector<cv::Point2f> img_pts;
  cv::projectPoints(pt3d_vec, rvec, tvec, camera_k, camera_d, img_pts);
  for (int k = 0; k < pt3d_idx.size(); ++k) {
    int x = floor(img_pts[k].x + 0.5);
    int y = floor(img_pts[k].y + 0.5);
    if (x < 0 || x >= image.cols || y < 0 || y >= image.rows) {
      continue;
    }

		//get id
		auto class_id  = parsing_image.at<cv::Vec3b>(y, x)[0];

		//Let the class_id' value ++
		label_vec_vec[pt3d_idx[k]][class_id]++;

		//draw circle at pixel that point can be projected into

		auto point = cloud->points[pt3d_idx[k]];
		int color_idx = static_cast<int>(64 * (pt3d_dis[k] - MIN_DISTANCE) 
			/ (MAX_DISTANCE - MIN_DISTANCE));

		//circle points based on distance
		//circle(image, cv::Point(x, y), 1, cv::Scalar(255 * B[color_idx],
			//255 * G[color_idx], 255 * R[color_idx]), 1);

		//circle points based on label
		circle(image, cv::Point(x, y), 2, cv::Scalar(colorlist[class_id][2],
			colorlist[class_id][1], colorlist[class_id][0]), 2);

    //if(cloud->points[pt3d_idx[k]].r == cloud->points[pt3d_idx[k]].g &&
       //cloud->points[pt3d_idx[k]].r == cloud->points[pt3d_idx[k]].b)
    //{
      //cloud->points[pt3d_idx[k]].r = image.at<cv::Vec3b>(y, x)[2];
      //cloud->points[pt3d_idx[k]].g = image.at<cv::Vec3b>(y, x)[1];
      //cloud->points[pt3d_idx[k]].b = image.at<cv::Vec3b>(y, x)[0];
    //}

  }
  return true;
}

std::vector<std::vector<unsigned char> > label_vec_vec(MAX_POINTS_NUM,
	std::vector<unsigned char>(CLASSID_NUM, 0));

void caculate_label(const std::vector<std::vector<unsigned char> > &label_vec_vec,
	pcl::PointCloud<PointLabel>& point_label) {

	//label_vec_vec.size() must equal to point_label.size()
	//assert(label_vec_vec.size() == point_label.size());

	for (auto i = 0; i < point_label.size(); i++) {
		unsigned char max_num = 0;
		unsigned char idx = 0;
		for (auto j = 0; j < CLASSID_NUM; j++) {
			if (label_vec_vec[i][j] > max_num) {
				max_num = label_vec_vec[i][j];
				idx = j;
			}
		}
		if (max_num > 0) {
			//set to the label whose amount is maximum.
			point_label.points[i].label = idx;
		}
		else {
			//This point have no label, we set it to 255 defaultly.
			point_label.points[i].label = INVALID_CLASS_ID;
		}
	}

}

void plot_parsing_result(const Config &config, const CameraCalibration camera_calibration[]) {
	//vector of bag file
	std::vector<std::string> vec_bag_files;
	//get all bag files at bag_path
	horizon::mapping::getAllBagFilesPath(config.bag_path, vec_bag_files);

#ifdef VIEW_POINTCLOUD
	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.addCoordinateSystem(3.0 ,"coor");
	viewer.initCameraParameters();
	viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, -1.0, 0.0);
	//viewer.setFullScreen(true);
#endif
	for (auto &bag_file:vec_bag_files) {
		rosbag::Bag bag;
		bag.open(bag_file, rosbag::bagmode::Read);

		rosbag::View points_view(bag, rosbag::TopicQuery(config.raw_point_topic));
		auto point_it = points_view.begin();

		int image_num = config.raw_image_topic_vec.size();
		int frame_num = 0;

		std::vector<rosbag::View> raw_images_view_vec(image_num);
		std::vector<rosbag::View> parsing_images_view_vec(image_num);

		//vector iterator
		std::vector<rosbag::View::iterator> raw_images_it_vec(image_num);
		std::vector<rosbag::View::iterator> parsing_images_it_vec(image_num);

		for (auto i = 0; i < image_num; i++) {
			raw_images_view_vec[i].addQuery(bag, 
				rosbag::TopicQuery(config.raw_image_topic_vec[i]));
			parsing_images_view_vec[i].addQuery(bag,
				rosbag::TopicQuery(config.parsing_image_topic_vec[i]));

			//point size must == image size
			//TODO
			//assert(points_view.size() == raw_images_view_vec[i].size());
			//assert(points_view.size() == parsing_images_view_vec[i].size());

			raw_images_it_vec[i] = raw_images_view_vec[i].begin();
			parsing_images_it_vec[i] = parsing_images_view_vec[i].begin();
		}


		//for test
		//TODO because the /pandora_image4/segdet_track/compressed have 48 frame extraly.
		
		//for (auto i = 0; i < 48; i++ ) {
			//(parsing_images_it_vec[4])++;
		//}
		//for (auto i = 0; i < 3300; i++) {
			//point_it++;
			//frame_num++;
			//for(auto j = 0; j < image_num; j++) {
				//(raw_images_it_vec[j])++;
				//(parsing_images_it_vec[j])++;
			//}
		//}

		//raw point cloud
		pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointRGB>::Ptr source_rgb(new pcl::PointCloud<PointRGB>);
		//define point label point cloud vector
		std::vector<pcl::PointCloud<PointLabel>> label_point_vec(points_view.size());
		
		//pointcloud timestamp vec
		std::vector<std_msgs::Time> label_point_stamp(points_view.size());

		std::vector<cv::Mat> raw_images_color_vec(image_num);
		std::vector<cv::Mat> parsing_images_color_vec(image_num);

		bool have_finished = false;
		while (point_it != points_view.end() && !have_finished) {
			auto point_ptr = point_it->instantiate<sensor_msgs::PointCloud2>();
			pcl::fromROSMsg(*point_ptr, *source);

			//set stamp
			label_point_stamp[frame_num].data = point_ptr->header.stamp;

			std::cout.precision(2);
			std::cout << "[point, img0, img1, img2, img3, img4]: " << std::fixed
				<< point_ptr->header.stamp.toSec();

			for (auto i = 0; i < image_num; i++) {
				auto raw_image_ptr = 
					raw_images_it_vec[i]->instantiate<sensor_msgs::CompressedImage>();
				auto parsing_image_ptr = 
					parsing_images_it_vec[i]->instantiate<sensor_msgs::CompressedImage>();

				//convert from ros msg
				raw_images_color_vec[i] = cv::imdecode(cv::Mat(raw_image_ptr->data), 1);
				parsing_images_color_vec[i] = cv::imdecode(cv::Mat(parsing_image_ptr->data), 1);
		
				std::cout << " " << raw_image_ptr->header.stamp.toSec() - 
					point_ptr->header.stamp.toSec();
			}
			std::cout << std::endl;
			
			//set to zero
			//memset(&label_vec_vec[0][0], 0, sizeof(MAX_POINTS_NUM * CLASSID_NUM));
			for (auto i = 0; i < MAX_POINTS_NUM; i++) {
				for (auto j = 0; j < CLASSID_NUM; j++) {
					label_vec_vec[i][j] = 0;
				}
			}

			label_point_vec[frame_num].resize(source->size());

			for (auto idx = 0; idx < image_num; idx++) {
				for (auto i = 0 ; i < raw_images_color_vec[idx].rows; i++) {
					for (auto j = 0 ; j < raw_images_color_vec[idx].cols; j++) {
						//get class id
						auto id = parsing_images_color_vec[idx].at<cv::Vec3b>(i, j)[0];
						auto color = colorlist[id];
						auto raw_color = raw_images_color_vec[idx].at<cv::Vec3b>(i, j);

						for (auto k = 0; k < raw_images_color_vec[idx].channels(); k++) {
							raw_color[k] =
								static_cast<uchar>(std::min(static_cast<uchar>(color[ 2 - k] * MASK_VALUE + 
									raw_color[k] * (1 - MASK_VALUE)), uchar(255)));
						}
						raw_images_color_vec[idx].at<cv::Vec3b>(i, j) = raw_color;
					}
				}

				//caculate camera_calibration to adapt to project_cloud_to_image_colored function
				double intrinsics[4];
				intrinsics[0] = camera_calibration[idx].cameraK.at<double>(0, 0);
				intrinsics[1] = camera_calibration[idx].cameraK.at<double>(1, 1);
				intrinsics[2] = camera_calibration[idx].cameraK.at<double>(0, 2);
				intrinsics[3] = camera_calibration[idx].cameraK.at<double>(1, 2);

				//because the images hase been undistored
				double distortion[5];
				//distortion[0] = camera_calibration[idx].cameraD.at<double>(0);
				//distortion[1] = camera_calibration[idx].cameraD.at<double>(1);
				//distortion[2] = camera_calibration[idx].cameraD.at<double>(2);
				//distortion[3] = camera_calibration[idx].cameraD.at<double>(3);
				//distortion[4] = camera_calibration[idx].cameraD.at<double>(4);

				distortion[0] = 0;
				distortion[1] = 0;
				distortion[2] = 0;
				distortion[3] = 0;
				distortion[4] = 0;

				double extrinsic[6];

				Eigen::Quaternion<double> rotation(
					camera_calibration[idx].cameraR[0],
					camera_calibration[idx].cameraR[1],
					camera_calibration[idx].cameraR[2],
					camera_calibration[idx].cameraR[3]);

				Eigen::AngleAxis<double> aa(rotation);
				//let avec have axis and angle
				Eigen::Vector3d avec = aa.axis() * aa.angle();

				//store axis angle
				extrinsic[0] = avec[0];
				extrinsic[1] = avec[1];
				extrinsic[2] = avec[2];

				//store translation
				extrinsic[3] = camera_calibration[idx].cameraT[0];
				extrinsic[4] = camera_calibration[idx].cameraT[1];
				extrinsic[5] = camera_calibration[idx].cameraT[2];

				convert_inv_extrinsics(extrinsic, extrinsic);

				//project_cloud_to_image_colored
				project_cloud_to_image_colored(source, label_vec_vec,
					intrinsics, distortion, extrinsic, raw_images_color_vec[idx],
					parsing_images_color_vec[idx]);
			}

			//choose the max num class id of each point of the source
			//TODO
			caculate_label(label_vec_vec, label_point_vec[frame_num]);

#ifdef VIEW_IMAGE
			//cv::imshow("pasring", raw_images_color_vec[0]);
			//cv::waitKey(1);

			ShowManyImages("image", 5, raw_images_color_vec[0], raw_images_color_vec[1],
				raw_images_color_vec[2], raw_images_color_vec[3], raw_images_color_vec[4]);
#endif

#ifdef VIEW_POINTCLOUD
			PointRGB temp;
			source_rgb->clear();
			source_rgb->resize(source->points.size());
			for(auto i = 0; i < source->points.size(); i++) {
				auto p = source->points[i];
				temp.x = p.x; 
				temp.y = p.y; 
				temp.z = p.z; 

				auto class_id = 
					static_cast<unsigned char>(label_point_vec[frame_num].points[i].label);
				if (class_id == INVALID_CLASS_ID) continue;

				//TODO for test
				//if (class_id == 1) {
					//temp.r = colorlist[class_id][0];
					//temp.g = colorlist[class_id][1];
					//temp.b = colorlist[class_id][2];
				//}
				//else {
					//temp.r = 255;
					//temp.g = 255;
					//temp.b = 255;
				//}

				temp.r = colorlist[class_id][0];
				temp.g = colorlist[class_id][1];
				temp.b = colorlist[class_id][2];

				source_rgb->points[i] = temp;
			}
			viewer.removePointCloud("cloud");
			viewer.addPointCloud(source_rgb, "cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
				3, "cloud");
			viewer.spinOnce(10);
#endif

			point_it++;
			frame_num++;

			for(auto j = 0; j < image_num; j++) {
				raw_images_it_vec[j]++;
				parsing_images_it_vec[j]++;
				if (parsing_images_it_vec[j] == parsing_images_view_vec[j].end()) {
					have_finished = true;
					break;
				}
			}
		}

		bag.close();

		//append label_point_vec to bag
		bag.open(bag_file, rosbag::bagmode::Append);
		for (auto i = 0; i < label_point_vec.size(); i++) {
		sensor_msgs::PointCloud2 ros_point;
		//convert to ros format
		pcl::toROSMsg(label_point_vec[i], ros_point);
			bag.write(config.label_point_topic, label_point_stamp[i].data, ros_point);
		}

		bag.close();
	}
}

int get_camera_params(const std::string contents, CameraCalibration calibs[]) {
  std::cout << "Parse Camera Calibration..." << std::endl;
  if (contents.empty()) {
    std::cout << "string is empty" << std::endl;
    return -1;
  }
  YAML::Node yn = YAML::LoadFile(contents);
  std::cout << "**************ParseCameraCalibration contents: \n" << contents << std::endl;
  std::string cameraId;
  for (int id = 0; id < 5; ++id) {
    // cameraId = std::to_string(i);
    cameraId = boost::lexical_cast<std::string>(id);
    cv::Mat intrinsicK, intrinsicD;
    // get intrinsicK
    if (yn[cameraId]["K"].IsDefined()) {
      intrinsicK = cv::Mat::zeros(3, 3, CV_64FC1);
      for (int i = 0; i < yn[cameraId]["K"].size(); ++i) {
        intrinsicK.at<double>(i) = yn[cameraId]["K"][i].as<double>();
      }
      calibs[id].cameraK = intrinsicK;
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // GET intrinsicD
    if (yn[cameraId]["D"].IsDefined()) {
      // std::cout<<"type: " << yn[cameraId]["D"].Type()<<std::endl;
      intrinsicD = cv::Mat::zeros(yn[cameraId]["D"].size(), 1, CV_64FC1);
      for (int i = 0; i < yn[cameraId]["D"].size(); ++i) {
        intrinsicD.at<double>(i) = yn[cameraId]["D"][i].as<double>();
      }
      calibs[id].cameraD = intrinsicD;
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // get camera
    if (yn[cameraId]["Extrinsic"]["rotation"]["x"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["rotation"]["y"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["rotation"]["z"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["rotation"]["w"].IsDefined()) {
      calibs[id].cameraR.push_back(
          yn[cameraId]["Extrinsic"]["rotation"]["w"].as<double>());
      calibs[id].cameraR.push_back(
          yn[cameraId]["Extrinsic"]["rotation"]["x"].as<double>());
      calibs[id].cameraR.push_back(
          yn[cameraId]["Extrinsic"]["rotation"]["y"].as<double>());
      calibs[id].cameraR.push_back(
          yn[cameraId]["Extrinsic"]["rotation"]["z"].as<double>());
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // get cameraR
    if (yn[cameraId]["Extrinsic"]["translation"]["x"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["translation"]["y"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["translation"]["z"].IsDefined()) {
      calibs[id].cameraT.push_back(
          yn[cameraId]["Extrinsic"]["translation"]["x"].as<double>());
      calibs[id].cameraT.push_back(
          yn[cameraId]["Extrinsic"]["translation"]["y"].as<double>());
      calibs[id].cameraT.push_back(
          yn[cameraId]["Extrinsic"]["translation"]["z"].as<double>());
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
  }
  return 0;
}


void load_config(char * file, Config& config) {
	YAML::Node yaml = YAML::LoadFile(file);

	config.bag_path = yaml["bag_path"].as<std::string>();

	config.raw_point_topic = yaml["raw_point_topic"].as<std::string>();
	config.label_point_topic = yaml["label_point_topic"].as<std::string>();

	for(auto i = 0; i < yaml["raw_image_topic"].size(); i++) {
		config.raw_image_topic_vec.push_back(yaml["raw_image_topic"][i].as<std::string>());
	}

	for(auto i = 0; i < yaml["parsing_image_topic"].size(); i++) {
		config.parsing_image_topic_vec.push_back(yaml["parsing_image_topic"][i].as<std::string>());
	}

	assert(yaml["raw_image_topic"].size() == yaml["parsing_image_topic"].size());
}

  
int main(int argc, char** argv) {
	if (argc < 3) {
		std::cerr << "rosrun plot_image_pasring_results_node config_file camera_params_file\n";
		exit(-1);
	}
	Config config;
	load_config(argv[1], config);
	CameraCalibration camera_calibration[5];
	get_camera_params(std::string(argv[2]), camera_calibration);
	plot_parsing_result(config, camera_calibration);

	return 0;
}
