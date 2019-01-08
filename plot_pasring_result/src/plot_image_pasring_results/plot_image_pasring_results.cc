// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     plot_image_pasring_results.cc
// ROLE:     TODO (some explanation)
// CREATED:  2019-01-07 17:20:44
// MODIFIED: 2019-01-07 20:54:32
#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/opencv.hpp>

const float MASK_VALUE = 0.5;
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
  
int main(int argc, char** argv) {
	if (argc < 4) {
		std::cerr << "Usage: rosrun plot_image_pasring_results "
							<< " plot_image_pasring_results_node bag_file raw_image_topic parsing_image_topic\n";
		exit(-1);
	}

	rosbag::Bag bag;
	bag.open(argv[1], rosbag::bagmode::Read);

	rosbag::View raw_images_view(bag, rosbag::TopicQuery(argv[2]));
	rosbag::View parsing_images_view(bag, rosbag::TopicQuery(argv[3]));

	auto raw_it = raw_images_view.begin();
	auto parsing_it = parsing_images_view.begin();

	//for test
	for (auto i = 0; i < 3000; i++) {
		raw_it++;
		parsing_it++;
	}


	//for(auto it = images_view.begin(); it != images_view.end(); it++) {
	while (raw_it != raw_images_view.end() && parsing_it != parsing_images_view.end()) {
		auto raw_image_ptr = raw_it->instantiate<sensor_msgs::CompressedImage>();
		auto parsing_image_ptr = parsing_it->instantiate<sensor_msgs::CompressedImage>();

		cv::Mat raw_image_color = cv::imdecode(cv::Mat(raw_image_ptr->data), 1);
		cv::Mat parsing_image_color = cv::imdecode(cv::Mat(parsing_image_ptr->data), 1);
		
		for (auto i = 0 ; i < raw_image_color.rows; i++) {
			for (auto j = 0 ; j < raw_image_color.cols; j++) {
				auto id = parsing_image_color.at<cv::Vec3b>(i, j)[0];
				auto color = colorlist[id];
				auto raw_color = raw_image_color.at<cv::Vec3b>(i, j);

				for (auto k = 0; k < raw_image_color.channels(); k++) {
					raw_color[k] =
						static_cast<uchar>(std::min(static_cast<uchar>(color[k] * MASK_VALUE + 
							raw_color[k] * (1 - MASK_VALUE)), uchar(255)));
				}
				raw_image_color.at<cv::Vec3b>(i, j) = raw_color;
			}
		}
		//cv::Mat channels[3];
		//cv::split(image_color, channels);
		//cv::imshow("pasring",channels[0] * 10);
		cv::imshow("pasring", raw_image_color);
		cv::waitKey(1);
		raw_it++;
		parsing_it++;
	}

	return 0;
}
