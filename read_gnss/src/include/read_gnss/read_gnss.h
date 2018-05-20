#ifndef READGNSS_READ_GNSS_
#define READGNSS_READ_GNSS_

#include <string>
#include <rosbag/bag.h>

namespace tools {
	class ReadGNSS {
	 public :
		 ReadGNSS(const std::string& bag_file_path, const std::string& gnss_odom_path, 
							const std::string& gnss_topic_name = "/sensor/gnss/odom") {
			bag_file_path_ = bag_file_path;
			gnss_topic_name_ = gnss_topic_name;
			gnss_odom_path_ = gnss_odom_path;
		 };
		 void SaveOdomToFile();
	 private :
		 rosbag::Bag bag_;
		 std::string bag_file_path_;
		 std::string gnss_topic_name_;
		 std::string gnss_odom_path_;
	};
}
#endif
