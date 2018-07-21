// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     main.cc
// ROLE:     TODO (some explanation)
// CREATED:  2018-07-17 20:14:01
// MODIFIED: 2018-07-18 13:53:22
#include <dirent.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>

#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h> 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

//load all files in a dir
vector<string> load_dir_files(const char *dir)
{
	DIR *dp;
	struct dirent *ep;
	char filename[PATH_MAX];
	vector<string> files;
	dp = opendir(dir);
	if (dp == NULL) {
		cerr << dir << " not exists";
		return files;
	}
	while ((ep = readdir(dp))) {
	  if (ep->d_name[0] == '.') continue;
			sprintf(filename, "%s/%s", dir, ep->d_name);
			files.push_back(string(filename));
		}
	closedir(dp);
	sort(files.begin(), files.end());
	cout << dir << " size is " << files.size() << "\n";
	return files;
}

vector<double> load_times(char *file_name) {
	vector<double> times;
	ifstream ifs(file_name);
	double time = 0;
	while (ifs >> time) {
		times.push_back(time + 10e-6);
	}
	cout << file_name << " size is " << times.size() << "\n";
	return times;
}

int main(int argc, char **argv) {
	if(argc < 4) {
		cout << "usage:\n"
			<< "rosrun convert_kitti_lidar_data_to_rosbag " 
			<< "convert_kitti_lidar_data_to_rosbag lidar_data_dir times_file "
			<< "output_bag_name\n";
			return -1;
	}


	//load lidar bin data
	auto lidar_bin_vec = load_dir_files(argv[1]);

	if (lidar_bin_vec.size() <= 0) {
		cerr << "load_dir_files: " << argv[1] << "failed!\n";
		return -1; 
	}
	//load lidar timestamp
	auto lidar_times_vec = load_times(argv[2]);
	if (lidar_times_vec.size() <= 0) {
		cerr << "load_times: " << argv[1] << "failed!\n";
		return -1; 
	}

	//define bag file
	rosbag::Bag bag;
	bag.open(argv[3], rosbag::bagmode::Write);
	if (lidar_bin_vec.size() != lidar_times_vec.size()) {
		cerr << "lidar data size != lidar stamp size\n";
		return -1;
	}

	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	size_t num = 1000000;
	float *data = (float*)malloc(num*sizeof(float));
	for (auto i = 0; i < lidar_bin_vec.size() ; i++) {
		sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd(new pcl::PointCloud<pcl::PointXYZI>);

		num = 1000000;
		// pointers
		float *px = data+0;
		float *py = data+1;
		float *pz = data+2;
		float *pr = data+3;
		//load lidar bin
		FILE *stream;
		stream = fopen (lidar_bin_vec[i].c_str(),"rb");
		num = fread(data, sizeof(float), num, stream)/4;
		for (auto j = 0; j < num; j++) {
			pcl::PointXYZI p;
			p.x = *px;
			p.y = *py;
			p.z = *pz;
			p.intensity = *pr;
			cloud_pcd->push_back(p);
			px+=4; py+=4; pz+=4; pr+=4;
		}
		fclose(stream);

		cout << "frame " << i << " size: "  
			<< cloud_pcd->points.size() << "\n";

		//convert to ros ppointcloud
		pcl::toROSMsg(*cloud_pcd, *cloud_msg);

		cloud_msg->header.stamp.fromSec(lidar_times_vec[i]);
		bag.write("/multi_scan_points", cloud_msg->header.stamp, *cloud_msg);
	}
	delete data;

	bag.close();
	
	return 0;
}
