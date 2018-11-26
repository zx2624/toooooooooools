// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     lidar_match.cc
// ROLE:     TODO (some explanation)
// CREATED:  2018-11-14 11:43:21
// MODIFIED: 2018-11-20 17:59:57
#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

using namespace std;
using namespace rosbag;

typedef pcl::PointXYZI PointT;

//for HDL32 
const float angular_resolution_horizon = 0.17;
const float angular_resolution_vertical = 1.3;
const float horizon_fov = 360.0;
const float vertical_fov = 40.0;

const float support_size = 0.2;

void compareKeyPoints(const string& bag_path, const string& points_topic) {
	Bag bag;
	bag.open(bag_path, bagmode::Read);
	
	cout << "Open " << bag_path << " success!\n";

	rosbag::View points_view(bag, rosbag::TopicQuery(points_topic));

	auto cnt = 0;
	sensor_msgs::PointCloud2ConstPtr ros_pcd;
	pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);

	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;

	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);
	viewer.initCameraParameters();
	viewer.setCameraPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	pcl::visualization::RangeImageVisualizer range_image_widget ("Range_image");

	for (auto ite = points_view.begin(); ite != points_view.end(); ite++) {
		cnt++;
		ros_pcd = ite->instantiate<sensor_msgs::PointCloud2>();
		pcl::fromROSMsg(*ros_pcd, *source);
		cout << "\nFrame " << cnt << " start!\n";
		cout << "PointCloud Size: " << source->points.size() << "\n";

		//create range image from point cloud
		range_image.createFromPointCloud(*source,
				pcl::deg2rad(angular_resolution_horizon), 
				pcl::deg2rad(angular_resolution_vertical),
				pcl::deg2rad(horizon_fov), 
				pcl::deg2rad(vertical_fov));

		cout << "range image Size: " << range_image.size() << "\n";
		cout << "angular resolution: "
		<< pcl::rad2deg (range_image.getAngularResolutionX ()) << "deg/pixel in x and "
		<< pcl::rad2deg (range_image.getAngularResolutionY ()) << "deg/pixel in y.\n" << std::endl;
		cout << "range image width: " << range_image.width << std::endl;
		cout << "range image height: " << range_image.height << std::endl;

		//show range image in viewer
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> 
			range_image_color_handler(range_image_ptr, 0, 0, 0);

		viewer.removePointCloud("range image");
		viewer.addPointCloud(range_image_ptr, range_image_color_handler,
			 "range image");
		viewer.setPointCloudRenderingProperties(
			 pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "range image");

		range_image_widget.showRangeImage (range_image);

		//extract narf keypoints
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector;
		narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage (&range_image);
		narf_keypoint_detector.getParameters ().support_size = support_size;

		pcl::PointCloud<int> keypoint_indices;
		narf_keypoint_detector.compute (keypoint_indices);

		std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

		//show keypoints in viewer
		pcl::PointCloud<pcl::PointXYZ>::Ptr 
			keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;

		keypoints.points.resize (keypoint_indices.points.size ());
		for (size_t i=0; i<keypoint_indices.points.size (); ++i) {
			keypoints.points[i].getVector3fMap () =
				range_image.points[keypoint_indices.points[i]].getVector3fMap ();
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
			keypoints_color_handler(keypoints_ptr, 0, 255, 0);

		viewer.removePointCloud("keypoints");
		viewer.addPointCloud<pcl::PointXYZ> 
			(keypoints_ptr, keypoints_color_handler, "keypoints");
		viewer.setPointCloudRenderingProperties 
			(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

		viewer.spinOnce(10);
		//while (1) {
		//range_image_widget.spinOnce(); 
		//}
	}
}

void sift_fpfh_ransac(const string& bag_path, const string& points_topic) {
	// Parameters for sift computation
	const float min_scale = 0.1f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 10;
	const float min_contrast = 0.5f;
	const int descriptor_size = 33;

	Bag bag;
	bag.open(bag_path, bagmode::Read);
	
	cout << "Open " << bag_path << " success!\n";

	rosbag::View points_view(bag, rosbag::TopicQuery(points_topic));

	auto cnt = 0;
	sensor_msgs::PointCloud2ConstPtr ros_pcd;
	pcl::PointCloud<PointT>::Ptr raw_source(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr last_source(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_source_color(new pcl::PointCloud<pcl::PointXYZRGB>);

#ifdef VIEWER
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);
	viewer.initCameraParameters();
	viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);
#endif

	/*  define sift object */
	pcl::SIFTKeypoint<PointT, PointT> sift;
	pcl::PointCloud<PointT>::Ptr last_keypoints(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_keypoints_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	pcl::search::KdTree<PointT>::Ptr tree_2(new pcl::search::KdTree<PointT>);
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);

	//define cloud normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the FPFH descriptors for each point.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr last_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());

	pcl::NormalEstimationOMP<PointT, pcl::Normal> normals_estimation;
	normals_estimation.setSearchMethod(tree);
	//TODO may be to adjusment the search number
	normals_estimation.setKSearch(5);

	//// FPFH estimation object.
	//pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
	//fpfh.setSearchMethod(tree);
	//// Search radius, to look for neighbors. Note: the value given here has to be
	//// larger than the number used to estimate the normals.
	//fpfh.setKSearch(15);

	// Object for pose estimation.
	pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> pose;
	// Instead of matching a descriptor with its nearest neighbor, choose randomly between
	// the N closest ones, making it more robust to outliers, but increasing time.
	pose.setCorrespondenceRandomness(2);
	// Set the fraction (0-1) of inlier points required for accepting a transformation.
	// At least this number of points will need to be aligned to accept a pose.
	pose.setInlierFraction(0.25f);
	// Set the number of samples to use during each iteration (minimum for 6 DoF is 3).
	pose.setNumberOfSamples(3);
	// Set the similarity threshold (0-1) between edge lengths of the polygons. The
	// closer to 1, the more strict the rejector will be, probably discarding acceptable poses.
	pose.setSimilarityThreshold(0.6f);
	// Set the maximum distance threshold between two correspondent points in source and target.
	// If the distance is larger, the points will be ignored in the alignment process.
	pose.setMaxCorrespondenceDistance(5.0);
	pose.setMaximumIterations(50);
	pose.setTransformationEpsilon(0.01);
	pose.setEuclideanFitnessEpsilon(0.1);

	Eigen::Matrix4f tf_start_to_now = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tf_last_to_now = Eigen::Matrix4f::Identity();

	for (auto ite = points_view.begin(); ite != points_view.end(); ite++) {
		cnt++;
		ros_pcd = ite->instantiate<sensor_msgs::PointCloud2>();
		pcl::fromROSMsg(*ros_pcd, *source);

		long num = 0;
		vector<int> indices;
		num = source->size();
		//remove NaN point
		pcl::removeNaNFromPointCloud(*source, *source, indices);

		cout << "\nFrame " << cnt << " start!\n";
		cout << "PointCloud Size: " << source->points.size() << "\n";
		cout << "NaN point Size: " << num - source->size() << "\n\n";

		/*  extract sift keypoints  */
		sift.setInputCloud(source);
		auto start = clock();
		sift.compute(*keypoints);
		auto end = clock();

		num = keypoints->size();
		//remove NaN point
		pcl::removeNaNFromPointCloud(*keypoints, *keypoints, indices);

		cout << "keypoints Size: " << keypoints->points.size() << "\n";
		cout << "extract keypoints time: " << (end - start) * 1000 / CLOCKS_PER_SEC << "ms\n";
		cout << "NaN keypoint Size: " << num - keypoints->size() << "\n\n";

		/* caculate fpfh descriptors */
		
		//estimate normal, must compute all points normal in source
		normals_estimation.setInputCloud(source);
		start = clock();
		normals_estimation.compute(*normals);
		end = clock();

		num = normals->size();
		//remove nan from normals
		pcl::removeNaNNormalsFromPointCloud(*normals, *normals, indices);

		cout << "normals Size: " << normals->points.size() << "\n";
		cout << "normals estimation time: " << (end - start) * 1000 / CLOCKS_PER_SEC << "ms\n";
		cout << "NaN normals Size: " << num - normals->size() << "\n\n";

#if 1
		//TODO START
		// FPFH estimation object.
		pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setSearchMethod(tree_2);
		// Search radius, to look for neighbors. Note: the value given here has to be
		// larger than the number used to estimate the normals.
		fpfh.setKSearch(6);
		//TODO END

		//estimate descriptors
		//fpfh.setInputCloud(keypoints);
		fpfh.setInputCloud(source);
		//must set
		//fpfh.setSearchSurface(source);
		//In case of search surface is set to be different from the input cloud,
		//normals should correspond to the search surface, not the input cloud!
		fpfh.setInputNormals(normals);

		start = clock();
		fpfh.compute(*descriptors);
		end = clock();

		cout << "descriptors Size: " << descriptors->points.size() << "\n";
		cout << "descriptors computing time: " << (end - start) * 1000 / CLOCKS_PER_SEC << "ms\n";

		//remove NaN from descriptors and keypoints 
		//vector<pcl::FPFHSignature33, Eigen::aligned_allocator<pcl::FPFHSignature33> >::iterator
			//descriptorIt = descriptors->points.begin(); 

		//vector<PointT, Eigen::aligned_allocator<PointT> >::iterator
			//keypointIt = keypoints->points.begin(); 

		//for (int j, i = 0, k = descriptors->size(); i < k; ++i) { 
			//for (j = 0; j < descriptor_size; ++j) {
				//if (isnan(descriptors->points[i].histogram[j])) break; 
			//}
			//if (j != descriptor_size) {
				//descriptors->erase(descriptorIt + i);
				//keypoints->erase(keypointIt + i);
				//--i;
				//--k;
				//cout << "Delete!\n";
			//} 
		//}


		if (cnt > 1) { 
			cout << "Start Ransac!\n";
			/* ransac */
			//pose.setInputTarget(last_keypoints);
			//pose.setInputSource(keypoints);
			pose.setInputTarget(last_source);
			pose.setInputSource(source);

			pose.setTargetFeatures(last_descriptors);
			pose.setSourceFeatures(descriptors);

			pose.align(*aligned);
		
			cout << "| has_converged | fitness_score | \n";
			cout << pose.hasConverged() << " | "
								<< pose.getFitnessScore() << "\n";
			
			tf_last_to_now = pose.getFinalTransformation();
			tf_start_to_now = tf_start_to_now * tf_last_to_now;

			cout << "tf(last_to_now):\n" << tf_last_to_now << "\n";	
			cout << "tf(start_to_now):\n" << tf_start_to_now << "\n\n";	

#ifdef VIEWER
			//set rgb cloud
			last_source_color->clear();
			for (auto i = 0; i < last_source->points.size(); i++) {
				auto p = last_source->points[i];
				pcl::PointXYZRGB pc;
				pc.x = p.x;
				pc.y = p.y;
				pc.z = p.z;
				pc.r = 255;
				pc.g = 0;
				pc.b = 0;
				last_source_color->push_back(pc);
			}

			source_color->clear();
			Eigen::Vector4f temp;
			for (auto i = 0; i < source->points.size(); i++) {
				auto p = source->points[i];
				temp << p.x, p.y, p.z, 1;
				temp = tf_last_to_now * temp;
				pcl::PointXYZRGB pc;
				pc.x = temp(0);
				pc.y = temp(1);
				pc.z = temp(2);
				pc.r = 0;
				pc.g = 255;
				pc.b = 0;
				source_color->push_back(pc);
			}

			last_keypoints_color->clear();
			for (auto i = 0; i < last_keypoints->points.size(); i++) {
				auto p = last_keypoints->points[i];
				pcl::PointXYZRGB pc;
				pc.x = p.x;
				pc.y = p.y;
				pc.z = p.z;
				pc.r = 0;
				pc.g = 255;
				pc.b = 0;
				last_keypoints_color->push_back(pc);
			}

			keypoints_color->clear();
			for (auto i = 0; i < keypoints->points.size(); i++) {
				auto p = keypoints->points[i];
				temp << p.x, p.y, p.z, 1;
				temp = tf_last_to_now * temp;
				pcl::PointXYZRGB pc;
				pc.x = temp(0);
				pc.y = temp(1);
				pc.z = temp(2);
				pc.r = 0;
				pc.g = 255;
				pc.b = 0;
				keypoints_color->push_back(pc);
			}

			// Visualization of keypoints along with the original cloud
			viewer.removePointCloud("last_cloud");
			viewer.addPointCloud(last_source_color, "last_cloud");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
					1, "last_cloud");

			viewer.removePointCloud("cloud");
			viewer.addPointCloud(source_color, "cloud");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
					1, "cloud");

			//viewer.removePointCloud("last_keypoints");
			//viewer.addPointCloud(last_keypoints_color, "last_keypoints");
			//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
					//7, "last_keypoints");

			//viewer.removePointCloud("keypoints");
			//viewer.addPointCloud(keypoints_color, "keypoints");
			//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
					//7, "keypoints");
			
			viewer.spinOnce (10);
#endif
		}

		//set source to last_source
		*last_source = *source;

		//set keypoints tp last_keypoints
		*last_keypoints = *keypoints;

		//set descriptors tp last_descriptors
		*last_descriptors = *descriptors;

#ifdef VIEWER
		//*last_source_color = *source_color;
		//*last_keypoints_color = *keypoints_color;
#endif

#endif

	}
}

void usage() {
	cerr << "rosrun lidar_match lidar_match_node bag_file\n";
}

int main(int argc, char** argv) {
	if (argc < 2) {
		usage();
		return -1;
	}
	string points_topic = "/sensor/velodyne/points";
	//string points_topic = "/velodyne_points";

	//compareKeyPoints(argv[1], points_topic);
	sift_fpfh_ransac(argv[1], points_topic);

	return 0;
}
