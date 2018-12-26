//Author: siyuan.yu

#include <ctime>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/project_inliers.h>

//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/impl/extract_clusters.hpp>

#include  <road_object/ground_estimation.h>

//BeiJing map base
const double X_ST = 439311;
const double Y_ST = 4426691;
const double Z_ST = 0;
const double EPS = 0.01;
const double resolution = 0.3;
const int min_points_per_leaf = 2;
const float tolerance = 1;
const int min_clustering_points_num = 2;
const int max_clustering_points_num = 10000;

const double min_len = 0.5;
const double max_len = 6;


float norm_a = -0.01371733;
float norm_b = 0.01460536;
float norm_c = 0.9997992999999999;
float norm_d = 1.988692;


struct AABB {
	AABB () {
		x_min = std::numeric_limits<double>::max();
		y_min = std::numeric_limits<double>::max();
		z_min = std::numeric_limits<double>::max();

		x_max = -std::numeric_limits<double>::max();
		y_max = -std::numeric_limits<double>::max();
		z_max = -std::numeric_limits<double>::max();
	}
	double getLength() const {
		return sqrt(pow(x_max - x_min, 2) + pow(y_max - y_min, 2) + pow(z_max - z_min, 2));
	}
	friend std::ostream& operator<<(std::ostream& out, AABB& aabb) {
		out << "[x_min, x_max, y_min, y_max, z_min, z_max]:\n"
			<< aabb.x_min << " " << aabb.x_max << " "
			<< aabb.y_min << " " << aabb.y_max << " "
			<< aabb.z_min << " " << aabb.z_max << "\n";
	}

	double x_min;
	double y_min;
	double z_min;

	double x_max;
	double y_max;
	double z_max;
};

void select_points(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
									pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected) {
	source_selected->clear();
	for(unsigned int i = 0; i < source->points.size(); i++) {
		pcl::PointXYZI p = source->points[i];
		 if ((fabs(p.x) < 2 && fabs(p.y) < 3 && fabs(p.z) < 2)) {
			 //remove the lidar
			continue;
		 }

		 if (fabs(p.x) > 30 || fabs(p.y) > 30) {
			 //remove too far points
			continue;
		 }
		 source_selected->points.push_back(p);
	}
}

//namespace pcl {
	//template <typename PointT>
		//class MyEuclideanClusterExtraction : public EuclideanClusterExtraction<PointT> {
		//protected:
			 //// Members derived from the base class
			//using BasePCLBase::input_;
			//using BasePCLBase::indices_;
			//using BasePCLBase::initCompute;
			//using BasePCLBase::deinitCompute;
		
			//using EuclideanClusterExtraction<PointT>::tree_;
		
			//using EuclideanClusterExtraction<PointT>::cluster_tolerance_;
									
			//using EuclideanClusterExtraction<PointT>::min_pts_per_cluster_;
									
			//using EuclideanClusterExtraction<PointT>::max_pts_per_cluster_;
		//};
//}

inline Eigen::Matrix4d getMatrix4dFromMsg(const nav_msgs::Odometry& msg) {
	Eigen::Quaterniond q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
	msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
	Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
	pose.block<3, 3>(0, 0) = q.toRotationMatrix();
	pose(0, 3) = msg.pose.pose.position.x;
	pose(1, 3) = msg.pose.pose.position.y;
	pose(2, 3) = msg.pose.pose.position.z;
	return pose;
}

template <typename PointT> 
void my_extractEuclideanClusters (const pcl::PointCloud<PointT> &cloud, 
															 const std::vector<int> &indices,
															 //const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
															 const typename pcl::search::KdTree<PointT>::Ptr tree,
															 float tolerance, std::vector<pcl::PointIndices> &clusters,
															 unsigned int min_pts_per_cluster, 
															 unsigned int max_pts_per_cluster)
{
	// \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
	//and indices[i]
	if (tree->getInputCloud ()->points.size () != cloud.points.size ())
	{
		PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n",
			tree->getInputCloud ()->points.size (), cloud.points.size ());
		return;
	}
	// Check if the tree is sorted -- if it is we don't need to check the first element
	int nn_start_idx = tree->getSortedResults () ? 1 : 0;

	// Create a bool vector of processed point indices, and initialize it to false
	std::vector<bool> processed (cloud.points.size (), false);

	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	// Process all points in the indices vector
	for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
	{
		if (processed[indices[i]])
			continue;

		std::vector<int> seed_queue;
		int sq_idx = 0;
		seed_queue.push_back (indices[i]);

		processed[indices[i]] = true;

		while (sq_idx < static_cast<int> (seed_queue.size ()))
		{
			// Search for sq_idx
			int ret = tree->radiusSearch (cloud.points[seed_queue[sq_idx]], tolerance, nn_indices, nn_distances);
			if( ret == -1)
			{
				PCL_ERROR("[pcl::extractEuclideanClusters] Received error code -1 from radiusSearch\n");
				exit(0);
			}
			if (!ret)
			{
				sq_idx++;
				continue;
			}

			for (size_t j = nn_start_idx; j < nn_indices.size (); ++j)             // can't assume sorted (default isn't!)
			{
				if (nn_indices[j] == -1 || processed[nn_indices[j]])        // Has this point been processed before ?
					continue;

				// Perform a simple Euclidean clustering
				seed_queue.push_back (nn_indices[j]);
				processed[nn_indices[j]] = true;
			}

			sq_idx++;
		}

		// If this queue is satisfactory, add to the clusters
		if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
		{
			pcl::PointIndices r;
			r.indices.resize (seed_queue.size ());
			for (size_t j = 0; j < seed_queue.size (); ++j)
				// This is the only place where indices come into play
				r.indices[j] = seed_queue[j];

			// These two lines should not be needed: (can anyone confirm?) -FF
			//r.indices.assign(seed_queue.begin(), seed_queue.end());
			std::sort (r.indices.begin (), r.indices.end ());
			r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

			r.header = cloud.header;
			clusters.push_back (r);   // We could avoid a copy by working directly in the vector
		}
	}
}

bool cmp (const pcl::PointIndices& a, const pcl::PointIndices& b) {
	return a.indices.size() > b.indices.size();
}

int main(int argc, char** argv) {
	if (argc < 3) {
		std::cerr << "Usgae: rosrun remove_dynamic_obstacle remove_dynamic_obstacle "
			<< "bag_file pointcloud_topic_name odom_topic_name \n";
		return -1;
	}
		rosbag::Bag bag;

		std::cout << "Open bag file " << argv[1] << std::endl;
		bag.open(argv[1], rosbag::bagmode::Read);
		std::cout << "open bag file success! \n";

		std::cout << "Read topic " << argv[2] << std::endl;
		rosbag::View points_view(bag, rosbag::TopicQuery(argv[2]));
		rosbag::View odom_view(bag, rosbag::TopicQuery(argv[3]));

		rosbag::View::iterator odom_iter = odom_view.begin();
		
		rosbag::View::iterator pti = points_view.begin();   


		bool is_first_frame = true;

		sensor_msgs::PointCloud2ConstPtr ros_pcd; 
		pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr source_selected(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		int64_t count = 0;

		clock_t start = clock(), end = clock();
		//statistic time 
		std::vector<double> time_count;

		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addCoordinateSystem(3.0 ,"coor");
		viewer.setBackgroundColor(0.0, 0.0, 0.0, 0.0);
		viewer.initCameraParameters();
		viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

		pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree (resolution);

		//define Ground Estimation 
		GroundEstimation ground_estimation;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

		pcl::PointCloud<pcl::PointXYZI>::Ptr non_road_pcl(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr non_road_pcl_temp(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);

		//set init norm using calibration results
		Eigen::Vector3f normals(norm_a, norm_b, norm_b);
		float lidar_height = norm_d;

		//EuclideanClusterExtraction 
		//pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZI>);

		pcl::ProjectInliers<pcl::PointXYZI> proj;

		while(odom_iter != odom_view.end() && pti != points_view.end()) { 
			nav_msgs::OdometryConstPtr od = odom_iter->instantiate<nav_msgs::Odometry>();
			sensor_msgs::PointCloud2ConstPtr pt = pti->instantiate<sensor_msgs::PointCloud2>();

			if(od->header.stamp.toSec() - pt->header.stamp.toSec() > EPS) {
				pti++;
				continue;
			}
			else if(pt->header.stamp.toSec() - od->header.stamp.toSec() > EPS) {
				odom_iter++;
				continue;
			}

			//get pointcloud
			pcl::fromROSMsg(*pt, *source);
			count++;

			//std::cout << "Before Select: " << source->points.size() << std::endl;
			//delete points on car
			select_points(source, source_selected);
			//std::cout << "After Select: " << source_selected->points.size() << std::endl;

			//delete ground plane points
			
			if (source_selected->points.size() > 0) {
				ground_estimation.setLidarHeight(lidar_height);
				ground_estimation.setNormal(normals);
				ground_estimation.setInputCloud(source_selected);

				ground_estimation.filter(); 

				//get result 
				ground_estimation.getNonGroundCloud(*non_road_pcl); 
				ground_estimation.getCoefficients(*coefficients);

				normals = Eigen::Vector3f(coefficients->values[0], coefficients->values[1],
					coefficients->values[2]);
				lidar_height = coefficients->values[3];
				//std::cout << "Plane coeff: \n" 
					//<< coefficients->values[0] << " "
					//<< coefficients->values[1] << " "
					//<< coefficients->values[2] << " "
					//<< coefficients->values[3]
					//<< std::endl; 
			}

			//may be the ground plane is error
			if (non_road_pcl->points.size() <= 0) {
				normals = Eigen::Vector3f(norm_a, norm_b, norm_c);
				lidar_height = norm_d;
			}

			//get pose
			auto mat = getMatrix4dFromMsg(*od);
			mat(0, 3) = mat(0, 3) - X_ST;
			mat(1, 3) = mat(1, 3) - Y_ST;
			mat(2, 3) = mat(2, 3) - Z_ST;

			start = clock();

			//std::cout << "translation: \n" 
				//<< mat(0, 3) << " "
				//<< mat(1, 3) << " "
				//<< mat(2, 3) << std::endl;

#ifdef USE_2D
			//project points into ground plane
			proj.setModelType (pcl::SACMODEL_PLANE);
			proj.setInputCloud (non_road_pcl);
			proj.setModelCoefficients (coefficients);
			proj.filter (*cloud_projected);

			//backup non_road_pcl
			*non_road_pcl_temp = *cloud_projected;

			//TODO for test
			*non_road_pcl = *cloud_projected;
#else
			//backup non_road_pcl
			*non_road_pcl_temp = *non_road_pcl;
#endif

			//transform pointcloud to world coordinate
			pcl::transformPointCloud (*non_road_pcl, *non_road_pcl, mat);

			// assign point cloud to octree
			octree.setInputCloud (non_road_pcl);

			std::cout << "Input points size: " << non_road_pcl->size() << std::endl;

			// add points from cloud to octree
			octree.addPointsFromInputCloud ();

			std::cout << "Leaf cnt: " << octree.getLeafCount() << std::endl;

			boost::shared_ptr<std::vector<int> > newPointIdxVector (new std::vector<int>);

			// get a vector of new points, which did not exist in previous buffer
			octree.getPointIndicesFromNewVoxels (*newPointIdxVector, min_points_per_leaf);

			std::cout << "Diffrent points size: " << newPointIdxVector->size() << std::endl;

			//Get more points ny clustering the newPointIdxVector's points
			cluster_indices.clear();
			//ec.setSearchMethod (octree);
			//ec.setInputCloud (non_road_pcl);
			//ec.extract (cluster_indices);
			kd_tree->setInputCloud (non_road_pcl);
			my_extractEuclideanClusters(*non_road_pcl, *newPointIdxVector, kd_tree,
				tolerance, cluster_indices, min_clustering_points_num, max_clustering_points_num);

			//sort the cluster_indices based the indices size
			std::sort (cluster_indices.begin(), cluster_indices.end(), cmp);

			// switch buffers - reset tree
			octree.switchBuffers ();

			filtered_cloud->clear();
			filtered_cloud->points.resize(non_road_pcl_temp->points.size());

			for(size_t i = 0; i < non_road_pcl_temp->points.size(); i++) {
				pcl::PointXYZI temp = non_road_pcl_temp->points[i];

				pcl::PointXYZRGB p1;
				p1.x = temp.x;
				p1.y = temp.y;
				p1.z = temp.z;

				p1.r = 0;
				p1.g = 255;
				p1.b = 0;

				filtered_cloud->points[i] = p1;
			}

			int cnt = 0;
			std::vector<AABB> vec_aabb(cluster_indices.size());

			viewer.removeAllShapes();
			for (auto it = cluster_indices.begin ();
				it != cluster_indices.end ();
				++it, cnt++) {
					std::cout << "cluster " << cnt << " points size: " << it->indices.size()
						<< std::endl;
				//delete cluster who is not valid obstacle 
				AABB aabb;

				for (auto itt = it->indices.begin (); itt != it->indices.end (); ++itt) {
					auto point = filtered_cloud->points[*itt];
					if (point.x < aabb.x_min) aabb.x_min = point.x;
					if (point.x > aabb.x_max) aabb.x_max = point.x;

					if (point.y < aabb.y_min) aabb.y_min = point.y;
					if (point.y > aabb.y_max) aabb.y_max = point.y;

					if (point.z < aabb.z_min) aabb.z_min = point.z;
					if (point.z > aabb.z_max) aabb.z_max = point.z;
				}
				double len = aabb.getLength();
				vec_aabb[cnt] = aabb;
				std::cout << aabb;
				std::cout << "len : " << len << std::endl;

				if (len > max_len || len < min_len) continue; 
				//add cude to viewer
				viewer.addCube(aabb.x_min, aabb.x_max, aabb.y_min, aabb.y_max,
					aabb.z_min, aabb.z_max, 1.0, 1.0, 1.0, std::to_string(cnt));

				for (auto itt = it->indices.begin (); itt != it->indices.end (); ++itt) {
					filtered_cloud->points[*itt].r = 255;
					filtered_cloud->points[*itt].g = 0;
					filtered_cloud->points[*itt].b = 0;
				}
			}

			for (std::vector<int>::iterator it = newPointIdxVector->begin (); 
				it != newPointIdxVector->end (); ++it) {
				filtered_cloud->points[*it].r = 0;
				filtered_cloud->points[*it].g = 0;
				filtered_cloud->points[*it].b = 255;
			}
				

			end = clock();

			time_count.push_back((end - start) * 1000/CLOCKS_PER_SEC);//get millisecond


			viewer.removePointCloud("cloud");
			viewer.addPointCloud(filtered_cloud, "cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
				3, "cloud");

			viewer.updateText(std::to_string(newPointIdxVector->size()), 10, 10, 3, 255, 0, 0, "text");
				
			viewer.spinOnce(10);

			std::cout << "\nFrame: " << count << std::endl;
			std::cout << "time: " << (end - start) * 1000/CLOCKS_PER_SEC << 
				"ms" << std::endl;
			pti++;
			odom_iter++;

		}

		double sum_time = 0.0;
		for(unsigned int i = 0;i < time_count.size(); i++) {
			sum_time += time_count[i];
		}
		std::sort(time_count.begin(), time_count.end());
		std::cout << "average_time: " << sum_time / time_count.size() << "ms" << std::endl;
		std::cout << "middle_time: " << time_count[time_count.size()/2] << "ms" << std::endl;

		bag.close();

		return 0;
	}
