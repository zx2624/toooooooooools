// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     cmp_cam_lidar_pd.h
// ROLE:     TODO (some explanation)
// CREATED:  2019-01-21 14:43:25
// MODIFIED: 2019-01-29 15:49:33

#ifndef HORIZON_MAPPING_EVALUATION_H_
#define HORIZON_MAPPING_EVALUATION_H_

#include <vector>

#include <pcl/search/kdtree.h>

namespace horizon { 
	namespace mapping { 
		namespace evaluation {
		  //Parsing Label
			enum HobotLabels{
					road                          = 0,
					lane                          = 1,
					stop_lane                     = 2,
					crosswalk_line                = 3,
					traffic_arrow                 = 4, 
					lane_marking                  = 5,
					guide_line                    = 6,
					speed_bump                    = 7,
					traffic_sign                  = 8,
					guide_post                    = 9,
					traffic_light                 = 10,
					pole                          = 11,
					building                      = 12,
					sidewalk                      = 13,
					moving_object                 = 14,
					background                    = 15
			};
    
			template <typename PointCam, typename PointLidar>
			class CmpCamLidarPd {
			 public :
				CmpCamLidarPd(const std::vector<HobotLabels>& label_vec, double cov, 
					int min_line_pt_num, int min_plane_pt_num, double leaf_size, 
					int min_pts_each_voxel):
					label_vec_(label_vec), cov_(cov), min_line_pt_num_(min_line_pt_num), 
					min_plane_pt_num_(min_plane_pt_num), mean_dis_(3, 0), std_dis_(3, 0), 
					leaf_size_(leaf_size), min_pts_each_voxel_(min_pts_each_voxel),
					cam_pd_(new typename pcl::PointCloud<PointCam>),
					lidar_pd_(new typename pcl::PointCloud<PointLidar>){
					//default value
					cov_ = 0.005;
					for (auto i = 0; i < label_vec_.size(); i++) {
						typename pcl::PointCloud<PointCam>::Ptr cam_temp(new 
							typename pcl::PointCloud<PointCam>);
						typename pcl::PointCloud<PointLidar>::Ptr lidar_temp(new
							typename pcl::PointCloud<PointLidar>);
						typename pcl::search::KdTree<PointLidar>::Ptr lidar_tree_temp(new
							typename pcl::search::KdTree<PointLidar>);

						cam_pd_vec_.push_back(cam_temp);
						lidar_pd_vec_.push_back(lidar_temp);
						lidar_kd_tree_vec_.push_back(lidar_tree_temp);
					}
			  }
				bool cacPoint2Line();
			  bool cacPoint2Plane();
			  void setCamPd(typename pcl::PointCloud<PointCam>::Ptr);
			  void setLidarPd(typename pcl::PointCloud<PointLidar>::Ptr);
			  void setCamPdCov(double cov) {cov_ = cov;};
				std::vector<double> getMeanDis() {return mean_dis_;};
				std::vector<double> getStdDis() {return std_dis_;};
				void cacMeanDis() {
					auto cnt = 0;
					for (auto j = 0; j < dis_vec_vec_.size(); j++) {
						for (auto k = 0; k < dis_vec_vec_[j].size(); k++) {
							//if dis_dir_vec_vec_ is not valid
							if (dis_dir_vec_vec_[j][k].norm() < 0.01 ) continue;
							mean_dis_[0] += dis_vec_vec_[j][k] * dis_dir_vec_vec_[j][k][0];
							mean_dis_[1] += dis_vec_vec_[j][k] * dis_dir_vec_vec_[j][k][1];
							mean_dis_[2] += dis_vec_vec_[j][k] * dis_dir_vec_vec_[j][k][2];
							cnt++;
						}
					}
					mean_dis_[0] /= cnt;
					mean_dis_[1] /= cnt;
					mean_dis_[2] /= cnt;
				}
				void cacStdDis() {
					auto cnt = 0;
					for (auto j = 0; j < dis_vec_vec_.size(); j++) {
						for (auto k = 0; k < dis_vec_vec_[j].size(); k++) {
							//if dis_dir_vec_vec_ is not valid
							if (dis_dir_vec_vec_[j][k].norm() < 0.01 ) continue;

							std_dis_[0] += pow(dis_vec_vec_[j][k] * dis_dir_vec_vec_[j][k][0] - mean_dis_[0], 2);
							std_dis_[1] += pow(dis_vec_vec_[j][k] * dis_dir_vec_vec_[j][k][1] - mean_dis_[1], 2);
							std_dis_[2] += pow(dis_vec_vec_[j][k] * dis_dir_vec_vec_[j][k][2] - mean_dis_[2], 2);
							cnt++;
						}
					}
					std_dis_[0] = sqrt(std_dis_[0]) / cnt;
					std_dis_[1] = sqrt(std_dis_[1]) / cnt;
					std_dis_[2] = sqrt(std_dis_[2]) / cnt;
				}

				void resetData() {
					for (auto i = 0; i < label_vec_.size(); i++) { 
						dis_vec_vec_[i].resize(cam_pd_vec_[i]->size(), 0);
						dis_dir_vec_vec_[i].resize(cam_pd_vec_[i]->size(), Eigen::Vector3f::Zero());
					}
				}

				std::vector<typename pcl::PointCloud<PointCam>::Ptr> getCamPdVec () {
					return cam_pd_vec_;
				}
				std::vector<typename pcl::PointCloud<PointLidar>::Ptr> getLidarPdVec () {
					return lidar_pd_vec_;
				}

			 protected :
				bool divideCamPdUsingLabel();
				bool divideLidarPdUsingLabel();
				typename pcl::PointCloud<PointCam>::Ptr cam_pd_;
				typename pcl::PointCloud<PointLidar>::Ptr lidar_pd_;

				std::vector<typename pcl::PointCloud<PointCam>::Ptr> cam_pd_vec_;
				std::vector<typename pcl::PointCloud<PointLidar>::Ptr> lidar_pd_vec_;

				std::vector<typename pcl::search::KdTree<PointLidar>::Ptr>
					lidar_kd_tree_vec_;

				double cov_;
				std::vector<HobotLabels> label_vec_;

				//For Latitude, lontitude, altitude
				std::vector<double> mean_dis_;
				std::vector<double> std_dis_;

				//store each point's dis value in each label
				std::vector<std::vector<double>> dis_vec_vec_;
				//store each point's dis dir in each label
				std::vector<std::vector<Eigen::Vector3f> > dis_dir_vec_vec_;

				int min_line_pt_num_;
				int min_plane_pt_num_;

				double leaf_size_;
				int min_pts_each_voxel_;
			};

		}
	}
}
#include <cmp_cam_lidar_pd/impl/cmp_cam_lidar_pd.hpp> 
#endif
