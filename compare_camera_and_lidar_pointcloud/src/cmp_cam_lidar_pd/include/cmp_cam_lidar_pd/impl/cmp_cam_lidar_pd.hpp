// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     cmp_cam_lidar_pd.hpp
// ROLE:     TODO (some explanation)
// CREATED:  2019-01-21 19:31:05
// MODIFIED: 2019-01-24 15:06:09
#ifndef HORIZON_MAPPING_EVALUATION_HPP_
#include <cmp_cam_lidar_pd/cmp_cam_lidar_pd.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

namespace horizon {
	namespace mapping {
		namespace evaluation {

			template<class PointCam, class PointLidar>
			bool CmpCamLidarPd<PointCam, PointLidar>::cacPoint2Line() {
				if (cam_pd_->size() <= 0) {
					LOG(FATAL) << "cam_pd_ size is 0! You must setCamPd() first.";
					return false;
				}
				if (lidar_pd_->size() <= 0) {
					LOG(FATAL) << "lidar_pd_ size is 0! You must setLidarPd() first.";
					return false;
				}

				resetData();

				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
				// Create the segmentation object
				pcl::SACSegmentation<PointLidar> seg;

				//for each label in label_vec_
				for (auto label_idx = 0; label_idx < label_vec_.size(); label_idx++) {
					pcl::IndicesPtr k_indices(new std::vector<int>(min_line_pt_num_));
					std::vector<float> k_sqr_dis(min_line_pt_num_);

					seg.setInputCloud(lidar_pd_vec_[label_idx]);
					//for each point int cam_pd_vec_[label_idx]
					for (auto cam_idx = 0; cam_idx < cam_pd_vec_[label_idx]->
							points.size(); cam_idx++) {
						//search nearest min_line_pt_num_ point 
						auto cam_pt = cam_pd_vec_[label_idx]->points[cam_idx];
						PointLidar cam_pt_temp;
						cam_pt_temp.x = cam_pt.x;
						cam_pt_temp.y = cam_pt.y;
						cam_pt_temp.z = cam_pt.z;

						auto ret = lidar_kd_tree_vec_[label_idx]->nearestKSearch(cam_pt_temp,
							min_line_pt_num_, *k_indices, k_sqr_dis);
						if (ret <= 0) continue;
							
						seg.setIndices(k_indices);
						//fit line
						seg.setOptimizeCoefficients (true);
						// Mandatory
						//seg.setModelType (pcl::SACMODEL_PLANE);
						seg.setModelType (pcl::SACMODEL_LINE);
						seg.setMethodType (pcl::SAC_RANSAC);
						seg.setDistanceThreshold (0.2);

						seg.segment (*inliers, *coefficients);
						
						Eigen::Vector3f cam_pt_xyz(cam_pt.x, cam_pt.y, cam_pt.z);
						Eigen::Vector3f line_pt(
							coefficients->values[0],
							coefficients->values[1],
							coefficients->values[2]
							);
						Eigen::Vector3f line_dir(
							coefficients->values[3],
							coefficients->values[4],
							coefficients->values[5]
							);

						/*dis = ||line_dir x (line_pt - cam_pt_xyz)|| / ||line_dir|| 
							= norm (cross (p2-p1, p1-p0)) / norm(p2-p1) */
						auto dis = line_dir.cross(line_pt - cam_pt_xyz).norm() / line_dir.norm();

						dis_vec_vec_[label_idx][cam_idx] = dis;
						
						// dir = line_dir x (line_pt - cam_pt_xyz)
						auto dir = line_dir.cross(line_dir.cross(line_pt - cam_pt_xyz));

						dir.normalize();

						//change dir's value to positive
						dir[0] = fabs(dir[0]);
						dir[1] = fabs(dir[1]);
						dir[2] = fabs(dir[2]);
						dis_dir_vec_vec_[label_idx][cam_idx] = dir;
					}
				}
				cacMeanDis();
				cacStdDis();
			}

			template<class PointCam, class PointLidar>
			bool CmpCamLidarPd<PointCam, PointLidar>::cacPoint2Plane() {
				if (cam_pd_->size() <= 0) {
					LOG(FATAL) << "cam_pd_ size is 0! You must setCamPd() first.";
					return false;
				}
				if (lidar_pd_->size() <= 0) {
					LOG(FATAL) << "lidar_pd_ size is 0! You must setLidarPd() first.";
					return false;
				}

				//reset data
				resetData();

				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
				// Create the segmentation object
				pcl::SACSegmentation<PointLidar> seg;

				//for each label in label_vec_
				for (auto label_idx = 0; label_idx < label_vec_.size(); label_idx++) {
					pcl::IndicesPtr k_indices(new std::vector<int>(min_line_pt_num_));
					std::vector<float> k_sqr_dis(min_line_pt_num_);

					seg.setInputCloud(lidar_pd_vec_[label_idx]);
					//for each point int cam_pd_vec_[label_idx]
					for (auto cam_idx = 0; cam_idx < cam_pd_vec_[label_idx]->
							points.size(); cam_idx++) {
						//search nearest min_line_pt_num_ point 
						auto cam_pt = cam_pd_vec_[label_idx]->points[cam_idx];
						PointLidar cam_pt_temp;
						cam_pt_temp.x = cam_pt.x;
						cam_pt_temp.y = cam_pt.y;
						cam_pt_temp.z = cam_pt.z;

						auto ret = lidar_kd_tree_vec_[label_idx]->nearestKSearch(cam_pt_temp,
							min_plane_pt_num_, *k_indices, k_sqr_dis);
						if (ret <= 0) continue;
							
						seg.setIndices(k_indices);
						//fit line
						seg.setOptimizeCoefficients (true);
						// Mandatory
						seg.setModelType (pcl::SACMODEL_PLANE);
						seg.setMethodType (pcl::SAC_RANSAC);
						seg.setDistanceThreshold (0.2);

						seg.segment (*inliers, *coefficients);
						
						Eigen::Vector3f cam_pt_xyz(cam_pt.x, cam_pt.y, cam_pt.z);
						Eigen::Vector3f norm_dir(
							coefficients->values[0],
							coefficients->values[1],
							coefficients->values[2]
							);

						//dis = |cam_pt_xyz * norm_dir| / ||norm_dir|| 
						auto dis = fabs(cam_pt_xyz.dot(norm_dir) + coefficients->values[3]) / norm_dir.norm();

						dis_vec_vec_[label_idx][cam_idx] = dis;
						
						auto dir = norm_dir;
						dir.normalize();

						//change dir's value to positive
						dir[0] = fabs(dir[0]);
						dir[1] = fabs(dir[1]);
						dir[2] = fabs(dir[2]);
						dis_dir_vec_vec_[label_idx][cam_idx] = dir;
					}
				}
				cacMeanDis();
				cacStdDis();
			}

			template<class PointCam, class PointLidar>
			void CmpCamLidarPd<PointCam, PointLidar>::setCamPd(typename pcl::PointCloud<PointCam>::Ptr pd) {
				*cam_pd_ = *pd;
				divideCamPdUsingLabel();

				dis_vec_vec_.resize(label_vec_.size());
				dis_dir_vec_vec_.resize(label_vec_.size());

				for (auto i = 0; i < label_vec_.size(); i++) {
					dis_vec_vec_[i].resize(cam_pd_vec_[i]->size(), 0);
					dis_dir_vec_vec_[i].resize(cam_pd_vec_[i]->size(), Eigen::Vector3f::Zero());
				}
			}

			template<class PointCam, class PointLidar>
			void CmpCamLidarPd<PointCam, PointLidar>::setLidarPd(typename 
				pcl::PointCloud<PointLidar>::Ptr pd) {
				*lidar_pd_ = *pd;
				divideLidarPdUsingLabel();
				//set the kdtree
				for (auto i = 0; i < lidar_pd_vec_.size(); i++) {
					lidar_kd_tree_vec_[i]->setInputCloud(lidar_pd_vec_[i]);
				}
			}

			template<class PointCam, class PointLidar>
			bool CmpCamLidarPd<PointCam, PointLidar>::divideCamPdUsingLabel() {
				if (cam_pd_->size() == 0) {
					LOG(FATAL) << "cam_pd_ size is 0! You must setCamPd() first.";
					return false;
				}
				for (auto cam_idx = 0; cam_idx < cam_pd_->size(); cam_idx++) {
					auto temp = cam_pd_->points[cam_idx];
					//get Z's cov = z^4 * cov(z)
					auto depth = temp.z;
					depth *= depth;
					depth *= depth;
					auto cov = depth * temp.cov[2];
					if (cov > cov_) continue;
					auto label = static_cast<HobotLabels>(temp.label);
					for (auto label_idx = 0; label_idx < label_vec_.size(); label_idx++) {
						if (label == label_vec_[label_idx]) {
							cam_pd_vec_[label_idx]->push_back(temp);
						}
					}
				}
			}

			template<class PointCam, class PointLidar>
			bool CmpCamLidarPd<PointCam, PointLidar>::divideLidarPdUsingLabel() {
				if (lidar_pd_->size() == 0) {
					LOG(FATAL) << "lidar_pd_ size is 0! You must setLidarPd() first.";
					return false;
				}
				for (auto lidar_idx = 0; lidar_idx < lidar_pd_->size(); lidar_idx++) {
					auto temp = lidar_pd_->points[lidar_idx];
					auto label = static_cast<HobotLabels>(temp.label);
					for (auto label_idx = 0; label_idx < label_vec_.size(); label_idx++) {
						if (label == label_vec_[label_idx]) {
							lidar_pd_vec_[label_idx]->push_back(temp);
						}
					}
				}
			}

		}
	}
}
#endif
