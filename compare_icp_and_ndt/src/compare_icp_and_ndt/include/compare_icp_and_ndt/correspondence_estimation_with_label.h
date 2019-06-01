// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     correspondence_estimation.h
// ROLE:     TODO (some explanation)
// CREATED:  2019-02-23 16:40:39
// MODIFIED: 2019-02-27 11:26:44

#include <string>
#include <deque>

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>

namespace pcl
{
  namespace registration
  {
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
	/** \brief @b CorrespondenceEstimationWithLabel represents the base class for
      * determining correspondences between target and query point
      * sets/features.
      *
      * Code example:
      *
      * \code
      * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
      * // ... read or fill in source and target
      * pcl::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      * est.setInputSource (source);
      * est.setInputTarget (target);
      *
      * pcl::Correspondences all_correspondences;
      * // Determine all reciprocal correspondences
      * est.determineReciprocalCorrespondences (all_correspondences);
      * \endcode
      *
      * \author Radu B. Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CorrespondenceEstimationWithLabel : public CorrespondenceEstimation<PointSource, PointTarget, Scalar>
    {
      public:
        typedef boost::shared_ptr<CorrespondenceEstimationWithLabel<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const CorrespondenceEstimationWithLabel<PointSource, PointTarget, Scalar> > ConstPtr;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_cloud_updated_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::force_no_recompute_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_reciprocal_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
        //using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
        using PCLBase<PointSource>::deinitCompute;

        typedef pcl::search::KdTree<PointTarget> KdTree;
        typedef typename pcl::search::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceEstimationWithLabel () 
        {
          corr_name_  = "CorrespondenceEstimationWithLabel";
        }

        CorrespondenceEstimationWithLabel (int label_size,
					const std::vector<HobotLabels>& valid_label_vec): label_size_(label_size),
					valid_label_vec_(valid_label_vec)
        {
					for (auto i = 0; i < label_size_; i++) {
						PointCloudTargetPtr temp_pd(new PointCloudTarget);
						target_vec_.push_back(temp_pd);

						KdTreePtr temp_kdtree(new KdTree);
						tree_vec_.push_back(temp_kdtree);
					}
        }
      
        /** \brief Empty destructor */
        virtual ~CorrespondenceEstimationWithLabel () {}
        
				//void setInputTarget (const PointCloudTargetConstPtr &cloud)
				void setInputTarget ()
				{

					//TODO
					std::cout << "I am here!\n";
					//TODO end

					if (target_->points.empty ())
					{
						PCL_ERROR ("[pcl::registration::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
						return;
					}
					//target_ = cloud;
		 
					// Set the internal point representation of choice
					if (point_representation_)
						tree_->setPointRepresentation (point_representation_);
			 
					//target_cloud_updated_ = true;

					divideLidarPdUsingLabel();
				}

				bool initCompute();

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ());

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        //virtual void 
        //determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            //double max_distance = std::numeric_limits<double>::max ());

        
        /** \brief Clone and cast to CorrespondenceEstimationBase */
        virtual boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > 
        clone () const
        {
          Ptr copy (new CorrespondenceEstimationWithLabel<PointSource, PointTarget, Scalar> (*this));
          return (copy);
        }
        
				inline bool isLabelValid (HobotLabels label) {
					for (auto i = 0; i < valid_label_vec_.size(); i++) {
						if (label == valid_label_vec_[i]) return true;
					}
					return false;
				}

        bool divideLidarPdUsingLabel() {
					if (target_->size() == 0) {
						std::cerr << "lidar_pd_ size is 0! You must setLidarPd() first.";
						return false;
					}
					//TODO
					if (target_indices_) 
						std::cout << "The target_indices_ is not null, "
							<< "So the code can's deal with this!\n";

					//clear target_vec_. reset have_pd_deque
					for (auto i = 0; i < target_vec_.size(); i++) {
						target_vec_[i]->clear();
					}

					have_pd_deque_.resize(target_vec_.size(), false);

					for (auto lidar_idx = 0; lidar_idx < target_->size(); lidar_idx++) {
						auto temp = target_->points[lidar_idx];
#ifdef Intensity_instead_Label
						auto label = static_cast<HobotLabels>(temp.intensity);
#else
						auto label = static_cast<HobotLabels>(temp.label);
#endif
						if (isLabelValid(label)) {
							target_vec_[label]->push_back(temp);
						}
					}

					for (auto i = 0; i < target_vec_.size(); i++) {
						if (target_vec_[i]->size() > 0) {
							have_pd_deque_[i] = true;
						}
					}

				}

			protected:
				std::vector<KdTreePtr> tree_vec_;
				std::vector<PointCloudTargetPtr> target_vec_; 
				std::vector<HobotLabels> valid_label_vec_; 
				int label_size_;

				std::deque<bool> have_pd_deque_;

     };
   
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl::registration::CorrespondenceEstimationWithLabel<PointSource, PointTarget, Scalar>::initCompute ()
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  // Only update target kd-tree if a new target cloud was set
  if (target_cloud_updated_ && !force_no_recompute_)
  {
    // If the target indices have been given via setIndicesTarget

    //if (target_indices_)
      //tree_->setInputCloud (target_, target_indices_);
    //else
      //tree_->setInputCloud (target_);

		//TODO hack call setInputTarget
		setInputTarget();
		//TODO

		//set Label cloud to tree_vec_
		for (auto i = 0; i < target_vec_.size(); i++) {
			if (target_vec_[i]->size() > 0) tree_vec_[i]->setInputCloud(target_vec_[i]);
		}

    target_cloud_updated_ = false;
  }

  return (PCLBase<PointSource>::initCompute ());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimationWithLabel<PointSource, PointTarget, Scalar>::determineCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{

  if (!initCompute ())
    return;

  double max_dist_sqr = max_distance * max_distance;

  correspondences.resize (indices_->size ());

  std::vector<int> index (1);
  std::vector<float> distance (1);
  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;
  
  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (isSamePointType<PointSource, PointTarget> ())
  {
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
			//getLabel
#ifdef Intensity_instead_Label
			auto label = static_cast<HobotLabels>(input_->points[*idx].intensity);
#else
			auto label = static_cast<HobotLabels>(input_->points[*idx].label);
#endif
			if (!isLabelValid(label) || !have_pd_deque_[label]) continue;

      //tree_->nearestKSearch (input_->points[*idx], 1, index, distance);
      tree_vec_[label]->nearestKSearch (input_->points[*idx], 1, index, distance);

      if (distance[0] > max_dist_sqr)
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  else
  {
    PointTarget pt;
    
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      // Copy the source data to a target PointTarget format so we can search in the tree
      copyPoint (input_->points[*idx], pt);

			//getLabel
#ifdef Intensity_instead_Label
			auto label = static_cast<HobotLabels>(pt.intensity);
#else
			auto label = static_cast<HobotLabels>(pt.label);
#endif
			if (!isLabelValid(label) || !have_pd_deque_[label]) continue;

      //tree_->nearestKSearch (pt, 1, index, distance);
      tree_vec_[label]->nearestKSearch (pt, 1, index, distance);

      if (distance[0] > max_dist_sqr)
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  correspondences.resize (nr_valid_correspondences);
	
	std::cout << "correspondences pair size: " << correspondences.size() << "\n";

  deinitCompute ();
}

