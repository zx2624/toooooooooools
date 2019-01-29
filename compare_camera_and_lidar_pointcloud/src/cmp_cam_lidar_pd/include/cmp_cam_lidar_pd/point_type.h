#ifndef HORIZON_MAPPING_EVALUATION_POINT_TYPE_H_
#define HORIZON_MAPPING_EVALUATION_POINT_TYPE_H_

#include <pcl/point_types.h>
#define PCL_NO_PRECOMPILE
namespace pcl {

struct EIGEN_ALIGN16 _PointXYZRGBLC {
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  unsigned char label;
  float cov;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointXYZRGBLCov
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;
	unsigned char label;
	float cov[3];
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16; // enforce SSE padding for correct memory alignment

struct EIGEN_ALIGN16 PointXYZRGBLC : public _PointXYZRGBLC
{
  inline PointXYZRGBLC (const _PointXYZRGBLC &p)
  {
    x = p.x; y = p.y; z = p.z;
		r = p.r; g = p.g; b = p.b;
		label = p.label;
		cov = p.cov;
  }

  inline PointXYZRGBLC (const _PointXYZI &p)
  {
    x = p.x; y = p.y; z = p.z;

		r = 0; 
		g = 0;
		b = 0;
		label = 0;
		cov = 0.0f;
  }

  inline PointXYZRGBLC (const float px, const float py, const float pz, const float pintensity)
  {
    x = px; y = py; z = pz;

		r = 0; 
		g = 0;
		b = 0;
		label = 0;
		cov = 0.0f;
  }

  inline PointXYZRGBLC ()
  {
    x = y = z = 0.0;

		r = 0; 
		g = 0;
		b = 0;
		label = 0;
		cov = 0.0f;
  }

  friend std::ostream& operator << (std::ostream& os, const PointXYZRGBLC& p);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}//namespace pcl

#include <pcl/impl/point_types.hpp>  // Include struct definitions

// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZRGBLC,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (unsigned char, r, r)
    (unsigned char, g, g)
    (unsigned char, b, b)
    (unsigned char, label, label)
    (float, cov, cov)
)
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZRGBLCov,
																	 (float, x, x)
																	 (float, y, y)
																	 (float, z, z)
																	 (unsigned char, r, r)
																	 (unsigned char, g, g)
																	 (unsigned char, b, b)
																	 (unsigned char, label, label)
																	 (float[3], cov, cov)
																	)
																	
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZRGBLC, pcl::_PointXYZRGBLC)



#endif
