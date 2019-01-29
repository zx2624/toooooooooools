// C/C++ File
// AUTHOR: siyuan.yu(siyuan.yu01@hobot.cc)
// FILE:     point_type.h
// ROLE:     TODO (some explanation)
// CREATED:  2019-01-23 10:37:17
// MODIFIED: 2019-01-29 14:55:04
#define PCL_NO_PRECOMPILE

#ifndef HORIZON_MAPPING_EVALUATION_POINT_TYPE_H_
#define HORIZON_MAPPING_EVALUATION_POINT_TYPE_H_
#include <pcl/point_types.h>

namespace horizon {
 namespace mapping {
  namespace evaluation {
		struct PointXYZRGBLCov
				{
					PCL_ADD_POINT4D;
					PCL_ADD_RGB;
					unsigned char label;
					float cov[3];
					EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
				} EIGEN_ALIGN16; // enforce SSE padding for correct memory alignment
	}
 }
}

POINT_CLOUD_REGISTER_POINT_STRUCT (horizon::mapping::evaluation::PointXYZRGBLCov,
																	 (float, x, x)
																	 (float, y, y)
																	 (float, z, z)
																	 (unsigned char, r, r)
																	 (unsigned char, g, g)
																	 (unsigned char, b, b)
																	 (unsigned char, label, label)
																	 (float[3], cov, cov)
																	)
#endif
