#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZIT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float time;  // 当前点的时间
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                                      intensity)(float, time, time))
