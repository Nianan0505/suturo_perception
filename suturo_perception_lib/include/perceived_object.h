#ifndef PERCEIVED_OBJECT_H
#define PERCEIVED_OBJECT_H

#include "point.h"
#include "roi.h"
#include <pcl/point_types.h>

namespace suturo_perception_lib
{
  class PerceivedObject
  {
    public:
      int c_id;
      Point c_centroid;
      double c_volume;
      int c_shape;
      uint8_t c_color_average_r;
      uint8_t c_color_average_g;
      uint8_t c_color_average_b;
      uint32_t c_color_average_h;
      double c_color_average_s;
      double c_color_average_v;
      std::string c_recognition_label_2d;
      std::vector<uint32_t> c_hue_histogram;
      uint8_t c_hue_histogram_quality;
      ROI c_roi;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
  };
}

#endif
