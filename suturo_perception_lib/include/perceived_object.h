#ifndef PERCEIVED_OBJECT_H
#define PERCEIVED_OBJECT_H

#include "point.h"
#include "roi.h"

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
      uint8_t c_color_average_h;
      uint8_t c_color_average_s;
      uint8_t c_color_average_v;
      std::string c_recognition_label_2d;
      std::vector<int> c_hue_histogram;
      uint8_t c_hue_histogram_quality;
      ROI c_roi;
  };
}

#endif
