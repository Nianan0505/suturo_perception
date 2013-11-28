#ifndef PERCEIVED_OBJECT_H
#define PERCEIVED_OBJECT_H

#include "point.h"

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
      std::string c_recognition_label_2d;
  };
}

#endif
