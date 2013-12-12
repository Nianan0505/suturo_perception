#ifndef SUTURO_PERCEPTION_ROI_H
#define SUTURO_PERCEPTION_ROI_H

#include "point2d.h"

namespace suturo_perception_lib
{
	/**
	 * This class represents a Region Of Interest (ROI) in a 2d image.
   * These values can be used, to extract a specific region
   * with cv::rect like this:
   *   cv::Rect region_of_interest = cv::Rect(roi_topleft_x, roi_topleft_y, roi_width, roi_height);
   *   cv::Mat image_roi = img(region_of_interest);
   */
  class ROI
  {
      public:
        // The top-left pixel in the Region of Interest
        Point2D origin;
        int width;
        int height;
  };
}
#endif
