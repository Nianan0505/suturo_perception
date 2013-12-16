#ifndef SUTURO_PERCEPTION_LABEL_ANNOTATOR_2D_H
#define SUTURO_PERCEPTION_LABEL_ANNOTATOR_2D_H

#include <boost/signals2/mutex.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "opencv2/core/core.hpp"
#include "capability.h"
#include "perceived_object.h"
#include "object_matcher.h"  // Include 2d Object recognizer
#include "suturo_perception_utils.h"

using namespace suturo_perception_lib;

namespace suturo_perception_2d_capabilities
{
  /**
   * This class will use the 
   * 2d_object_recognition_lib to match a given
   * image with trained images and put the recognized
   * label into the given PerceivedObject.
   *
   * The ROI for the object recognition has to be passed in the
   * PerceivedObject.
   */
  class LabelAnnotator2D : public suturo_perception_lib::Capability
  {
    public:
      // capability method
      LabelAnnotator2D(PerceivedObject &obj, boost::shared_ptr<cv::Mat> original_image, ObjectMatcher &object_matcher);
      void execute();

    private:
      boost::shared_ptr<cv::Mat> original_image_;
      ObjectMatcher &object_matcher_;
      suturo_perception_utils::Logger logger_;
  };
}
#endif
