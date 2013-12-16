#include "suturo_perception_2d_capabilities/label_annotator_2d.h"

using namespace suturo_perception_2d_capabilities;

LabelAnnotator2D::LabelAnnotator2D(PerceivedObject &obj, boost::signals2::mutex &m, boost::shared_ptr<cv::Mat> original_image, ObjectMatcher &object_matcher)
  : original_image_(original_image), object_matcher_(object_matcher), Capability(obj, m)
{
  logger_ = suturo_perception_utils::Logger("LabelAnnotator2D");
}

void LabelAnnotator2D::execute()
{
    suturo_perception_lib::ROI &roi = perceivedObject.c_roi;
    cv::Rect region_of_interest = cv::Rect(
        roi.origin.x,
        roi.origin.y,
        roi.width,
        roi.height);
    cv::Mat image_roi = (*original_image_)(region_of_interest);

    ObjectMatcher::ExecutionResult om_res = object_matcher_.recognizeTrainedImages(image_roi, true); // run headless (true)
    if(!om_res.label.empty()){
      logger_.logInfo( (boost::format(" Recognized a object with the label %s") % om_res.label).str() );
      perceivedObject.c_recognition_label_2d = om_res.label;
    }else{
      logger_.logInfo( "No Label found" );
    }
}
