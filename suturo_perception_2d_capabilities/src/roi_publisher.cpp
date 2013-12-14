#include "suturo_perception_2d_capabilities/roi_publisher.h"

using namespace suturo_perception_2d_capabilities;

ROIPublisher::ROIPublisher(suturo_perception_lib::PerceivedObject &obj, PublisherHelper &ph, 
                           boost::shared_ptr<cv::Mat> original_image, std::string frame_id) 
    : topic_name_("roi_publisher"),
      ph_(ph), original_image_(original_image), frame_id_(frame_id),
      suturo_perception_lib::Capability(obj) {
}
/**
 * Read the ROI from the given PerceivedObject set by setPerceivedObject
 * Extract a cv::Mat from it
 * Publish it to given topics
 */
void ROIPublisher::execute()
{
    suturo_perception_lib::ROI &roi = perceivedObject.c_roi;
    cv::Rect region_of_interest = cv::Rect(
        roi.origin.x,
        roi.origin.y,
        roi.width,
        roi.height);
    cv::Mat image_roi = (*original_image_)(region_of_interest);

    ph_.publish_cv_mat(topic_name_ , image_roi, frame_id_);
}
void ROIPublisher::setTopicName(const std::string s)
{
  topic_name_ = s;
}
