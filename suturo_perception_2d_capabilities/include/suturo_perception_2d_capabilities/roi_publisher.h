#ifndef SUTURO_PERCEPTION_ROI_PUBLISHER_H
#define SUTURO_PERCEPTION_ROI_PUBLISHER_H

#include "boost/date_time/posix_time/posix_time.hpp"
#include "opencv2/core/core.hpp"
#include "capability.h"
#include "perceived_object.h"
#include "publisher_helper.h"

using namespace suturo_perception_ros_utils;

namespace suturo_perception_2d_capabilities
{
  /**
   * This class is able to extract a ROI from a given input image (called original_image_)
   * and publish it to a given topic.
   * The publishing will be done with the PublisherHelper class, implemented in
   * the package suturo_perception_ros_utils
   * Please ensure, that the topics are already advertised, when you try to
   * use this class.
   */
  class ROIPublisher : public suturo_perception_lib::Capability
  {
    public:
      // ROIPublisher();
      ROIPublisher(suturo_perception_lib::PerceivedObject &obj, PublisherHelper &ph, 
                   boost::shared_ptr<cv::Mat> original_image, std::string frame_id);
      void setTopicName(const std::string s); // optional

      // capability method
      void execute();

    private:
      suturo_perception_utils::Logger logger;
      std::string topic_name_;
      boost::shared_ptr<cv::Mat> original_image_;
      PublisherHelper &ph_;
      std::string frame_id_;
  };
}
#endif 
