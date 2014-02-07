#ifndef SUTURO_PERCEPTION_BARCODESCANNER_NODE_H
#define SUTURO_PERCEPTION_BARCODESCANNER_NODE_H

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <boost/signals2/mutex.hpp>
#include <zbar.h>

#include "suturo_perception_utils.h"
#include "suturo_perception_msgs/GetBarcode.h"

using namespace suturo_perception_utils;
using namespace zbar;

namespace suturo_perception_barcodescanner
{

  class SuturoPerceptionBarcodeScannerNode
  {
    public:
      SuturoPerceptionBarcodeScannerNode(ros::NodeHandle& n);
      void receive_image(const sensor_msgs::ImageConstPtr& inputImage);
      bool getCode(suturo_perception_msgs::GetBarcode::Request &req,
        suturo_perception_msgs::GetBarcode::Response &res);

    private:
      ros::NodeHandle nh_;
      ros::ServiceServer barcodeService_;
      Logger logger;
      ros::Subscriber sub_image_;
      std::string imageTopic_;
      std::string currentBarcode_;
      boost::signals2::mutex mutex_;
      bool processing_;
  };

}

#endif