#ifndef SUTURO_PERCEPTION_BARCODESCANNER_NODE_H
#define SUTURO_PERCEPTION_BARCODESCANNER_NODE_H

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <boost/signals2/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Magick++.h>
#include <zbar.h>

#include "suturo_perception_utils.h"
#include "suturo_perception_msgs/GetBarcode.h"
#include "suturo_perception_msgs/Barcode.h"

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
      std::vector<suturo_perception_msgs::Barcode> currentBarcodes_;
      boost::signals2::mutex mutex_;
      cv_bridge::CvImagePtr cv_bridge_;
      bool processing_;
  };

}

#endif