#ifndef SUTURO_PERCEPTION_BARCODESCANNER_NODE_H
#define SUTURO_PERCEPTION_BARCODESCANNER_NODE_H

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <zbar.h>

#include "suturo_perception_utils.h"

using namespace suturo_perception_utils;
using namespace zbar;

namespace suturo_perception_barcodescanner
{

  class SuturoPerceptionBarcodeScannerNode
  {
    public:
      SuturoPerceptionBarcodeScannerNode(ros::NodeHandle& n);
      void receive_image(const sensor_msgs::ImageConstPtr& inputImage);

    private:
      ros::NodeHandle nh_;
      ros::ServiceServer barcodeService_;
  };

}

#endif