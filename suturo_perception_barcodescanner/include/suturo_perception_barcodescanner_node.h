#ifndef SUTURO_PERCEPTION_BARCODESCANNER_NODE_H
#define SUTURO_PERCEPTION_BARCODESCANNER_NODE_H

#include "ros/ros.h"
#include <utility>
#include <sensor_msgs/Image.h>
#include <boost/signals2/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Magick++.h>
#include <zbar.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <image_transport/image_transport.h>

#include "suturo_perception_utils.h"
#include "suturo_perception_msgs/GetBarcode.h"
#include "suturo_perception_msgs/Barcode.h"
#include "suturo_perception_msgs/ScannerFocus.h"
#include "highgui.h"
#include <opencv2/opencv.hpp>

using namespace suturo_perception_utils;
using namespace zbar;

namespace suturo_perception_barcodescanner
{

  class SuturoPerceptionBarcodeScannerNode
  {
    public:
      SuturoPerceptionBarcodeScannerNode(ros::NodeHandle& n, std::string ci, std::string vd, std::string ii);
      void receive_image(const sensor_msgs::ImageConstPtr& inputImage);
      bool getCode(suturo_perception_msgs::GetBarcode::Request &req,
        suturo_perception_msgs::GetBarcode::Response &res);

    private:
      ros::NodeHandle nh_;
      ros::ServiceServer barcodeService_;
      Logger logger;
      ros::Subscriber sub_image_;
      std::string cameraImageTopic_;
      std::string videoDevice_;
      std::string infoImageTopic_;
      std::vector<suturo_perception_msgs::Barcode> currentBarcodes_;
      boost::signals2::mutex mutex_;
      cv_bridge::CvImagePtr cv_bridge_;
      ros::Publisher imagePub_;
      bool processing_;
      bool want_new_images_;
      int focusValue_;
      typedef std::pair <cv::Point, cv::Point> InfoBoxPair; // Point pairs for box drawing
      std::vector<InfoBoxPair> infoBoxPairs_; // vector for the box pairs
      std::vector<cv::Point> infoPoints_;     // vector for the symbol points
      
      void computeInfoImage(const Symbol&);
      cv::Point getTopLeftIndex(const Symbol&);
      cv::Point getBottomRightIndex(const Symbol&);
      void publishInfoImage();
      void refocus(uint8_t);
  };

}

#endif