#include "suturo_perception_barcodescanner_node.h"

using namespace suturo_perception_barcodescanner;
using namespace zbar;

SuturoPerceptionBarcodeScannerNode::SuturoPerceptionBarcodeScannerNode(ros::NodeHandle& n) : nh_(n)
{
  logger = Logger("perception_barcodescanner");
  
  barcodeService_ = nh_.advertiseService("/suturo/barcodescanner", 
    &SuturoPerceptionBarcodeScannerNode::getCode, this);

  imageTopic_ = "/camera/image_raw";
  processing_ = false;

  logger.logInfo("BarcodeScanner Service ready!");

}

void SuturoPerceptionBarcodeScannerNode::receive_image(const sensor_msgs::ImageConstPtr& inputImage)
{
  if(processing_)
  {
    try
    {
      cv_bridge_ = cv_bridge::toCvCopy(inputImage, "mono8");  
    }
    catch(cv_bridge::Exception& e)
    {
      logger.logError("Could not convert image");
      processing_ = false;
      return;
    }

    ImageScanner is;
    is.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    int width  = cv_bridge_->image.cols;
    int height = cv_bridge_->image.rows;
    
    Magick::Blob blob(cv_bridge_->image.ptr(0), width * height);
    const void *raw = blob.data();
    Image image(width, height, "Y800", raw, width * height);

    std::vector<suturo_perception_msgs::Barcode> tmpBarcodes;
    if(is.scan(image) > 0) // codes found
    {
      for(Image::SymbolIterator symbol = image.symbol_begin();
          symbol != image.symbol_end(); ++symbol)
      {
        logger.logInfo((boost::format("Barcode found: Type: %s, Code: %s") 
          % symbol->get_type_name() % symbol->get_data()).str());

        suturo_perception_msgs::Barcode barcode;
        barcode.type = symbol->get_type_name();
        barcode.code = symbol->get_data();
        //cv::imwrite("/home/banacer/ros_workspace/yes.jpg",cv_bridge_ptr_->image);
        tmpBarcodes.push_back(barcode);   
     }
    }
    else
      logger.logWarn("No code found");

    mutex_.lock();
    currentBarcodes_ = tmpBarcodes; //result from is
    mutex_.unlock();
    
    logger.logDebug("finished callback");
    processing_ = false;
  }
}

bool SuturoPerceptionBarcodeScannerNode::getCode(suturo_perception_msgs::GetBarcode::Request &req,
        suturo_perception_msgs::GetBarcode::Response &res)
{
  logger.logDebug("Got service call");
  processing_ = true;

  sub_image_ = nh_.subscribe(imageTopic_, 1, 
          &SuturoPerceptionBarcodeScannerNode::receive_image, this);
  
  ros::Rate r(20); // 20 hz
  boost::posix_time::ptime cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(10);
  while(processing_)
  {
    if(boost::posix_time::second_clock::local_time() >= cancelTime)
    {
      processing_ = false;
      logger.logError("No image received in time. Call failed");
      return false;
    }
      
    ros::spinOnce();
    r.sleep();
  }

  mutex_.lock();
  res.barcodes = currentBarcodes_; // set service result
  mutex_.unlock();

  logger.logDebug("Call finished successfully");
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "suturo_perception_barcodescanner");
  ros::NodeHandle nh;
  SuturoPerceptionBarcodeScannerNode bsn(nh);
  
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  
  return 0;
}