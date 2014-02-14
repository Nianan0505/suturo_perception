#include "suturo_perception_barcodescanner_node.h"

using namespace suturo_perception_barcodescanner;
using namespace zbar;

static const char WINDOW[] = "Suturo BarcodeScanner";

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

        // cv playground
        try
        {
          cv::cvtColor(cv_bridge_->image, cv_bridge_->image, CV_GRAY2BGR);  
        }
        catch(cv::Exception& e)
        {
          logger.logWarn("CV Exception thrown in color conversion");
        }

        for(int i = 0; i <= symbol->get_location_size(); ++i)
        {
          cv::Point point;
          point.x = symbol->get_location_x(i);
          point.y = symbol->get_location_y(i);
          cv::ellipse(cv_bridge_->image, point, cv::Size(5,5), 0.0, 0.0, 0.0, cv::Scalar(0,255,0), 7, 8, 0);
        }

        cv::Point pointLeft = getTopLeftIndex(*symbol);
        cv::Point pointRight = getBottomRightIndex(*symbol);
        cv::rectangle(cv_bridge_->image, pointRight, pointLeft, cv::Scalar(255,0,0), 2);

        tmpBarcodes.push_back(barcode);   
     }
    }
    else
      logger.logWarn("No code found");

    cv::resize(cv_bridge_->image, cv_bridge_->image, cv::Size(848,480));
    cv::imshow(WINDOW, cv_bridge_->image);
    cv::waitKey(3); // wait 3ms

    mutex_.lock();
    currentBarcodes_ = tmpBarcodes; //result from is
    mutex_.unlock();
    
    logger.logDebug("finished callback");
    processing_ = false;
  }
}

cv::Point SuturoPerceptionBarcodeScannerNode::getTopLeftIndex(const Symbol &symbol)
{
  int minX = 42000;
  int minY = 42000;

  for(int i = 0; i <= symbol.get_location_size(); ++i)
  { 
    // lib return funky -1 Points for an index, check that index position is > 0
    if(minX > symbol.get_location_x(i) && symbol.get_location_x(i) > 0) minX = symbol.get_location_x(i);
    if(minY > symbol.get_location_y(i) && symbol.get_location_x(i) > 0) minY = symbol.get_location_y(i);
  }
    
  return cv::Point(minX-20,minY-20);
}

cv::Point SuturoPerceptionBarcodeScannerNode::getBottomRightIndex(const Symbol &symbol)
{
  int maxX = 0;
  int maxY = 0;

  for(int i = 0; i <= symbol.get_location_size(); ++i)
  {
    if(maxX < symbol.get_location_x(i)) maxX = symbol.get_location_x(i);
    if(maxY < symbol.get_location_y(i)) maxY = symbol.get_location_y(i);
  }
    
  return cv::Point(maxX+20,maxY+20);
}

bool SuturoPerceptionBarcodeScannerNode::getCode(suturo_perception_msgs::GetBarcode::Request &req,
        suturo_perception_msgs::GetBarcode::Response &res)
{
  logger.logDebug("Got service call");
  processing_ = true;

  sub_image_ = nh_.subscribe(imageTopic_, 1, 
          &SuturoPerceptionBarcodeScannerNode::receive_image, this);
  
  ros::Rate r(50); // 50 hz
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
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
  SuturoPerceptionBarcodeScannerNode bsn(nh);
  
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  
  return 0;
}