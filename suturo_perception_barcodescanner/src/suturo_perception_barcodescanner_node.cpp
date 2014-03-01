#include "suturo_perception_barcodescanner_node.h"

using namespace suturo_perception_barcodescanner;
using namespace zbar;

static const char WINDOW[] = "Suturo BarcodeScanner";

SuturoPerceptionBarcodeScannerNode::SuturoPerceptionBarcodeScannerNode(ros::NodeHandle& n, std::string ci, 
                                                                       std::string vd, std::string ii) 
  : nh_(n),
    cameraImageTopic_(ci),
    videoDevice_(vd),
    infoImageTopic_(ii)
{
  logger = Logger("perception_barcodescanner");
  
  barcodeService_ = nh_.advertiseService("/suturo/barcodescanner", 
    &SuturoPerceptionBarcodeScannerNode::getCode, this);

  processing_ = false;
  focusValue_ = 250;
  want_new_images_ = true;

  imagePub_ = nh_.advertise<sensor_msgs::Image>(infoImageTopic_, 1);

  logger.logInfo("BarcodeScanner Service ready!");
}

/*
 * Callback receiving the webcam image
 */
void SuturoPerceptionBarcodeScannerNode::receive_image(const sensor_msgs::ImageConstPtr& inputImage)
{
  if(want_new_images_)
  {
    want_new_images_ = false;
    // reset the info images
    infoPoints_.clear();
    infoBoxPairs_.clear();

    try
    {
      // do an extra gray scale conversion. copying to mono8 directly 
      // will result in a distorted sensor_msgs image
      cv_bridge_ = cv_bridge::toCvCopy(inputImage, "bgr8");
      cv::cvtColor(cv_bridge_->image, cv_bridge_->image, CV_BGR2GRAY);
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

        // mark the found symbol in the input image
        computeInfoImage(*symbol);
        tmpBarcodes.push_back(barcode);
      }
    }
    else
    {
      logger.logWarn((boost::format("No code found, refocusing to %i") % focusValue_).str());
      refocus(focusValue_);
      if (focusValue_ <= 0) focusValue_ = 250;
      else focusValue_-=10; // run through the focus values to find a code
      if(processing_ == true) want_new_images_ = true;
      else want_new_images_ = false; // service callback timed out, probably no barcode there
      return; // focus reset, wait for new image
    }

    publishInfoImage();

    mutex_.lock();
    currentBarcodes_ = tmpBarcodes; //result from image scanner
    mutex_.unlock();
    
    logger.logDebug("finished callback");
    processing_ = false;
  }
}

/*
 * For each symbol found, compute the registered location points and a bounding box.
 * Since the original image is processed iteratively by zbar, the computed values are
 * stored in containers, so the original image is not modified while zbar is still processing it.
 */
void SuturoPerceptionBarcodeScannerNode::computeInfoImage(const Symbol &symbol)
{
  try
  {
    cv::cvtColor(cv_bridge_->image, cv_bridge_->image, CV_GRAY2BGR);
  }
  catch(cv::Exception& e)
  {
    logger.logWarn("CV Exception thrown in color conversion");
  }

  for(int i = 0; i <= symbol.get_location_size(); ++i)
  {
    cv::Point point;
    point.x = symbol.get_location_x(i);
    point.y = symbol.get_location_y(i);
    infoPoints_.push_back(point);
  }

  cv::Point pointLeft = getTopLeftIndex(symbol);
  cv::Point pointRight = getBottomRightIndex(symbol);
  infoBoxPairs_.push_back(std::make_pair(pointLeft, pointRight));
}

/*
 * Draw the computed points and bounding boxes and publish the info image.
 */
void SuturoPerceptionBarcodeScannerNode::publishInfoImage()
{
  for(std::vector<cv::Point>::iterator it = infoPoints_.begin(); it != infoPoints_.end(); ++ it)
    cv::ellipse(cv_bridge_->image, *it, cv::Size(5,5), 0.0, 0.0, 0.0, cv::Scalar(0,255,0), 7, 8, 0);
  for(std::vector<InfoBoxPair>::iterator it = infoBoxPairs_.begin(); it != infoBoxPairs_.end(); ++ it)
    cv::rectangle(cv_bridge_->image, it->second, it->first, cv::Scalar(255,0,0), 2);
  
  cv::resize(cv_bridge_->image, cv_bridge_->image, cv::Size(848,480));
  imagePub_.publish(cv_bridge_->toImageMsg());
}

/*
 * Get the top left index of a symbol to compute a bounding box
 */
cv::Point SuturoPerceptionBarcodeScannerNode::getTopLeftIndex(const Symbol &symbol)
{
  int minX = 42000;
  int minY = 42000;

  for(int i = 0; i <= symbol.get_location_size(); ++i)
  { 
    // lib returns funky -1 Points for an index, check that index position is > 0
    if(minX > symbol.get_location_x(i) && symbol.get_location_x(i) > 0) minX = symbol.get_location_x(i);
    if(minY > symbol.get_location_y(i) && symbol.get_location_x(i) > 0) minY = symbol.get_location_y(i);
  }
    
  return cv::Point(minX-20,minY-20);
}

/*
 * Get the bottom right index of a symbol to compute a bounding box
 */
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

/*
 * Service method called by clients of the barcode service
 */
bool SuturoPerceptionBarcodeScannerNode::getCode(suturo_perception_msgs::GetBarcode::Request &req,
        suturo_perception_msgs::GetBarcode::Response &res)
{
  logger.logDebug("Got service call");
  processing_ = true;
  want_new_images_ = true;

  sub_image_ = nh_.subscribe(cameraImageTopic_, 1, 
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

/*
 * Send a service call to the focus server to adjust the focus
 * of the camera, if no code is found.
 */
void SuturoPerceptionBarcodeScannerNode::refocus(uint8_t focusValue)
{
  ros::ServiceClient focusClient = nh_.serviceClient<suturo_perception_msgs::ScannerFocus>("/suturo/ScannerFocus");
  suturo_perception_msgs::ScannerFocus scannerFocus;
  scannerFocus.request.videoDevice = videoDevice_;
  scannerFocus.request.focusValue = focusValue;
  if (!focusClient.call(scannerFocus))
    logger.logWarn("refocus service call failed");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "suturo_perception_barcodescanner");
  ros::NodeHandle nh;
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

  // get parameters
  std::string cameraImageTopic, infoImageTopic, videoDevice;
  if(ros::param::get("/suturo_perception/barcode_image_topic", cameraImageTopic)) ROS_INFO("Using parameters from Parameter Server");
  else { cameraImageTopic = "/camera/image_raw"; ROS_INFO("Using default parameters");}
  if(ros::param::get("/suturo_perception/barcode_video_device", videoDevice)) ROS_INFO("Using parameters from Parameter Server");
  else { videoDevice = "/dev/video1"; ROS_INFO("Using default parameters");}
  if(ros::param::get("/suturo_perception/barcode_info_image_topic", infoImageTopic)) ROS_INFO("Using parameters from Parameter Server");
  else { infoImageTopic = "/suturo/barcode_info_image"; ROS_INFO("Using default parameters");}

  SuturoPerceptionBarcodeScannerNode bsn(nh, cameraImageTopic, videoDevice, infoImageTopic);
  
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  
  return 0;
}