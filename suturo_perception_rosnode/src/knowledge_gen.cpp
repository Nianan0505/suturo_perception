#include "ros/ros.h"
#include <boost/signals2/mutex.hpp>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <dynamic_reconfigure/server.h>
#include <suturo_perception_rosnode/SuturoPerceptionConfig.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>

#include "suturo_perception.h"
#include "visualization_publisher.h"
#include "suturo_perception_2d_capabilities/roi_publisher.h"
#include "random_sample_consensus.h" // shape detector capability
#include "perceived_object.h"
#include "point.h"
#include "suturo_perception_msgs/GetClusters.h"
#include "suturo_perception_msgs/PrologQuery.h"
#include "suturo_perception_msgs/PrologNextSolution.h"
#include "suturo_perception_msgs/PrologFinish.h"
#include "object_matcher.h"  // Include 2d Object recognizer
#include "color_analysis.h"
#include "publisher_helper.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "suturo_perception_utils.h"
#include <sensor_msgs/Image.h>
#include "suturo_perception_2d_capabilities/label_annotator_2d.h"
#include "vfh_estimation.h"
#include "svm_classification.h"
#include "suturo_perception_3d_capabilities/cuboid_matcher_annotator.h"

using namespace suturo_perception_ros_utils;
using namespace suturo_perception_utils;
using namespace suturo_perception_color_analysis;
using namespace suturo_perception_svm_classification;
namespace enc = sensor_msgs::image_encodings;

  bool processing; // processing flag
  bool callback_called; // cloud and image received, callback is running
  bool fallback_enabled; // flag, if fallback to just cloud data is enabled
  ros::Subscriber sub_cloud; // fallback subscriber
  ObjectMatcher object_matcher_;
  suturo_perception_lib::SuturoPerception sp;
  std::vector<suturo_perception_lib::PerceivedObject> perceivedObjects;
  SVMClassification svm_classification;
  boost::signals2::mutex mutex;
  // ID counter for the perceived objects
  int objectID;
  // services
  ros::ServiceServer clusterService;
  ros::ServiceClient is_edible_service;
  ros::ServiceClient is_edible_service_next;
  ros::ServiceClient is_edible_service_finish;
  
  std::string pointTopic = "/kinect_head/depth_registered/points";
  std::string colorTopic = "";
  std::string frameId = "";
  std::string recognitionDir = "";
  // dynamic reconfigure
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig> reconfSrv;
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig>::CallbackType reconfCb;
  // color analysis config
  double color_analysis_lower_s;
  double color_analysis_upper_s;
  double color_analysis_lower_v;
  double color_analysis_upper_v;

  VisualizationPublisher visualizationPublisher;
  Logger logger;

  int numThreads;
  
/*
 * Receive callback for the /camera/depth_registered/points subscription
 */
void receive_image_and_cloud(const sensor_msgs::ImageConstPtr& inputImage, 
                             const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  // process only one cloud
  callback_called = true;
  if(processing)
  {
    logger.logInfo("Receiving cloud");
    logger.logInfo("processing...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*inputCloud,*cloud_in);

    // Gazebo sends us unorganized pointclouds!
    // Reorganize them to be able to compute the ROI of the objects
    // This workaround is only tested for gazebo 1.9!
    if(!cloud_in->isOrganized ())
    {
      logger.logInfo((boost::format("Received an unorganized PointCloud: %d x %d .Convert it to a organized one ...") % cloud_in->width % cloud_in->height ).str());

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr org_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
      org_cloud->width = 640;
      org_cloud->height = 480;
      org_cloud->is_dense = false;
      org_cloud->points.resize(640 * 480);

      for (int i = 0; i < cloud_in->points.size(); i++) {
          pcl::PointXYZRGB result;
          result.x = 0;
          result.y = 0;
          result.z = 0;
          org_cloud->points[i]=cloud_in->points[i];
      }

      cloud_in = org_cloud;
    }
    if(!fallback_enabled)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(inputImage, enc::BGR8);

      // Make a deep copy of the passed cv::Mat and set a new
      // boost pointer to it.
      boost::shared_ptr<cv::Mat> img(new cv::Mat(cv_ptr->image.clone()));
      sp.setOriginalRGBImage(img);
    }
    
    std::stringstream ss;
    ss << "Received a new point cloud: size = " << cloud_in->points.size();
    logger.logInfo((boost::format("Received a new point cloud: size = %s") % cloud_in->points.size()).str());
    sp.setOriginalCloud(cloud_in);
    sp.processCloudWithProjections(cloud_in);
    processing = false;
    logger.logInfo("Cloud processed. Lock buffer and return the results");      
  }
  callback_called = false;
}

void fallback_receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  if(processing)
  {
    logger.logInfo("Received a pointcloud message");
    sensor_msgs::ImageConstPtr nullPtr; // boost::shared_ptr NullPtr
    receive_image_and_cloud(nullPtr, inputCloud);
  }
}
  
  /*
std::vector<suturo_perception_msgs::PerceivedObject> *convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject> *objects)
{
  std::vector<suturo_perception_msgs::PerceivedObject> *result = new std::vector<suturo_perception_msgs::PerceivedObject>();
  for (std::vector<suturo_perception_lib::PerceivedObject>::iterator it = objects->begin(); it != objects->end(); ++it)
  {
    suturo_perception_msgs::PerceivedObject *msgObj = new suturo_perception_msgs::PerceivedObject();
    msgObj->c_id = it->get_c_id();
    msgObj->c_shape = it->get_c_shape();
    msgObj->c_volume = it->get_c_volume();
    msgObj->c_centroid.x = it->get_c_centroid().x;
    msgObj->c_centroid.y = it->get_c_centroid().y;
    msgObj->c_centroid.z = it->get_c_centroid().z;
    msgObj->frame_id = frameId;
    msgObj->c_color_average_r = it->get_c_color_average_r();
    msgObj->c_color_average_g = it->get_c_color_average_g();
    msgObj->c_color_average_b = it->get_c_color_average_b();
    msgObj->c_color_average_h = it->get_c_color_average_h();
    msgObj->c_color_average_s = it->get_c_color_average_s();
    msgObj->c_color_average_v = it->get_c_color_average_v();
    msgObj->c_color_average_qh = it->get_c_color_average_qh();
    msgObj->c_color_average_qs = it->get_c_color_average_qs();
    msgObj->c_color_average_qv = it->get_c_color_average_qv();
    msgObj->c_roi_origin.x = it->get_c_roi().origin.x;
    msgObj->c_roi_origin.y = it->get_c_roi().origin.y;
    msgObj->c_roi_width = it->get_c_roi().width;
    msgObj->c_roi_height = it->get_c_roi().height;
    msgObj->c_hue_histogram = *it->get_c_hue_histogram();
    msgObj->c_hue_histogram_quality = it->get_c_hue_histogram_quality();
    msgObj->recognition_label_2d = it->get_c_recognition_label_2d();
  
    Cuboid c = it->get_c_cuboid();
    msgObj->matched_cuboid.length1  = c.length1;
    msgObj->matched_cuboid.length2  = c.length2;
    msgObj->matched_cuboid.length3  = c.length3;
    msgObj->matched_cuboid.volume   = c.volume;
    msgObj->matched_cuboid.pose.position.x   = c.center(0);
    msgObj->matched_cuboid.pose.position.y   = c.center(1);
    msgObj->matched_cuboid.pose.position.z   = c.center(2);
    msgObj->matched_cuboid.pose.orientation.x   = c.orientation.x();
    msgObj->matched_cuboid.pose.orientation.y   = c.orientation.y();
    msgObj->matched_cuboid.pose.orientation.y   = c.orientation.z();
    msgObj->matched_cuboid.pose.orientation.w   = c.orientation.w();

    for (int i = 0; i < 308; i++) 
    {
      msgObj->c_vfh_estimation.push_back(it->get_c_vfhs().histogram[i]);
    }
    msgObj->c_svm_result = ""; //svm_classification.classifyVFHSignature308(it->get_c_vfhs());
    msgObj->c_pose = 0; //svm_classification.classifyPoseVFHSignature308(it->get_c_vfhs(), msgObj->c_svm_result);

    // these are not set for now
    msgObj->recognition_label_3d = "";
    
    result->push_back(*msgObj);
  }
  return result;
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "knowledge_gen");

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, colorTopic, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, pointTopic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, pc_sub);

  sync.registerCallback(boost::bind(&receive_image_and_cloud, _1, _2));

  logger.logInfo("Waiting for processed cloud");
  ros::Rate r(20); // 20 hz
  // cancel service call, if no cloud is received after 10s
  boost::posix_time::ptime cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(10);
  while(processing)
  {
    if(boost::posix_time::second_clock::local_time() >= cancelTime && !callback_called)
    {
      processing = false;
      // register normal callback just for the cloud topic and retry, if no color data is available
      if(!fallback_enabled)
      {
        processing = true;
        logger.logWarn("No color image received. Falling back to point clouds only.");
        image_sub.unsubscribe();
        pc_sub.unsubscribe();
        sub_cloud = nh.subscribe(pointTopic, 1, 
          &fallback_receive_cloud);
        fallback_enabled = true;
        cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(10);
      }
      else // fallback failed as well. no data available. Abort service call.
      {
        logger.logError("No sensor data available. Aborting.");
        return false;
      }
    } 
    ros::spinOnce();
    r.sleep();
  }

  mutex.lock();
  perceivedObjects = sp.getPerceivedObjects();
  
  // Execution pipeline
  // Each capability provides an enrichment for the
  // returned PerceivedObject

  // initialize threadpool
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  std::auto_ptr<boost::asio::io_service::work> work(
    new boost::asio::io_service::work(ioService));

  // Add worker threads to threadpool
  for(int i = 0; i < numThreads; ++i)
  {
    threadpool.create_thread(
      boost::bind(&boost::asio::io_service::run, &ioService)
      );
  }

  for (int i = 0; i < perceivedObjects.size(); i++) 
  {
    // Initialize Capabilities
    ColorAnalysis ca(perceivedObjects[i]);
    ca.setLowerSThreshold(color_analysis_lower_s);
    ca.setUpperSThreshold(color_analysis_upper_s);
    ca.setLowerVThreshold(color_analysis_lower_v);
    ca.setUpperVThreshold(color_analysis_upper_v);
    suturo_perception_shape_detection::RandomSampleConsensus sd(perceivedObjects[i]);
    suturo_perception_vfh_estimation::VFHEstimation vfhe(perceivedObjects[i]);
    // suturo_perception_3d_capabilities::CuboidMatcherAnnotator cma(perceivedObjects[i]);
    // Init the cuboid matcher with the table coefficients
    suturo_perception_3d_capabilities::CuboidMatcherAnnotator cma(perceivedObjects[i], sp.getTableCoefficients() );

    // post work to threadpool
    ioService.post(boost::bind(&ColorAnalysis::execute, ca));
    ioService.post(boost::bind(&suturo_perception_shape_detection::RandomSampleConsensus::execute, sd));
    ioService.post(boost::bind(&suturo_perception_vfh_estimation::VFHEstimation::execute, vfhe));
    ioService.post(boost::bind(&suturo_perception_3d_capabilities::CuboidMatcherAnnotator::execute, cma));

    // Set an empty label
    perceivedObjects[i].set_c_recognition_label_2d("");

  }
  //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  // wait for thread completion.
  // destroy the work object to wait for all queued tasks to finish
  work.reset();
  ioService.run();
  threadpool.join_all();

  //res.perceivedObjs = *convertPerceivedObjects(&perceivedObjects); // TODO handle images in this method

  //logger.logInfo((boost::format(" Extracted images vector: %s vs. Extracted PointCloud Vector: %s") % perceived_cluster_images.size() % perceivedObjects.size()).str());

  mutex.unlock();

  //boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  //logger.logTime(start, end, "TOTAL");

  logger.logInfo("Shutting down subscriber");
  image_sub.unsubscribe(); // shutdown subscriber, to mitigate funky behavior
  pc_sub.unsubscribe(); // shutdown subscriber, to mitigate funky behavior
  // sync.shutdown();
  //visualizationPublisher.publishMarkers(res.perceivedObjs);

  logger.logInfo("Service call finished. return");
  return true;
}



