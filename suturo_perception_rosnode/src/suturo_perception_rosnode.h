#include "ros/ros.h"
#include <boost/signals2/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <dynamic_reconfigure/server.h>
#include <suturo_perception_rosnode/SuturoPerceptionConfig.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>

#include "suturo_perception.h"
#include "visualization_publisher.h"
#include "suturo_perception_2d_capabilities/roi_publisher.h"
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

using namespace suturo_perception_ros_utils;
using namespace suturo_perception_utils;
using namespace suturo_perception_color_analysis;

class SuturoPerceptionROSNode
{
public:
  SuturoPerceptionROSNode(ros::NodeHandle& n, std::string pt, std::string fi, std::string rd);
  void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
  void receive_image_and_cloud(const sensor_msgs::ImageConstPtr& inputImage, const sensor_msgs::PointCloud2ConstPtr& inputCloud);
  bool getClusters(suturo_perception_msgs::GetClusters::Request &req,
    suturo_perception_msgs::GetClusters::Response &res);
  void reconfigureCallback(suturo_perception_rosnode::SuturoPerceptionConfig &config, uint32_t level);

private:
  // Declare the names of the used Topics
  static const std::string TABLE_PLANE_TOPIC;
  static const std::string ALL_OBJECTS_ON_PLANE_TOPIC;
  static const std::string COLLISION_CLOUD_TOPIC;
  static const std::string IMAGE_PREFIX_TOPIC;
  static const std::string CROPPED_IMAGE_PREFIX_TOPIC;
  static const std::string HISTOGRAM_PREFIX_TOPIC;

  bool processing; // processing flag
  ObjectMatcher object_matcher_;
  suturo_perception_lib::SuturoPerception sp;
  std::vector<suturo_perception_lib::PerceivedObject> perceivedObjects;
  ros::NodeHandle nh;
  boost::signals2::mutex mutex;
  // ID counter for the perceived objects
  int objectID;
  // services
  ros::ServiceServer clusterService;
  ros::ServiceClient is_edible_service;
  ros::ServiceClient is_edible_service_next;
  ros::ServiceClient is_edible_service_finish;
  
  std::string pointTopic;
  std::string frameId;
  std::string recognitionDir;
  // Helper Class for Publishing Business
  PublisherHelper ph;
  // dynamic reconfigure
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig> reconfSrv;
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig>::CallbackType reconfCb;

  VisualizationPublisher visualizationPublisher;
  Logger logger;

  // capabilities


  /*
   * Convert suturo_perception_lib::PerceivedObject list to suturo_perception_msgs:PerceivedObject list
   */
  std::vector<suturo_perception_msgs::PerceivedObject> 
    *convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject> *objects);
};

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
