#define DISABLE_LOGGING 1

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
#include "suturo_perception_3d_capabilities/cuboid_matcher_annotator.h"

using namespace suturo_perception_ros_utils;
using namespace suturo_perception_utils;
using namespace suturo_perception_color_analysis;

class SuturoPerceptionKnowledgeROSNode
{
public:
  SuturoPerceptionKnowledgeROSNode(ros::NodeHandle& n, std::string rd);
  std::vector<suturo_perception_msgs::PerceivedObject> receive_image_and_cloud(const sensor_msgs::ImageConstPtr& inputImage, const sensor_msgs::PointCloud2ConstPtr& inputCloud);
  void reconfigureCallback(suturo_perception_rosnode::SuturoPerceptionConfig &config, uint32_t level);

private:
  ros::Subscriber sub_cloud; // fallback subscriber
  ObjectMatcher object_matcher_;
  suturo_perception_lib::SuturoPerception sp;
  //std::vector<suturo_perception_lib::PerceivedObject> perceivedObjects;
  std::vector<suturo_perception_lib::PerceivedObject, Eigen::aligned_allocator<suturo_perception_lib::PerceivedObject> > perceivedObjects;
  ros::NodeHandle nh;
  boost::signals2::mutex mutex;
  // ID counter for the perceived objects
  int objectID;
  // services
  std::string recognitionDir;
  // dynamic reconfigure
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig> reconfSrv;
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig>::CallbackType reconfCb;
  // color analysis config
  double color_analysis_lower_s;
  double color_analysis_upper_s;
  double color_analysis_lower_v;
  double color_analysis_upper_v;
  
  Logger logger;

  int numThreads;
  boost::asio::io_service ioService;
  boost::thread_group threadpool;

  /*
   * Convert suturo_perception_lib::PerceivedObject list to suturo_perception_msgs:PerceivedObject list
   */
  std::vector<suturo_perception_msgs::PerceivedObject> 
    *convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject, Eigen::aligned_allocator<suturo_perception_lib::PerceivedObject> > *objects);
    //*convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject> *objects);
};

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
