#include "ros/ros.h"
#include <boost/signals2/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <dynamic_reconfigure/server.h>
#include <suturo_perception_rosnode/SuturoPerceptionConfig.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>

#include "suturo_perception.h"
#include "perceived_object.h"
#include "point.h"
#include "suturo_perception_msgs/GetClusters.h"
#include "suturo_perception_msgs/PrologQuery.h"
#include "suturo_perception_msgs/PrologNextSolution.h"
#include "suturo_perception_msgs/PrologFinish.h"
#include "object_matcher.h"  // Include 2d Object recognizer
#include "publisher_helper.h"

using namespace suturo_perception_utils;

class SuturoPerceptionROSNode
{
public:
  SuturoPerceptionROSNode(ros::NodeHandle& n, std::string pt, std::string fi, std::string rd);
  void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
  bool getClusters(suturo_perception_msgs::GetClusters::Request &req,
    suturo_perception_msgs::GetClusters::Response &res);
  void reconfigureCallback(suturo_perception_rosnode::SuturoPerceptionConfig &config, uint32_t level);

private:
  // Declare the name of the used Topics
  static const std::string TABLE_PLANE_TOPIC;
  static const std::string ALL_OBJECTS_ON_PLANE_TOPIC;
  static const std::string COLLISION_CLOUD_TOPIC;

  bool processing; // processing flag
  ObjectMatcher object_matcher_;
  suturo_perception_lib::SuturoPerception sp;
  std::vector<suturo_perception_lib::PerceivedObject> perceivedObjects;
  ros::NodeHandle nh;
  boost::signals2::mutex mutex;
  // ID counter for the perceived objects
  int objectID;
  ros::ServiceServer clusterService;
  ros::Publisher vis_pub;
  // ros::Publisher table_plane_pub;
  // ros::Publisher objects_on_plane_pub;
  // ros::Publisher collision_cloud_pub;
  ros::ServiceClient is_edible_service;
  ros::ServiceClient is_edible_service_next;
  ros::ServiceClient is_edible_service_finish;
  int maxMarkerId;
  std::string pointTopic;
  std::string frameId;
  std::string recognitionDir;
  // Helper Class for Publishing Business
  PublisherHelper ph;
  // dynamic reconfigure
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig> reconfSrv;
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig>::CallbackType reconfCb;

  /*
   * Convert suturo_perception_lib::PerceivedObject list to suturo_perception_msgs:PerceivedObject list
   */
  std::vector<suturo_perception_msgs::PerceivedObject> 
    *convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject> *objects);

  void publishVisualizationMarkers(std::vector<suturo_perception_msgs::PerceivedObject> objs);
};

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
