#include "ros/ros.h"
#include <boost/signals2/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <suturo_perception_rosnode/SuturoPerceptionConfig.h>
#include <visualization_msgs/Marker.h>

#include "suturo_perception.h"
#include "perceived_object.h"
#include "point.h"
#include "suturo_perception_msgs/GetClusters.h"

/*
 * Callback for the dynamic reconfigure service
 */
void reconfigureCallback(suturo_perception_rosnode::SuturoPerceptionConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %i",
           config.int_param);
  
  // do nothing for now
}

class SuturoDummyPerceptionROSNode
{
public:
  /*
   * Constructor
   */
  SuturoDummyPerceptionROSNode(ros::NodeHandle& n, std::string fi) : 
    nh(n), 
    frameId(fi)
  {
    clusterService = nh.advertiseService("GetClusters", 
      &SuturoDummyPerceptionROSNode::getClusters, this);
    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    objectID = 0;

    // Fill the dummy data vector
    suturo_perception_msgs::PerceivedObject msgObj;
    msgObj.c_id = 0;
    msgObj.c_volume = 23;
    msgObj.c_centroid.x = 1;
    msgObj.c_centroid.y = 2;
    msgObj.c_centroid.z = 3;
    msgObj.frame_id = frameId;
    perceivedObjects.push_back(msgObj);

    suturo_perception_msgs::PerceivedObject msgObj2;
    msgObj2.c_id = 0;
    msgObj2.c_volume = 42;
    msgObj2.c_centroid.x = 0.5f;
    msgObj2.c_centroid.y = 0.5f;
    msgObj2.c_centroid.z = 0.6f;
    msgObj2.frame_id = frameId;

    perceivedObjects.push_back(msgObj2);
  }

  /*
   * Implementation of the GetClusters Service.
   *
   * This method will simply return a list of pre-defined
   * objects.
   */
  bool getClusters(suturo_perception_msgs::GetClusters::Request &req,
    suturo_perception_msgs::GetClusters::Response &res)
  {
    // Push all stored fake Objects into the response
    for (std::vector<suturo_perception_msgs::PerceivedObject>::iterator it = perceivedObjects.begin(); it != perceivedObjects.end (); ++it)
    {
      (*it).c_id = objectID;
      res.perceivedObjs.push_back(*it); 
      objectID++;
    }

    publishVisualizationMarkers(res.perceivedObjs);
    // objectID++;
    
    return true;
  }

private:
  ros::NodeHandle nh;
  boost::signals2::mutex mutex;
  std::vector<suturo_perception_msgs::PerceivedObject> perceivedObjects;
  // ID counter for the perceived objects
  int objectID;
  ros::ServiceServer clusterService;
  ros::Publisher vis_pub;
  int maxMarkerId;
  // std::string pointTopic;
  std::string frameId;
  
  void publishVisualizationMarkers(std::vector<suturo_perception_msgs::PerceivedObject> objs)
  {
    ROS_INFO("Publishing centroid visualization markers");
    int markerId = 0;
    std::vector<suturo_perception_msgs::PerceivedObject>::iterator it = objs.begin();
    for (std::vector<suturo_perception_msgs::PerceivedObject>::iterator it = objs.begin(); 
         it != objs.end (); ++it)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frameId;
      marker.header.stamp = ros::Time();
      marker.ns = "suturo_perception";
      marker.id = markerId;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = it->c_centroid.x;
      marker.pose.position.y = it->c_centroid.y;
      marker.pose.position.z = it->c_centroid.z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      markerId++;
      maxMarkerId++;
      vis_pub.publish(marker);
    }
    // remove markers that have not been updated
    for(int i = markerId; i <= maxMarkerId; ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frameId;
      marker.header.stamp = ros::Time();
      marker.ns = "suturo_perception";
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
      vis_pub.publish(marker);
    }
    maxMarkerId = markerId;
  }
};


int main (int argc, char** argv)
{
  ros::init(argc, argv, "suturo_perception");
  ros::NodeHandle nh;

  // get parameters
  std::string frameId;

  if(ros::param::get("/suturo_perception/frame_id", frameId));
  else frameId = "camera_rgb_optical_frame";
  
  SuturoDummyPerceptionROSNode spr(nh, frameId);  

  // Initialize dynamic reconfigure
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig> srv;
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig>::CallbackType f;
  f = boost::bind(&reconfigureCallback, _1, _2);
  srv.setCallback(f);

  ROS_INFO("suturo_dummy_perception READY");
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return (0);
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
