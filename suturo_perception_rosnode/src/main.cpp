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

class SuturoPerceptionROSNode
{
public:
  /*
   * Constructor
   */
  SuturoPerceptionROSNode(ros::NodeHandle& n) : nh(n)
  {
    clusterService = nh.advertiseService("GetClusters", 
      &SuturoPerceptionROSNode::getClusters, this);
    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    objectID = 0;
    //processing = false;
  }

   /*
    * Receive callback for the /camera/depth_registered/points subscription
    */
   void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
   {
    // process only one cloud
    ROS_INFO("Receiving cloud");
    if(processing)
    {
      ROS_INFO("processing...");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*inputCloud,*cloud_in);

      sp.processCloud(cloud_in); 
      processing = false;

      ROS_INFO("Received a new point cloud: size = %lu",cloud_in->points.size());
    }
  }

  /*
   * Implementation of the GetClusters Service.
   *
   * This method will subscribe to the /camera/depth_registered/points topic, 
   * wait for the processing of a single point cloud, and return the result from
   * the calulations as a list of PerceivedObjects.
   */
  bool getClusters(suturo_perception_msgs::GetClusters::Request &req,
    suturo_perception_msgs::GetClusters::Response &res)
  {
    ros::Subscriber sub;
    processing = true;

    // signal failed call, if request string does not match
    if (req.s.compare("get") != 0)
    {
      return false;
    }

    // Subscribe to the depth information topic
    sub = nh.subscribe("/camera/depth_registered/points", 1, 
      &SuturoPerceptionROSNode::receive_cloud, this);

    ROS_INFO("Waiting for processed cloud");
    ros::Rate r(20); // 20 hz
    while(processing)
    {
      ros::spinOnce();
      r.sleep();
    }
    ROS_INFO("Cloud processed. Lock buffer and return the results");

    mutex.lock();
    perceivedObjects = sp.getPerceivedObjects();
    res.perceivedObjs = *convertPerceivedObjects(&perceivedObjects);
    mutex.unlock();

    ROS_INFO("Publishing centroid visualization markers");
    int markerId = 0;
    std::vector<suturo_perception_msgs::PerceivedObject>::iterator it = res.perceivedObjs.begin();
    for (std::vector<suturo_perception_msgs::PerceivedObject>::iterator it = res.perceivedObjs.begin(); 
         it != res.perceivedObjs.end (); ++it)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "camera_rgb_optical_frame";
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
      vis_pub.publish(marker);
    }
    return true;
  }

private:
  bool processing; // processing flag
  suturo_perception_lib::SuturoPerception sp;
  std::vector<suturo_perception_lib::PerceivedObject> perceivedObjects;
  ros::NodeHandle nh;
  boost::signals2::mutex mutex;
  // ID counter for the perceived objects
  int objectID;
  ros::ServiceServer clusterService;
  ros::Publisher vis_pub;
  
  /*
   * Convert suturo_perception_lib::PerceivedObject list to suturo_perception_msgs:PerceivedObject list
   */
  std::vector<suturo_perception_msgs::PerceivedObject> *convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject> *objects)
  {
    std::vector<suturo_perception_msgs::PerceivedObject> *result = new std::vector<suturo_perception_msgs::PerceivedObject>();
    for (std::vector<suturo_perception_lib::PerceivedObject>::iterator it = objects->begin(); it != objects->end(); ++it)
    {
      suturo_perception_msgs::PerceivedObject *msgObj = new suturo_perception_msgs::PerceivedObject();
      msgObj->c_id = it->c_id;
      msgObj->c_volume = it->c_volume;
      msgObj->c_centroid.x = it->c_centroid.x;
      msgObj->c_centroid.y = it->c_centroid.y;
      msgObj->c_centroid.z = it->c_centroid.z;
      result->push_back(*msgObj);
    }
    return result;
  }
};


int main (int argc, char** argv)
{
  ros::init(argc, argv, "suturo_perception");
  ros::NodeHandle nh;
  SuturoPerceptionROSNode spr(nh);

  // Initialize dynamic reconfigure
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig> srv;
  dynamic_reconfigure::Server<suturo_perception_rosnode::SuturoPerceptionConfig>::CallbackType f;
  f = boost::bind(&reconfigureCallback, _1, _2);
  srv.setCallback(f);

  ROS_INFO("suturo_perception READY");
    //sp.sayHi();
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return (0);
}
