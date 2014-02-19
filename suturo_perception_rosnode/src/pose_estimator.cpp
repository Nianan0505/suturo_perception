/**
 * This node subscribes to /camera/depth_registered/points,
 * looks for a table, and segments the object on the table out
 * Afterwards, the suturo_perception_match_cuboid Library will be used
 * to estimate a Cuboid for every cluster on the table.
 *
 * The node assumes, that the pointcloud topic is already
 * advertised by your RGBD software.
 *
 */
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "suturo_perception.h"
#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace boost;

suturo_perception_lib::SuturoPerception sp;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pc_ptr;

ros::Publisher vis_pub;
ros::Publisher first_t_pub;
ros::Publisher second_t_pub;

void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{

  rgb_pc_ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*inputCloud,*cloud_in);
  // ROS_INFO("Received a new point cloud: size = %lu",cloud_in->points.size());
  sp.setOriginalCloud(cloud_in);
  sp.setCalculateHullVolume(false);
  sp.setEcMinClusterSize(4000);
  sp.processCloudWithProjections(cloud_in);
  pcl::ModelCoefficients::Ptr table_coefficients = sp.getTableCoefficients();

  //std::vector<suturo_perception_lib::PerceivedObject> perceivedObjects;
  std::vector<suturo_perception_lib::PerceivedObject, Eigen::aligned_allocator<suturo_perception_lib::PerceivedObject> > perceivedObjects;

  perceivedObjects = sp.getPerceivedObjects();

  ROS_INFO("Received perceived Objects %lu", perceivedObjects.size());
  for (int i = 0; i < perceivedObjects.size(); i++) {
    rgb_pc_ptr object_cloud = perceivedObjects.at(i).get_pointCloud();
    pcl::PCDWriter writer;
    CuboidMatcher cm;
    cm.setInputCloud(object_cloud);
    cm.setDebug(true);
    cm.setTableCoefficients(table_coefficients);
    cm.setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
    cm.setSaveIntermediateResults(true);
    Cuboid cuboid;
    cm.execute(cuboid);

    // Publish visualization markers
    visualization_msgs::Marker cuboidMarker;
    cuboidMarker.header.frame_id = inputCloud->header.frame_id;
    cuboidMarker.header.stamp = ros::Time();
    cuboidMarker.ns = "suturo_perception";
    cuboidMarker.id = i;
    cuboidMarker.lifetime = ros::Duration(1);
    cuboidMarker.type = visualization_msgs::Marker::CUBE;
    cuboidMarker.action = visualization_msgs::Marker::ADD;
    cuboidMarker.pose.position.x = cuboid.center(0);
    cuboidMarker.pose.position.y = cuboid.center(1);
    cuboidMarker.pose.position.z = cuboid.center(2);
    cuboidMarker.pose.orientation.x = cuboid.orientation.x();
    cuboidMarker.pose.orientation.y = cuboid.orientation.y();
    cuboidMarker.pose.orientation.z = cuboid.orientation.z();
    cuboidMarker.pose.orientation.w = cuboid.orientation.w();
    cuboidMarker.scale.x = cuboid.length1;
    cuboidMarker.scale.y = cuboid.length2;
    cuboidMarker.scale.z = cuboid.length3;
    cuboidMarker.color.a = 1.0;
    cuboidMarker.color.r = 1.0;
    cuboidMarker.color.g = 0.0;
    cuboidMarker.color.b = 0.0;
    vis_pub.publish(cuboidMarker);

    if(cm.getIntermediateClouds().size()>=1)
    {
      sensor_msgs::PointCloud2 pub_message;
      pcl::toROSMsg(*cm.getIntermediateClouds().at(0), pub_message );
      pub_message.header.frame_id = inputCloud->header.frame_id;
      first_t_pub.publish(pub_message);
    }

    if(cm.getIntermediateClouds().size()>=2)
    {
      sensor_msgs::PointCloud2 pub_message;
      pcl::toROSMsg(*cm.getIntermediateClouds().at(1), pub_message );
      pub_message.header.frame_id = inputCloud->header.frame_id;
      second_t_pub.publish(pub_message);
    }
// std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getIntermediateClouds(){ 
  }

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimator");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  std::string pointcloud_topic="";

  // "HashMap" for program parameters
  po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("topic,t", po::value<std::string>(&pointcloud_topic), "The ROS topic this node should listen to")
    ;

    po::positional_options_description p;
    po::store(po::command_line_parser(argc, argv).
    options(desc).positional(p).run(), vm); 

    if (vm.count("help")) {
      cout << "Usage: pose_estimator [-t topic]" << endl << endl;
      cout << desc << "\n";
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

  }
  catch(std::exception& e)
  {
		cout << "Usage: pose_estimator [-t topic]" << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

  if(pointcloud_topic.empty())
      pointcloud_topic = "/camera/depth_registered/points";

  vis_pub = nh.advertise<visualization_msgs::Marker>("/suturo/pose_markers", 0);  
  first_t_pub = nh.advertise<sensor_msgs::PointCloud2>("/suturo/pose/first_transform", 1000); 
  second_t_pub = nh.advertise<sensor_msgs::PointCloud2>("/suturo/pose/second_transform", 1000); 

  std::cout << "Subscribing to " << pointcloud_topic << std::endl;
  sub = nh.subscribe(pointcloud_topic, 1, 
      &receive_cloud);

  ros::Rate r(20); // 20 hz
  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
