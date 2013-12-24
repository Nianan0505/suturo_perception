/**
 * This node subscribes to /camera/depth_registered/points,
 * looks for a table, and segments the object on the table out
 * If objects have been found, they will be written to the
 * Harddrive ( /tmp ).
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

bool cloud_processed = false;
suturo_perception_lib::SuturoPerception sp;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pc_ptr;

void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{

  rgb_pc_ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*inputCloud,*cloud_in);
  ROS_INFO("Received a new point cloud: size = %lu",cloud_in->points.size());
  sp.setOriginalCloud(cloud_in);
  sp.processCloudWithProjections(cloud_in);

  std::vector<suturo_perception_lib::PerceivedObject> perceivedObjects;
  perceivedObjects = sp.getPerceivedObjects();

  for (int i = 0; i < perceivedObjects.size(); i++) {
    rgb_pc_ptr object_cloud = perceivedObjects.at(i).get_pointCloud();
    pcl::PCDWriter writer;

		std::stringstream ss;
		ss << "/tmp/segment_object_" << i << ".pcd";
		writer.write(ss.str(), *object_cloud);
    std::cout << "File: " << ss.str() << " written." << std::endl;
  }

  cloud_processed = true; // Fetched cloud and processed it. Shutdown the node
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "segment_objects");
  ros::NodeHandle nh;
  ros::Subscriber sub;

  sub = nh.subscribe("/camera/depth_registered/points", 1, 
      &receive_cloud);

  ros::Rate r(20); // 20 hz
  while(!cloud_processed && ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
