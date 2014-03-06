/******
 * PointCloud Registration with known transformations
 * from a localized robot
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <boost/program_options.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

namespace po = boost::program_options;
using namespace boost;
using namespace std;
// class RegistrationWithTransformations
// {
//   public:
//     RegistrationWithTransformations() : tf_(), target_frame_("base_link")
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout << "Cloud received @t=" << input->header.stamp;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*input,*cloud_in);
  ros::Time inputTime = input->header.stamp;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    // TODO make the frames variables
    std::cout << "Waiting for transform";
    listener.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
                              inputTime, ros::Duration(3.0));
    listener.lookupTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
                             inputTime, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  std::cout << "CB end" << std::endl;

  // sensor_msgs::PointCloud2 output;
  // // Publish the data
  // pub.publish (output);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "registration_w_transformations");
  ros::NodeHandle node;

  std::string cloud_topic="";

  // "HashMap" for program parameters
  po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("topic,t", po::value<std::string>(&cloud_topic), "The ROS topic this node should listen to")
    ;

    po::positional_options_description p;
    po::store(po::command_line_parser(argc, argv).
    options(desc).positional(p).run(), vm); 

    if (vm.count("help")) {
      cout << "Usage: registration_w_transformations [-t topic]" << endl << endl;
      cout << desc << "\n";
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

  }
  catch(std::exception& e)
  {
		cout << "Usage: registration_w_transformations [-t topic]" << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

  if(cloud_topic.empty())
      cloud_topic = "/camera/depth_registered/points";


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = node.subscribe (cloud_topic, 1, cloud_cb);
  ROS_INFO("Subscribed to %s", cloud_topic.c_str() );
  
  // Spin
  ros::spin ();

  // ros::service::waitForService("spawn");
  // ros::ServiceClient add_turtle =
  //      node.serviceClient<turtlesim::Spawn>("spawn");
  // turtlesim::Spawn srv;
  // add_turtle.call(srv);

  // ros::Publisher turtle_vel =
  //      node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

  // tf::TransformListener listener;

  // ros::Rate rate(10.0);
  // while (node.ok()){
  //   tf::StampedTransform transform;
  //   try{
  //     listener.lookupTransform("/turtle2", "/turtle1",
  //                              ros::Time(0), transform);
  //   }
  //   catch (tf::TransformException ex){
  //     ROS_ERROR("%s",ex.what());
  //   }

  //   turtlesim::Velocity vel_msg;
  //   vel_msg.angular = 4 * atan2(transform.getOrigin().y(),
  //                               transform.getOrigin().x());
  //   vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
  //                                pow(transform.getOrigin().y(), 2));
  //   turtle_vel.publish(vel_msg);

  //   rate.sleep();
  // }
  return 0;
};
