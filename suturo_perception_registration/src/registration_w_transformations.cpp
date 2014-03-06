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
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <pcl/io/pcd_io.h>

namespace po = boost::program_options;
using namespace boost;
using namespace std;

// class RegistrationWithTransformations
// {
//   public:
//     RegistrationWithTransformations(std::string cloud_topic) : tf_(), target_frame_("base_link")
//     {
//       pc_sub_.subscribe (n_, cloud_topic, 10);
//       tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pc_sub_, tf_, target_frame_, 10);
//       tf_filter_->registerCallback( boost::bind(&RegistrationWithTransformations::msgCallback, this, _1) );
//     };
//   private:
//     message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
//     tf::TransformListener tf_;
//     tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
//     ros::NodeHandle n_;
//     std::string target_frame_;
// 
//   // Callback to register with tf::MessageFilter to be called when transforms are available
//   void msgCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& point_ptr) 
//   {
//     try 
//     {
//       std::cout << "foo";
//     }
//     catch (tf::TransformException &ex) 
//     {
//       printf ("Failure %s\n", ex.what()); //Print exception which was caught
//     }
//   };
// 
// };

class RegistrationWithTransformations
{
  public:
    tf::TransformListener tf_;
    RegistrationWithTransformations(std::string cloud_topic) : tf_(ros::Duration(20) ), target_frame_("base_link")
    {
      std::cout << "Waiting for first transform" << std::endl;
      tf_.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
          ros::Time(), ros::Duration(5.0));
      std::cout << "Waiting done" << std::endl;
      ros::Duration d(1); // Fill the TF buffer
      d.sleep();


      sub_ = n_.subscribe (cloud_topic, 1, &RegistrationWithTransformations::cloud_cb, this);

    };
  private:
    ros::NodeHandle n_;
    std::string target_frame_;
    ros::Subscriber sub_;

    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
    {
      ROS_INFO("FOO ");
      std::cout << "Cloud received @t=" << input->header.stamp;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*input,*cloud_in);
      ros::Time inputTime = input->header.stamp;

      tf::StampedTransform transform;
      try{
        // TODO make the frames variables
        tf_.lookupTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
            inputTime, transform);
        pcl_ros::transformPointCloud(*cloud_in, *cloud_out, transform);

        pcl::PCDWriter writer;
        std::stringstream ss;
        ss << "/tmp/transformed_pc.pcd" << inputTime << ".pcd";
        writer.write(ss.str(), *cloud_out);
        std::cout << "File: " << ss.str() << " written." << std::endl;
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
    }
};
// 
// void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
//   ROS_INFO("FOO");
//   // std::cout << "Cloud received @t=" << input->header.stamp;
//   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
//   // pcl::fromROSMsg(*input,*cloud_in);
//   // ros::Time inputTime = input->header.stamp;
// 
//   tf::TransformListener listener(ros::Duration(15));
//   tf::StampedTransform transform;
// 
//   // ros::Time nao = ros::Time::now();
//   // std::cout << "FIGHT: " << nao << " vs . " << inputTime << std::endl;
//   try{
//     // TODO make the frames variables
//     std::cout << "Waiting for transform";
//     listener.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
//                               ros::Time::now(), ros::Duration(10));
//     // wtf.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
//                               // ros::Time::now(), ros::Duration(10));
//     listener.lookupTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
//                               ros::Time::now(), transform);
//   }
//   catch (tf::TransformException ex){
//     ROS_ERROR("%s",ex.what());
//   }
//   std::cout << "CB end" << std::endl;
// }

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


  // tf::TransformListener wtf(ros::Duration(15)); 

  // std::cout << "Waiting for first transformation" << std::endl;
  // wtf.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
  //                             ros::Time::now(), ros::Duration(10));
  // wtf.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
  //                             ros::Time::now(), ros::Duration(10));
  // wtf.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
  //                             ros::Time::now(), ros::Duration(10));
  // wtf.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
  //                             ros::Time::now(), ros::Duration(10));
  // wtf.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
  //                             ros::Time::now(), ros::Duration(10));
  // wtf.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
  //                             ros::Time::now(), ros::Duration(10));
  // wtf.waitForTransform("/head_mount_kinect_rgb_optical_frame", "/base_link",
  //                             ros::Time::now(), ros::Duration(10));
  // std::cout << "Fuck yeah at"<< ros::Time::now() << std::endl;

  // // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = node.subscribe (cloud_topic, 1, cloud_cb);
  // ROS_INFO("Subscribed to %s", cloud_topic.c_str() );
  
  RegistrationWithTransformations r(cloud_topic);
  // Spin
  ros::spin ();

  return 0;
};
