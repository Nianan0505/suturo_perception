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
#include <pcl/filters/voxel_grid.h>
#include <publisher_helper.h>

namespace po = boost::program_options;
using namespace boost;
using namespace std;


class RegistrationWithTransformations
{
  public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered_cloud_;
    tf::TransformListener tf_;
    RegistrationWithTransformations(std::string cloud_topic, std::string frame_id_from,
        std::string frame_id_to) : tf_( ros::Duration(20) ), frame_id_from_(frame_id_from), frame_id_to_(frame_id_to), ph_(n_)
    {
      registered_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

      std::cout << "Waiting for first transform" << std::endl;
      tf_.waitForTransform(frame_id_from_, frame_id_to_,
          ros::Time(), ros::Duration(5.0));
      std::cout << "Waiting done" << std::endl;
      ros::Duration d(1); // Fill the TF buffer
      d.sleep();



      sub_ = n_.subscribe (cloud_topic, 1, &RegistrationWithTransformations::cloud_cb, this);
      // pub_ = n_.advertise<sensor_msgs::PointCloud2>("/suturo/registration/registered_cloud",1);
      ph_.advertise<sensor_msgs::PointCloud2>("/suturo/registration/registered_cloud");

    };
  private:
    ros::NodeHandle n_;
    std::string frame_id_to_;
    std::string frame_id_from_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    suturo_perception_ros_utils::PublisherHelper ph_;

    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
    {
      std::cout << "Cloud received @t=" << input->header.stamp;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*input,*cloud_in);
      ros::Time inputTime = input->header.stamp;
      std::string frame_id = input->header.frame_id;

      tf::StampedTransform transform;
      try{
        // TODO make the frames variables
        tf_.lookupTransform(frame_id_to_, frame_id_from_,
            inputTime, transform);
        pcl_ros::transformPointCloud(*cloud_in, *cloud_out, transform);

        std::cout.precision(3);
        std::cout.setf(std::ios::fixed,std::ios::floatfield);
        std::cout << " Transform time " << transform.stamp_.toSec() << std::endl;
        // double yaw, pitch, roll;
        // transform.getBasis().getRPY(roll, pitch, yaw);
        // tf::Quaternion q = transform.getRotation();
        // tf::Vector3 v = transform.getOrigin();
        // std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
        // std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
        //   << q.getZ() << ", " << q.getW() << "]" << std::endl
        //   << "            in RPY [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl;

        // Write transformed pointclouds to disk.
        // Open all of them with pcd_viewer / pcl_viewer to see
        // the combined result
        pcl::PCDWriter writer;
        std::stringstream ss;
        // ss << "/tmp/transformed_pc.pcd" << inputTime << ".pcd";
        // writer.write(ss.str(), *cloud_out);
        // std::cout << "File: " << ss.str() << " written." << std::endl;

        (*registered_cloud_) += (*cloud_out);

        // Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud (registered_cloud_);
        sor.setLeafSize (0.04f, 0.04f, 0.04f);
        sor.filter (*registered_cloud_);
        ph_.publish_pointcloud("/suturo/registration/registered_cloud", registered_cloud_, frame_id_to_);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "registration_w_transformations");
  ros::NodeHandle node;

  std::string cloud_topic   = "";
  std::string frame_id_from = "";
  std::string frame_id_to   = "";

  // "HashMap" for program parameters
  po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("topic,t", po::value<std::string>(&cloud_topic), "The ROS topic this node should listen to")
      ("frame_from,f", po::value<std::string>(&frame_id_from)->required(), "The frame id of the incoming sensor data")
      ("frame_to,r", po::value<std::string>(&frame_id_to)->required(), "The reference frame where the sensor data will be mapped to. This should be /map or any other global reference frame.")
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

  RegistrationWithTransformations r(cloud_topic, frame_id_from, frame_id_to);
  // Spin
  ros::spin ();

  return 0;
};
