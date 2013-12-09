#include "ros/ros.h"
#include "suturo_perception_rosnode.h"


int main (int argc, char** argv)
{
  ros::init(argc, argv, "suturo_perception");
  ros::NodeHandle nh;
  std::string recognitionDir = "";
  if(argc > 0 && strcmp(argv[1], "_") != 0)
  {
    recognitionDir = argv[1];
    ROS_INFO ("Dir for 2D recognition data set: %s", recognitionDir.c_str());
  }
  else ROS_INFO ("Dir for 2D recognition data not set. 2D Recognition will be disabled.");

  // get parameters
  std::string pointTopic;
  std::string frameId;

  // ros strangeness strikes again. don't try to && these!
  if(ros::param::get("/suturo_perception/point_topic", pointTopic)) ROS_INFO("Using parameters from Parameter Server");
  else { pointTopic = "/camera/depth_registered/points"; ROS_INFO("Using default parameters");}
  if(ros::param::get("/suturo_perception/frame_id", frameId));
  else frameId = "camera_rgb_optical_frame";
  
  SuturoPerceptionROSNode spr(nh, pointTopic, frameId, recognitionDir);

  ROS_INFO("                    _____ ");
  ROS_INFO("                   |     | ");
  ROS_INFO("                   | | | | ");
  ROS_INFO("                   |_____| ");
  ROS_INFO("             ____ ___|_|___ ____ ");
  ROS_INFO("            ()___)         ()___) ");
  ROS_INFO("            // /|           |\\ \\\\ ");
  ROS_INFO("           // / |           | \\ \\\\ ");
  ROS_INFO("          (___) |___________| (___) ");
  ROS_INFO("          (___)   (_______)   (___) ");
  ROS_INFO("          (___)     (___)     (___) ");
  ROS_INFO("          (___)      |_|      (___) ");
  ROS_INFO("          (___)  ___/___\\___   | | ");
  ROS_INFO("           | |  |           |  | | ");
  ROS_INFO("           | |  |___________| /___\\ ");
  ROS_INFO("          /___\\  |||     ||| //   \\\\ ");
  ROS_INFO("         //   \\\\ |||     ||| \\\\   // ");
  ROS_INFO("         \\\\   // |||     |||  \\\\ // ");
  ROS_INFO("          \\\\ // ()__)   (__() ");
  ROS_INFO("                ///       \\\\\\ ");
  ROS_INFO("               ///         \\\\\\ ");
  ROS_INFO("             _///___     ___\\\\\\_ ");
  ROS_INFO("            |_______|   |_______| ");

  ROS_INFO("           suturo_perception READY");
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return (0);
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
