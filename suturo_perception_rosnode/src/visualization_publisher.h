#ifndef VISUALIZATION_PUBLISHER_H
#define VISUALIZATION_PUBLISHER_H

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

#include "suturo_perception_msgs/PerceivedObject.h"
#include "suturo_perception_utils.h"
using namespace suturo_perception_utils;
/**
 * Class to manage visualization markers for rviz
 */
class VisualizationPublisher
{
  private:
    ros::NodeHandle nh;
    ros::Publisher vis_pub;
    ros::Publisher cuboid_pub;
    Logger logger;

    int maxMarkerId, maxCuboidMarkerId;
    std::string frameId;
  public:
    VisualizationPublisher(){maxMarkerId = 0; maxCuboidMarkerId = 0;};
    VisualizationPublisher(ros::NodeHandle& n, std::string fi);
    void publishMarkers(std::vector<suturo_perception_msgs::PerceivedObject> objs);
    void publishCuboids(std::vector<suturo_perception_msgs::PerceivedObject> objs);
};

#endif
