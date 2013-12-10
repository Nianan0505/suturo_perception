#include "visualization_publisher.h"

/*
 * Constructor. Advertise the marker topic
 */
VisualizationPublisher::VisualizationPublisher(ros::NodeHandle& n, std::string fi) : 
  nh(n),
  frameId(fi)
{
  maxMarkerId = 0;
  vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);  
  Logger("perception_rosnode");
}

void VisualizationPublisher::publishMarkers(std::vector<suturo_perception_msgs::PerceivedObject> objs)
{
  logger.logInfo("Publishing visualization markers");
  logger.logDebug("Publishing centroid visualization markers");
  int markerId = 0;
  std::vector<suturo_perception_msgs::PerceivedObject>::iterator it = objs.begin();
  for (std::vector<suturo_perception_msgs::PerceivedObject>::iterator it = objs.begin(); 
       it != objs.end (); ++it)
  {
    logger.logDebug("Publishing centroid marker");

    visualization_msgs::Marker centroidMarker;
    centroidMarker.header.frame_id = frameId;
    centroidMarker.header.stamp = ros::Time();
    centroidMarker.ns = "suturo_perception";
    centroidMarker.id = markerId;
    centroidMarker.type = visualization_msgs::Marker::SPHERE;
    centroidMarker.action = visualization_msgs::Marker::ADD;
    centroidMarker.pose.position.x = it->c_centroid.x;
    centroidMarker.pose.position.y = it->c_centroid.y;
    centroidMarker.pose.position.z = it->c_centroid.z;
    centroidMarker.pose.orientation.x = 0.0;
    centroidMarker.pose.orientation.y = 0.0;
    centroidMarker.pose.orientation.z = 0.0;
    centroidMarker.pose.orientation.w = 0.0;
    centroidMarker.scale.x = 0.1;
    centroidMarker.scale.y = 0.1;
    centroidMarker.scale.z = 0.1;
    centroidMarker.color.a = 1.0;
    centroidMarker.color.r = 0.0;
    centroidMarker.color.g = 1.0;
    centroidMarker.color.b = 0.0;
    markerId++;
    maxMarkerId++;
    vis_pub.publish(centroidMarker);

    logger.logDebug("Publishing text marker");
    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = frameId;
    textMarker.header.stamp = ros::Time();
    textMarker.ns = "suturo_perception";
    textMarker.id = markerId + 1000;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::ADD;
    textMarker.pose.position.x = it->c_centroid.x - 0.0;
    textMarker.pose.position.y = it->c_centroid.y - 0.1;
    textMarker.pose.position.z = it->c_centroid.z - 0.4;
    textMarker.scale.x = 0.025;
    textMarker.scale.y = 0.025;
    textMarker.scale.z = 0.025;
    textMarker.color.a = 1.0;
    textMarker.color.r = 1.0;
    textMarker.color.g = 0.0;
    textMarker.color.b = 0.0;
    std::stringstream ss;
    ss << "Volume: " << it->c_volume << "\n" 
       << "rgb: " << (int)it->c_color_average_r << "." << 
          (int)it->c_color_average_g << "." << (int)it->c_color_average_b << "\n"
       << "Shape: ";
    switch(it->c_shape)
    {
      case 0: ss << "None"; break;
      case 1: ss << "Box"; break;
      case 2: ss << "Cylinder"; break;
      case 3: ss << "Sphere"; break;
      default: ss << "None"; break;
    }
    ss << "\nLabel: " << it->recognition_label_2d;
    textMarker.text = ss.str();
    vis_pub.publish(textMarker);
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
    // also remove text marker
    marker.id = i + 1001;
    vis_pub.publish(marker);
  }
  maxMarkerId = markerId;
}
