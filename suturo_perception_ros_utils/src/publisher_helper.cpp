#include "publisher_helper.h"

using namespace suturo_perception_ros_utils;

PublisherHelper::PublisherHelper(ros::NodeHandle &nh) : _node_handle(nh), _queue_size(DEFAULT_QUEUE_SIZE)
{
}
ros::Publisher* PublisherHelper::getPublisher(std::string topic)
{
  if(isAdvertised(topic))
  {
    return &_topic_to_publisher_map[topic];
  }
  else
  {
    return NULL;
  }

}

bool PublisherHelper::isAdvertised(std::string topic)
{
  if(_is_advertised_map.find(topic) == _is_advertised_map.end())
    return false; // Element has not been found -> Topic not advertised yet

  return _is_advertised_map[topic];
}

void PublisherHelper::setAdvertised(std::string topic)
{
  _is_advertised_map[topic] = true;
}

void PublisherHelper::publish_pointcloud(ros::Publisher &publisher, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_publish, std::string frame)
{

  if(cloud_to_publish != NULL)
  {
    sensor_msgs::PointCloud2 pub_message;
    pcl::toROSMsg(*cloud_to_publish, pub_message );
    pub_message.header.frame_id = frame;
    publisher.publish(pub_message);
  }
  else
  {
    ROS_ERROR("publish_pointcloud : Input cloud is NULL");
  }
}

bool PublisherHelper::publish_pointcloud(std::string topic, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_publish, std::string frame)
{
  if( !isAdvertised(topic))
  {
    ROS_ERROR("publish_pointcloud : Given topic is not advertised");
    return false;
  }

  if(cloud_to_publish != NULL)
  {
    sensor_msgs::PointCloud2 pub_message;
    pcl::toROSMsg(*cloud_to_publish, pub_message );
    pub_message.header.frame_id = frame;
    getPublisher(topic)->publish(pub_message);
    return true;
  }
  else
  {
    ROS_ERROR("publish_pointcloud : Input cloud is NULL");
    return false;
  }
}



// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
