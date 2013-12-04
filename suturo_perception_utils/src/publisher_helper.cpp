#include "publisher_helper.h"
#include <string> 
#include <iostream> 
#include "ros/ros.h"

using namespace suturo_perception_helper;

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

// template<class T>
// void PublisherHelper::advertise(std::string topic)
// {
//   if(!isAdvertised(topic))
//   {
//       _topic_to_publisher_map[topic] = _node_handle.advertise<T> (topic, _queue_size);
//       setAdvertised(topic);
//   }
//   else
//   {
//     std::cerr << "Tried to advertise on an already existing topic" << std::endl;
//   }
// }

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

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
