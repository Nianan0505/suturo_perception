#ifndef PUBLISHER_HELPER_H
#define PUBLISHER_HELPER_H

#include <string>
#include <map>
#include "ros/ros.h"
#include <unistd.h>

#define DEFAULT_QUEUE_SIZE 5

namespace suturo_perception_helper
{
  class PublisherHelper
  {
    private:
      std::map<std::string,ros::Publisher> _topic_to_publisher_map;
      std::map<std::string,bool> _is_advertised_map;
      ros::NodeHandle &_node_handle;
      int _queue_size;
    public:
      // Constructor
      PublisherHelper(ros::NodeHandle& nh);

      void setAdvertised(std::string topic);

      // Is the given Topic already advertised? 
      bool isAdvertised(std::string topic);

      // Returns the publisher object for a given Topic. If no publisher is existing, NULL will be returned.
      ros::Publisher *getPublisher(std::string topic);

      // Advertise a topic with Type T
      template<class T>
      void advertise(std::string topic)
      {
        if(!isAdvertised(topic))
        {
          _topic_to_publisher_map[topic] = _node_handle.advertise<T> (topic, _queue_size);
          // std::cout << _topic_to_publisher_map[topic].getTopic() << std::endl;
          setAdvertised(topic);
          // sleep(10);
        }
        else
        {
          ROS_ERROR("Tried to advertise on an already existing topic");
        }
      }

  };
}

#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
