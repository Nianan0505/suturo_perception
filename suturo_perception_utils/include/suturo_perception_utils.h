#ifndef SUTURO_PERCEPTION_UTILS_H
#define SUTURO_PERCEPTION_UTILS_H

#include <string>
#include "ros/ros.h"


namespace suturo_perception_utils
{
  class SuturoPerceptionUtils
  {
    private:
      ros::NodeHandle &_node_handle;

    public:
      /**
       * log methods for different levels
       */
      void logDebug  (std::string s);
      void logInfo   (std::string s);
      void logWarning(std::string s);
      void logError  (std::string s);
     
  };
}
#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
