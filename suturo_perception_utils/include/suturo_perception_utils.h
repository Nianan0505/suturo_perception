#ifndef SUTURO_PERCEPTION_UTILS_H
#define SUTURO_PERCEPTION_UTILS_H

#include <iostream>
#include <string>

#ifndef HAVE_NO_ROS
#include "ros/ros.h"
#endif



namespace suturo_perception_utils
{
  class SuturoPerceptionUtils
  {
    private:
      std::string module;
      enum level {DEBUG, INFO, WARN, ERROR};
      void log(level, std::string s);

    public:
      // Constructor to instantiate a logger for a module
      SuturoPerceptionUtils(std::string moduleName);
      /**
       * log methods for different levels
       */
      void logDebug(std::string s);
      void logInfo (std::string s);
      void logWarn (std::string s);
      void logError(std::string s);
     
  };
}
#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
