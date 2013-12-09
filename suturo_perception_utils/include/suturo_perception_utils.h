#ifndef SUTURO_PERCEPTION_UTILS_H
#define SUTURO_PERCEPTION_UTILS_H

#include <iostream>
#include <string>

#ifndef HAVE_NO_ROS
#include "ros/ros.h"
#endif



namespace suturo_perception_utils
{
  class Logger
  {
    private:
      std::string module;
      enum level {DEBUG, INFO, WARN, ERROR};
      void log(level, std::string s);

    public:
      // Constructor to instantiate a logger for a module
      Logger(){};
      Logger(std::string moduleName);
      /**
       * log methods for different levels
       */
      void logDebug(const std::string s);
      void logInfo (const std::string s);
      void logWarn (const std::string s);
      void logError(const std::string s);
     
  };
}
#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
