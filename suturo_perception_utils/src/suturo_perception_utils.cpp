#include "suturo_perception_utils.h"

using namespace suturo_perception_utils;

Logger::Logger(std::string m) : module(m){}

void Logger::log(level lvl, std::string s)
{ 
  #ifndef HAVE_NO_ROS
  switch(lvl)
  {
    case DEBUG:
      ROS_DEBUG("[%s] %s", module.c_str(), s.c_str());
    break;
    case INFO:
      ROS_INFO( "[%s] %s", module.c_str(), s.c_str());
    break;
    case WARN:
      ROS_WARN( "[%s] %s", module.c_str(), s.c_str());
    break;
    case ERROR:
      ROS_ERROR("[%s] %s", module.c_str(), s.c_str());
    break;
  }
  
  #else
    switch(lvl)
  {
    case DEBUG:
      std::cout << "[DEBUG] " << "[" << module << "]  " << s << std::endl;
    break;
    case INFO:
      std::cout << "[INFO] "  << "[" << module << "]  " << s << std::endl;
    break;
    case WARN:
      std::cout << "[WARN] "  << "[" << module << "]  " << s << std::endl;
    break;
    case ERROR:
      std::cout << "[ERROR] " << "[" << module << "]  " << s << std::endl;
    break;
  }
  #endif

}

void Logger::logDebug(const std::string s)
{
  log(DEBUG, s);
}

void Logger::logInfo(const std::string s)
{
  log(INFO, s);
}

void Logger::logWarn(const std::string s)
{
  log(WARN, s);
}

void Logger::logError(const std::string s)
{
  log(ERROR, s);
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
