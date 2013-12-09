#include "suturo_perception_utils.h"

using namespace suturo_perception_utils;

SuturoPerceptionUtils::SuturoPerceptionUtils(std::string m) : module(m){}

void SuturoPerceptionUtils::log(level lvl, std::string s)
{ 
  #ifndef HAVE_NO_ROS
  switch(lvl)
  {
    case DEBUG:
      ROS_DEBUG("[%s]  %s", module.c_str(), s.c_str());
    break;
    case INFO:
      ROS_INFO( "[%s]  %s", module.c_str(), s.c_str());
    break;
    case WARN:
      ROS_WARN( "[%s]  %s", module.c_str(), s.c_str());
    break;
    case ERROR:
      ROS_ERROR("[%s]  %s", module.c_str(), s.c_str());
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

void SuturoPerceptionUtils::logDebug(std::string s)
{
  log(DEBUG, s);
}

void SuturoPerceptionUtils::logInfo(std::string s)
{
  log(INFO, s);
}

void SuturoPerceptionUtils::logWarn(std::string s)
{
  log(WARN, s);
}

void SuturoPerceptionUtils::logError(std::string s)
{
  log(ERROR, s);
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
