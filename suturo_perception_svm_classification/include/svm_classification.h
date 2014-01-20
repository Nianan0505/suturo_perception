#ifndef SUTURO_PERCEPTION_SVM_CLASSIFICATION
#define SUTURO_PERCEPTION_SVM_CLASSIFICATION

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/signals2/mutex.hpp>

#include <ros/package.h>
#include "suturo_perception_utils.h"
#include "capability.h"
#include "perceived_object.h"
#include "suturo_perception_ml_classifiers_msgs/ClassDataPoint.h"


namespace suturo_perception_svm_classification
{
  class SVMClassification
  {
    public:
      SVMClassification();

      // basic functionality (wrapper for ml_classifiers)
      bool createClassifier(std::string identifier);
      bool addData(std::string identifier, std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> dpv);
      bool trainClassifier(std::string identifier);
      std::vector<std::string> classifyData(std::string identifier, std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> dpv);

      // vfh estimation related methods
      bool trainVFHData();

    private:
      suturo_perception_utils::Logger logger;
      ros::NodeHandle n;
      ros::Duration service_wait_timeout;

      bool loadVFHData(std::string directory);
      std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
      std::vector<std::string> split(const std::string &s, char delim);
  };
}
#endif 
