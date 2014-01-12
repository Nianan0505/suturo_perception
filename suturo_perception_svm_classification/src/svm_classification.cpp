#include "svm_classification.h"

#include "suturo_perception_ml_classifiers_msgs/CreateClassifier.h"
#include "suturo_perception_ml_classifiers_msgs/AddClassData.h"
#include "suturo_perception_ml_classifiers_msgs/TrainClassifier.h"
#include "suturo_perception_ml_classifiers_msgs/ClassifyData.h"

using namespace suturo_perception_svm_classification;

SVMClassification::SVMClassification(suturo_perception_lib::PerceivedObject &obj) : Capability(obj)
{
  logger = suturo_perception_utils::Logger("svm_classification");
  service_wait_timeout = ros::Duration(10, 0);
}

bool 
SVMClassification::createClassifier(std::string identifier)
{
  if (!ros::service::waitForService("/ml_classifiers/create_classifier", service_wait_timeout))
  {
    logger.logError("timeout when waiting for service /ml_classifiers/create_classifier");
    return false;
  }
  ros::ServiceClient create_classifier = n.serviceClient<suturo_perception_ml_classifiers_msgs::CreateClassifier>("ml_classifiers/create_classifier", true);
  suturo_perception_ml_classifiers_msgs::CreateClassifier cc_srv;
  cc_srv.request.identifier = identifier;
  cc_srv.request.class_type = "ml_classifiers/SVMClassifier";
  if (create_classifier.call(cc_srv))
  {
    logger.logInfo("call to create_classifier successful!");
  }
  else
  {
    logger.logError("Failed to call service create_classifier");
    return false;
  }

  logger.logInfo(cc_srv.response.success?"create: true":"create: false");
  return true;

}

bool 
SVMClassification::addData(std::string identifier, std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> dpv)
{
  if (!ros::service::waitForService("/ml_classifiers/add_class_data", service_wait_timeout))
  {
    logger.logError("timeout when waiting for service /ml_classifiers/add_class_data");
    return false;
  }
  ros::ServiceClient add_class_data = n.serviceClient<suturo_perception_ml_classifiers_msgs::AddClassData>("/ml_classifiers/add_class_data", true);
  suturo_perception_ml_classifiers_msgs::AddClassData acd_srv;
  acd_srv.request.identifier = identifier;

  acd_srv.request.data = dpv;
  if (add_class_data.call(acd_srv))
  {
    logger.logInfo("call to add_class_data successful!");
    return true;
  }
  else
  {
    logger.logError("Failed to call service add_class_data");
    return false;
  }

}

bool 
SVMClassification::trainClassifier(std::string identifier)
{
  if (!ros::service::waitForService("/ml_classifiers/train_classifier", service_wait_timeout))
  {
    logger.logError("timeout when waiting for service /ml_classifiers/train_classifier");
    return false;
  }
  ros::ServiceClient train_classifier = n.serviceClient<suturo_perception_ml_classifiers_msgs::TrainClassifier>("/ml_classifiers/train_classifier", true);
  suturo_perception_ml_classifiers_msgs::TrainClassifier t_srv;
  t_srv.request.identifier = identifier;
  if (train_classifier.call(t_srv))
  {
    logger.logInfo("call to train_classifier successful!");
    return true;
  }
  else
  {
    logger.logError("Failed to call service train_classifier");
    return false;
  }
}

std::vector<std::string> 
SVMClassification::classifyData(std::string identifier, std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> dpv)
{
  std::vector<std::string> empty_ret;
  if (!ros::service::waitForService("/ml_classifiers/classify_data", service_wait_timeout))
  {
    logger.logError("timeout when waiting for service /ml_classifiers/classify_data");
    return empty_ret;
  }
  ros::ServiceClient classify_data = n.serviceClient<suturo_perception_ml_classifiers_msgs::ClassifyData>("/ml_classifiers/classify_data", true);
  suturo_perception_ml_classifiers_msgs::ClassifyData cd_srv;
  cd_srv.request.identifier = identifier;

  cd_srv.request.data = dpv;
  if (classify_data.call(cd_srv))
  {
    logger.logInfo("call to classify_data successful!");
    return cd_srv.response.classifications;
  }
  else
  {
    logger.logError("Failed to call service classify_data");
    return empty_ret;
  }
  
}

void
SVMClassification::execute()
{

}
