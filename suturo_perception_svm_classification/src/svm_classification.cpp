#include "svm_classification.h"

using namespace suturo_perception_svm_classification;

SVMClassification::SVMClassification(suturo_perception_lib::PerceivedObject &obj) : Capability(obj)
{
  logger = suturo_perception_utils::Logger("svm_classification");
}

void
SVMClassification::execute()
{

}
