#include "ros/ros.h"
#include <cstdlib>
#include "classifiers_test/ClassDataPoint.h"
#include "classifiers_test/CreateClassifier.h"
#include "classifiers_test/AddClassData.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "classifiers_test");

  ros::NodeHandle n;
  ros::ServiceClient create_classifier = n.serviceClient<classifiers_test::CreateClassifier>("/ml_classifiers/create_classifier", true);
  classifiers_test::CreateClassifier cc_srv;
  cc_srv.request.identifier = "test";
  cc_srv.request.class_type = "ml_classifiers/SVMClassifier";
  if (create_classifier.call(cc_srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    ROS_INFO("call to create_classifier successful!");
  }
  else
  {
    ROS_ERROR("Failed to call service create_classifier");
    return 1;
  }

  ros::ServiceClient add_class_data = n.serviceClient<classifiers_test::AddClassData>("/ml_classifiers/add_class_data", true);
  classifiers_test::AddClassData acd_srv;
  classifiers_test::ClassDataPoint dp;
  dp.point.insert(dp.point.end(), 1.0);
  dp.point.insert(dp.point.end(), 0.5);
  dp.target_class = "foobar";
  acd_srv.request.identifier = "test";
  acd_srv.request.data.insert(acd_srv.request.data.end(), dp);
  /*
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  */
  if (add_class_data.call(acd_srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    ROS_INFO("call to add_class_data successful!");
  }
  else
  {
    ROS_ERROR("Failed to call service add_class_data");
    return 1;
  }

  return 0;
}
