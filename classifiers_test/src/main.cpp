/*
 * roslaunch ml_classifiers classifier_server.launch
 * rosservice call /classifier_server/set_logger_level 'ros.ml_classifiers' 'DEBUG'
 */
#include "ros/ros.h"
#include <cstdlib>
#include "classifiers_test/ClassDataPoint.h"
#include "classifiers_test/CreateClassifier.h"
#include "classifiers_test/AddClassData.h"
#include "classifiers_test/TrainClassifier.h"
#include "classifiers_test/ClassifyData.h"


bool createClassifier(ros::NodeHandle n, std::string identifier, std::string type)
{
  ros::ServiceClient create_classifier = n.serviceClient<classifiers_test::CreateClassifier>("ml_classifiers/create_classifier", true);
  classifiers_test::CreateClassifier cc_srv;
  cc_srv.request.identifier = identifier;
  cc_srv.request.class_type = type;
  if (create_classifier.call(cc_srv))
  {
    ROS_INFO("call to create_classifier successful!");
  }
  else
  {
    ROS_ERROR("Failed to call service create_classifier");
    return false;
  }

  ROS_INFO(cc_srv.response.success?"create: true":"create: false");
  return true;
}

bool addClassDataPoints(ros::NodeHandle n, std::string identifier, std::vector<classifiers_test::ClassDataPoint> dpv)
{
  ros::ServiceClient add_class_data = n.serviceClient<classifiers_test::AddClassData>("/ml_classifiers/add_class_data", true);
  classifiers_test::AddClassData acd_srv;
  acd_srv.request.identifier = identifier;
  for (std::vector<classifiers_test::ClassDataPoint>::iterator it = dpv.begin(); it < dpv.end(); it++)
  {
    printf("%s: ",it->target_class.c_str());
    for (int i = 0; i < it->point.size(); i++)
    {
      printf("%f ", it->point[i]);
    }
    printf("\n");
  }

  acd_srv.request.data = dpv;
  if (add_class_data.call(acd_srv))
  {
    ROS_INFO("call to add_class_data successful!");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service add_class_data");
    return false;
  }
}

bool trainClassifier(ros::NodeHandle n, std::string identifier)
{
  ros::ServiceClient train_classifier = n.serviceClient<classifiers_test::TrainClassifier>("/ml_classifiers/train_classifier", true);
  classifiers_test::TrainClassifier t_srv;
  t_srv.request.identifier = identifier;
  if (train_classifier.call(t_srv))
  {
    ROS_INFO("call to train_classifier successful!");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service train_classifier");
    return false;
  }
}

bool classifyPoints(ros::NodeHandle n, std::string identifier, std::vector<classifiers_test::ClassDataPoint> dpv)
{
  ros::ServiceClient classify_data = n.serviceClient<classifiers_test::ClassifyData>("/ml_classifiers/classify_data", true);
  classifiers_test::ClassifyData cd_srv;
  cd_srv.request.identifier = identifier;
  printf("points to classify:\n");
  for (std::vector<classifiers_test::ClassDataPoint>::iterator it = dpv.begin(); it < dpv.end(); it++)
  {
    for (int i = 0; i < it->point.size(); i++)
    {
      printf("%f ", it->point[i]);
    }
    printf("\n");
  }

  cd_srv.request.data = dpv;
  if (classify_data.call(cd_srv))
  {
    ROS_INFO("call to classify_data successful!");
  }
  else
  {
    ROS_ERROR("Failed to call service classify_data");
    return false;
  }
  
  printf("classifications: \n");
  for (int i = 0; i < cd_srv.response.classifications.size(); i++)
  {
    printf("%d: %s\n", i, cd_srv.response.classifications[i].c_str());
  }

  return true;
}


#define data_dimension 2
#define train_size 5
#define test_size 3


int main(int argc, char **argv)
{
  float train_data[train_size][data_dimension] = {
    {0.1,0.2},
    {0.3,0.1},
    {3.1,3.2},
    {3.3,4.1},
    {5.1,5.2}
  };
  std::string train_data_targets[train_size] = {"1","1","2","2","3"};

  float test_data[test_size][data_dimension] = {
    {0.0,0.0},
    {5.5,5.5},
    {2.9,3.6}
  };

  std::vector<classifiers_test::ClassDataPoint> train_points;
  std::vector<classifiers_test::ClassDataPoint> test_points;

  printf("train:\n");
  for (int i = 0; i < train_size; i++)
  {
    classifiers_test::ClassDataPoint *dp = new classifiers_test::ClassDataPoint();
    for (int j = 0; j < data_dimension; j++)
    {
      dp->point.insert(dp->point.end(), train_data[i][j]);
      printf("%f ", train_data[i][j]);
    }
    printf("\n");
    dp->target_class = train_data_targets[i];
    train_points.insert(train_points.end(), *dp);
  }

  printf("test: \n");
  for (int i = 0; i < test_size; i++)
  {
    classifiers_test::ClassDataPoint *dp = new classifiers_test::ClassDataPoint();
    for (int j = 0; j < data_dimension; j++)
    {
      dp->point.insert(dp->point.end(), test_data[i][j]);
      printf("%f ", test_data[i][j]);
    }
    printf("\n");
    dp->target_class = "";
    test_points.insert(test_points.end(), *dp);
  }

  ros::init(argc, argv, "classifiers_test");

  ros::NodeHandle n;

  ros::service::waitForService("/ml_classifiers/create_classifier");

  if (!createClassifier(n, "testc","ml_classifiers/SVMClassifier"))
      return 1;
  
  if (!addClassDataPoints(n, "testc", train_points))
    return 1;

  if (!trainClassifier(n, "testc"))
    return 1;

  if (!classifyPoints(n, "testc", test_points))
    return 1;
  
  return 0;
}
