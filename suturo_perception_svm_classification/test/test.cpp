#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include "suturo_perception_ml_classifiers_msgs/CreateClassifier.h"
#include "suturo_perception_ml_classifiers_msgs/AddClassData.h"
#include "suturo_perception_ml_classifiers_msgs/TrainClassifier.h"
#include "suturo_perception_ml_classifiers_msgs/ClassifyData.h"
#include "svm_classification.h"

TEST(svm_classification_test, classification_1_test)
{
  float train_data[5][2] = {
    {0.1,0.2},
    {0.3,0.1},
    {3.1,3.2},
    {3.3,4.1},
    {5.1,5.2}
  };
  std::string train_data_targets[5] = {"1","1","2","2","3"};

  float test_data[3][2] = {
    {0.0,0.0},
    {5.5,5.5},
    {2.9,3.6}
  };

  std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> train_points;
  std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> test_points;

  printf("train:\n");
  for (int i = 0; i < 5; i++)
  {
    suturo_perception_ml_classifiers_msgs::ClassDataPoint *dp = new suturo_perception_ml_classifiers_msgs::ClassDataPoint();
    for (int j = 0; j < 2; j++)
    {
      dp->point.insert(dp->point.end(), train_data[i][j]);
      printf("%f ", train_data[i][j]);
    }
    printf("\n");
    dp->target_class = train_data_targets[i];
    train_points.insert(train_points.end(), *dp);
  }

  printf("test: \n");
  for (int i = 0; i < 3; i++)
  {
    suturo_perception_ml_classifiers_msgs::ClassDataPoint *dp = new suturo_perception_ml_classifiers_msgs::ClassDataPoint();
    for (int j = 0; j < 2; j++)
    {
      dp->point.insert(dp->point.end(), test_data[i][j]);
      printf("%f ", test_data[i][j]);
    }
    printf("\n");
    dp->target_class = "";
    test_points.insert(test_points.end(), *dp);
  }

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "classifiers_test");

  suturo_perception_lib::PerceivedObject obj;
  suturo_perception_svm_classification::SVMClassification svmc(obj);

  if (!svmc.createClassifier("testc"))
    ASSERT_TRUE(false);
  
  if (!svmc.addData("testc", train_points))
    ASSERT_TRUE(false);

  if (!svmc.trainClassifier("testc"))
    ASSERT_TRUE(false);

  std::vector<std::string> classification = svmc.classifyData("testc", test_points);

  ASSERT_EQ(3, classification.size());
  ASSERT_EQ("1", classification.at(0));
  ASSERT_EQ("3", classification.at(1));
  ASSERT_EQ("2", classification.at(2));
 
  // TODO: test outcome!
  //if (!svmc.classifyData("testc", test_points))
  //  ASSERT_TRUE(false);

 
  ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
