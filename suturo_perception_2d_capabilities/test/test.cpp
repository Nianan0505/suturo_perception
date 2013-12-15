#include "suturo_perception_2d_capabilities/label_annotator_2d.h"
#include "perceived_object.h"
#include "object_matcher.h"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/package.h>

using namespace suturo_perception_2d_capabilities;

TEST(suturo_perception_2d_capabilities, test_execute)
{
  suturo_perception_lib::PerceivedObject po;
  ObjectMatcher om;
  cv::Mat image;
  std::string path = ros::package::getPath("suturo_perception_2d_capabilities");

  image = imread(path + "/test/muesli_front.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
  if(! image.data )                              // Check for invalid input
  {
    std::cerr <<  "Error: Could not open or find the training image" << std::endl ;
    FAIL();
  }
  // LabelAnnotator2D la(PerceivedObject &obj, boost::shared_ptr<cv::Mat> original_image, ObjectMatcher &object_matcher);
  SUCCEED();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
