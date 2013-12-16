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
  // Set the ROI manually, so the object recognizer can pick the correct region
  suturo_perception_lib::ROI roi;
  roi.origin.x = 0;
  roi.origin.y = 0;
  roi.width = 284;
  roi.height = 380;
  po.set_c_roi(roi);
  po.set_c_recognition_label_2d("");

  ObjectMatcher om;
  cv::Mat image;
  std::string package_path = ros::package::getPath("suturo_perception_2d_capabilities");

  image = imread(package_path + "/test/muesli_front.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
  if(! image.data )                              // Check for invalid input
  {
    std::cerr <<  "Error: Could not open or find the training image" << std::endl ;
    FAIL();
  }

  boost::shared_ptr<cv::Mat> original_image( new cv::Mat(image) );
  std::vector<std::string> train_images;
  std::vector<std::string> train_labels;
  train_images.push_back( package_path + "/test/muesli_front.jpg");
  train_labels.push_back( "muesli" );
  om.trainImages(train_images, train_labels);
  LabelAnnotator2D la(po, original_image, om);
  la.execute();
  // After the object recognition, the perceived object should
  // contain the 2d label "muesli".
  ASSERT_STREQ("muesli", po.get_c_recognition_label_2d().c_str() );
  SUCCEED();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
