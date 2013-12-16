#include "suturo_perception.h"
#include "perceived_object.h"
#include "point.h"
#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

TEST(suturo_perception_test, box_1_test)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::stringstream boxpath;
  boxpath << "box1.pcd";

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (boxpath.str().c_str(), *cloud) == -1) //* load the file
  {
    std::stringstream error_msg;
    error_msg << "Couldn't read file " << boxpath << "\n";
    PCL_ERROR (error_msg.str().c_str());
    FAIL() << error_msg.str().c_str();
  }
  
  suturo_perception_lib::SuturoPerception sp;
  sp.processCloudWithProjections(cloud);
  
  std::vector<suturo_perception_lib::PerceivedObject> objects = sp.getPerceivedObjects();
  
  for (int i = 0; i < objects.size(); i++) 
  {
    std::cout << "Object " << i 
              << "\n\tid = " << objects.at(i).get_c_id()
              << "\n\tvolume = " << objects.at(i).get_c_volume()
              << "\n\tcentroid = ( " << objects.at(i).get_c_centroid().x << " | " 
                                     << objects.at(i).get_c_centroid().y << " | " 
                                     << objects.at(i).get_c_centroid().z << " )\n";
    std::cout << "\tshape = " << objects.at(i).get_c_shape() << std::endl;
    std::cout << "\tcolor_average_r = " << +objects.at(i).get_c_color_average_r() << std::endl;
    std::cout << "\tcolor_average_g = " << +objects.at(i).get_c_color_average_g() << std::endl;
    std::cout << "\tcolor_average_b = " << +objects.at(i).get_c_color_average_b() << std::endl;
    std::cout << "\tcolor_average_h = " << objects.at(i).get_c_color_average_h() << std::endl;
  }
  
  ASSERT_EQ(1, objects.size());
  ASSERT_LT(0.0032, objects.at(0).get_c_volume());
  ASSERT_GT(0.0036, objects.at(0).get_c_volume());
  ASSERT_LT(-0.009603, objects.at(0).get_c_centroid().x);
  ASSERT_GT(-0.009403, objects.at(0).get_c_centroid().x);
  ASSERT_LT(0.105987, objects.at(0).get_c_centroid().y);
  ASSERT_GT(0.107987, objects.at(0).get_c_centroid().y);
  ASSERT_LT(0.940000, objects.at(0).get_c_centroid().z);
  ASSERT_GT(0.940144, objects.at(0).get_c_centroid().z);
  
  SUCCEED();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
