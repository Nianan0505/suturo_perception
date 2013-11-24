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
  sp.processCloud(cloud);
  
  std::vector<suturo_perception_lib::PerceivedObject> objects = sp.getPerceivedObjects();
  
  for (int i = 0; i < objects.size(); i++) 
  {
    std::cout << "Object " << i 
              << "\n\tid = " << objects.at(i).c_id 
              << "\n\tvolume = " << objects.at(i).c_volume
              << "\n\tcentroid = ( " << objects.at(i).c_centroid.x << " | " << objects.at(i).c_centroid.y << " | " << objects.at(i).c_centroid.z << " )\n";
  }
  
  ASSERT_EQ(1, objects.size());
  ASSERT_LT(0.0032, objects.at(0).c_volume);
  ASSERT_GT(0.0036, objects.at(0).c_volume);
  ASSERT_LT(-0.00453912, objects.at(0).c_centroid.x);
  ASSERT_GT(-0.00453904, objects.at(0).c_centroid.x);
  ASSERT_LT(0.108890, objects.at(0).c_centroid.y);
  ASSERT_GT(0.108898, objects.at(0).c_centroid.y);
  ASSERT_LT(0.948475, objects.at(0).c_centroid.z);
  ASSERT_GT(0.948483, objects.at(0).c_centroid.z);
  
  SUCCEED();
}

TEST(suturo_perception_test, color_1_test)
{
  suturo_perception_lib::SuturoPerception sp;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>());
  
  pcl::PointXYZRGB point1, point2, point3, point4, point5, point6;
  uint8_t r = 255, g = 0, b = 0;
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  point1.rgb = *reinterpret_cast<float*>(&rgb);
  point2.rgb = *reinterpret_cast<float*>(&rgb);
  point3.rgb = *reinterpret_cast<float*>(&rgb);
  cloud->points.push_back(point1);
  cloud->points.push_back(point2);
  cloud->points.push_back(point3);
  
  uint32_t averageColor = sp.getAverageColor(cloud);
  ASSERT_EQ(255, (averageColor >> 16) & 0x0000ff);

  r = 0, g = 200, b = 100;
  uint8_t g1 = 150, b1=50;
  uint32_t rgb1 = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  uint32_t rgb2 = ((uint32_t)r << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
  point4.rgb = *reinterpret_cast<float*>(&rgb1);
  point5.rgb = *reinterpret_cast<float*>(&rgb2);
  point6.rgb = *reinterpret_cast<float*>(&rgb2);
  cloud2->points.push_back(point4);
  cloud2->points.push_back(point5);
  cloud2->points.push_back(point6);

  averageColor = sp.getAverageColor(cloud2);
  ASSERT_EQ(166, (averageColor >> 8) & 0x0000ff);
  ASSERT_EQ(66, (averageColor) & 0x0000ff);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
