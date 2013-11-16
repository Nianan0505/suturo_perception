#include "suturo_perception.h"
#include "suturo_perception_msgs/GetClusters.h"
#include "Point.h"
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
  
  SuturoPerception sp;
  sp.process_cloud(cloud);
  
  std::vector<suturo_perception_msgs::PerceivedObject> objects = sp.getPerceivedObjects();
  
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
  ASSERT_LT(0.00523330, objects.at(0).c_centroid.x);
  ASSERT_GT(0.00523334, objects.at(0).c_centroid.x);
  ASSERT_LT(0.106525, objects.at(0).c_centroid.y);
  ASSERT_GT(0.106529, objects.at(0).c_centroid.y);
  ASSERT_LT(0.948483, objects.at(0).c_centroid.z);
  ASSERT_GT(0.948487, objects.at(0).c_centroid.z);
  
  SUCCEED();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}