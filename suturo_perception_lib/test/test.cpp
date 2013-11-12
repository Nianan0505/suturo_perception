#include "suturo_perception.h"
#include "PerceivedObject.h"
#include "Point.h"
#include <iostream>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

TEST(suturo_perception_test, magic_number_is_magic)
{
    SuturoPerception mc;
    EXPECT_EQ(mc.magicNumber(), 23);
}

TEST(suturo_perception_test, box_1_test)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/ofenrohr/kinect-records/pcl/box1.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file /home/ofenrohr/kinect-records/pcl/box1.pcd \n");
    FAIL() << "Couldn't read file /home/ofenrohr/kinect-records/pcl/box1.pcd" ;
  }
  
  SuturoPerception sp;
  sp.process_cloud(cloud);
  
  std::vector<PerceivedObject> *objects = sp.getPerceivedObjects();
  
  for (int i = 0; i < objects->size(); i++) 
  {
    std::cout << "Object " << i 
              << "\n\tid = " << objects->at(i).c_id 
              << "\n\tvolume = " << objects->at(i).c_volume
              << "\n\tcentroid = ( " << objects->at(i).c_centroid.x << " | " << objects->at(i).c_centroid.y << " | " << objects->at(i).c_centroid.z << " )\n";
  }
  
  SUCCEED();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}