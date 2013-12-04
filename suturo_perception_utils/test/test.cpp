#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include "publisher_helper.h"
#include "ros/ros.h"

TEST(suturo_perception_utils, check_initial_state)
{
	char** argv;
	int argc = 0;
	// char[] nodename = "suturo_perception_test";
	// argv[0] = &
	// argv[1] = NULL;
	// argv = NULL;

  ros::init(argc, argv, "suturo_perception_test");
  ros::NodeHandle nh;
  ASSERT_EQ(1, 1);
  
  SUCCEED();
  ros::shutdown();
}


int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
