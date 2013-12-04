#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include "publisher_helper.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>

using namespace suturo_perception_helper;

TEST(suturo_perception_utils, check_initial_state)
{
	char** argv;
	int argc = 0;

  ros::init(argc, argv, "suturo_perception_test");
  ros::NodeHandle nh;
  PublisherHelper ph(nh);
	ASSERT_FALSE(ph.isAdvertised("/gtest_topic"));
	ASSERT_TRUE( ph.getPublisher("/gtest_topic") == NULL);
  ros::shutdown();
  SUCCEED();
}

TEST(suturo_perception_utils, check_advertise_topic)
{
	char** argv;
	int argc = 0;

  ros::init(argc, argv, "suturo_perception_test");
  ros::NodeHandle nh;
  PublisherHelper ph(nh);
	ASSERT_FALSE(ph.isAdvertised("/gtest_topic"));
	ASSERT_TRUE( ph.getPublisher("/gtest_topic") == NULL);
	// Advertise a topic
	ph.advertise<sensor_msgs::PointCloud>("/gtest_topic");
	ASSERT_TRUE(ph.isAdvertised("/gtest_topic"));
	ASSERT_TRUE( ph.getPublisher("/gtest_topic") != NULL);

	std::vector<std::string> topics;
	ros::this_node::getAdvertisedTopics(topics);

	// We must be able to find the announced topic with ROS methods
	ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/gtest_topic")!=topics.end());

	// ASSERT_STREQ works with C Strings! Convert them if you need to
	ASSERT_STREQ(ph.getPublisher("/gtest_topic")->getTopic().c_str(), "/gtest_topic");
  
  ros::shutdown();
  SUCCEED();
}

TEST(suturo_perception_utils, check_advertise_multiple_topics)
{
	char** argv;
	int argc = 0;

  ros::init(argc, argv, "suturo_perception_test");
  ros::NodeHandle nh;
  PublisherHelper ph(nh);
	ASSERT_FALSE(ph.isAdvertised("/gtest_topic"));
	ASSERT_FALSE(ph.isAdvertised("/gtest_topic1"));
	ASSERT_FALSE(ph.isAdvertised("/gtest_topic2"));

	// Advertise multiple topics
	ph.advertise<sensor_msgs::PointCloud>("/gtest_topic");
	ph.advertise<sensor_msgs::PointCloud>("/gtest_topic1");
	ph.advertise<sensor_msgs::PointCloud>("/gtest_topic2");

	ASSERT_TRUE(ph.isAdvertised("/gtest_topic"));
	ASSERT_TRUE(ph.isAdvertised("/gtest_topic1"));
	ASSERT_TRUE(ph.isAdvertised("/gtest_topic2"));

	std::vector<std::string> topics;
	ros::this_node::getAdvertisedTopics(topics);

	// We must be able to find the announced topic with ROS methods
	ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/gtest_topic")!=topics.end());
	ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/gtest_topic1")!=topics.end());
	ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/gtest_topic2")!=topics.end());

	// ASSERT_STREQ works with C Strings! Convert them if you need to
	ASSERT_STREQ(ph.getPublisher("/gtest_topic")->getTopic().c_str(), "/gtest_topic");
	ASSERT_STREQ(ph.getPublisher("/gtest_topic1")->getTopic().c_str(), "/gtest_topic1");
	ASSERT_STREQ(ph.getPublisher("/gtest_topic2")->getTopic().c_str(), "/gtest_topic2");
  
  ros::shutdown();
  SUCCEED();
}



int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
