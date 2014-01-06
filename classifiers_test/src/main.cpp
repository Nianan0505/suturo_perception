#include "ros/ros.h"
#include <cstdlib>
#include "classifiers_test/ClassDataPoint.h"
#include "classifiers_test/AddClassData.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "classifiers_test");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<classifiers_test::AddClassData>("/ml_classifiers/add_class_data");
  classifiers_test::AddClassData srv;
  classifiers_test::ClassDataPoint dp;
  dp.point.insert(dp.point.end(), 1.0);
  dp.point.insert(dp.point.end(), 0.5);
  dp.target_class = "foobar";
  srv.request.data.insert(srv.request.data.end(), dp);
  /*
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  */
  if (client.call(srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    ROS_INFO("call successful!");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
