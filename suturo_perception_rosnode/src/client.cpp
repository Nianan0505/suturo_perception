/**
 * This class implements the client interface to query
 * the PerceivedObjects from the PerceptionServer.
 *
 * It will execute a service call to "GetClusters" and print
 * the returned list of PerceivedObjects in a textual form.
 */
#include "ros/ros.h"
#include "suturo_perception_msgs/GetClusters.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_client");

  ros::NodeHandle n;

  ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetClusters>("/suturo/GetClusters");
  suturo_perception_msgs::GetClusters clusterSrv;
  clusterSrv.request.s = "get";
  ROS_INFO_STREAM("ServiceClient initialized");
  // run until service gets shut down
  while(true)
  {
    if (clusterClient.call(clusterSrv))
    {
      ROS_INFO("Cluster Service call successful");
      ROS_INFO("List size: %ld", (long int)clusterSrv.response.perceivedObjs.size() );

      // wait a sec if list is empty. service may not be ready yet
      if((long int)clusterSrv.response.perceivedObjs.size() == 0)
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ROS_INFO_STREAM("No objects perceived yet.");
        continue;
      }

      for(int i=0; i < clusterSrv.response.perceivedObjs.size(); i++ ) {
        ROS_INFO("ID of perceived object is: %d", clusterSrv.response.perceivedObjs[i].c_id);
        ROS_INFO("Frame of perceived object is: %s", clusterSrv.response.perceivedObjs[i].frame_id.c_str());
        ROS_INFO("Volume of perceived object is: %f", clusterSrv.response.perceivedObjs[i].c_volume);
        ROS_INFO("Shape of perceived object is: %i", clusterSrv.response.perceivedObjs[i].c_shape);
        ROS_INFO("Average RGB color of perceived object is: r: %i  g: %i  b: %i", 
                  clusterSrv.response.perceivedObjs[i].c_color_average_r,
                  clusterSrv.response.perceivedObjs[i].c_color_average_g,
                  clusterSrv.response.perceivedObjs[i].c_color_average_b);
        ROS_INFO("Average HSV color of perceived object is: h: %i  s: %f  v: %f", 
                  clusterSrv.response.perceivedObjs[i].c_color_average_h,
                  clusterSrv.response.perceivedObjs[i].c_color_average_s,
                  clusterSrv.response.perceivedObjs[i].c_color_average_v);
        ROS_INFO("2D recognition label of perceived object is: %s", clusterSrv.response.perceivedObjs[i].recognition_label_2d.c_str());
        ROS_INFO("ROI of object: (origin, width, height): ( %f x %f ) , %i , %i ",
          clusterSrv.response.perceivedObjs[i].c_roi_origin.x,
          clusterSrv.response.perceivedObjs[i].c_roi_origin.y,
          clusterSrv.response.perceivedObjs[i].c_roi_width, 
          clusterSrv.response.perceivedObjs[i].c_roi_height);
        ROS_INFO("Quality of hue histogram: %d ", clusterSrv.response.perceivedObjs[i].c_hue_histogram_quality);
        ROS_INFO("Centroid(x) of perceived object is: %f , %f , %f ",
          clusterSrv.response.perceivedObjs[i].c_centroid.x,
          clusterSrv.response.perceivedObjs[i].c_centroid.y,
          clusterSrv.response.perceivedObjs[i].c_centroid.z);
        std::stringstream vfh_ss;
        for (int j = 0; j < clusterSrv.response.perceivedObjs[i].c_vfh_estimation.size(); j++)
        {
          vfh_ss << clusterSrv.response.perceivedObjs[i].c_vfh_estimation.at(j);
          vfh_ss << " ";
        }
        ROS_INFO("VFH Estimation: %s ", vfh_ss.str().c_str());
        ROS_INFO("SVM Result: %s ", clusterSrv.response.perceivedObjs[i].c_svm_result.c_str());
        ROS_INFO("Pose: %d ", clusterSrv.response.perceivedObjs[i].c_pose);
      }
      ROS_INFO_STREAM("------------------------------------------------------------");
    }
    else
    {
      ROS_ERROR("Failed to call service /suturo/GetClusters");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}
