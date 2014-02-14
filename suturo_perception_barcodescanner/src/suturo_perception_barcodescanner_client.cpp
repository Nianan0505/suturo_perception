/**
 * This is a test client for the barcode scanner node
 *
 * It will call the provided scanner service in a loop and
 * show the codes found by the node.
 */
#include "ros/ros.h"
#include "suturo_perception_msgs/GetBarcode.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suturo_barcode_client");

  ros::NodeHandle n;

  ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetBarcode>("/suturo/barcodescanner");
  suturo_perception_msgs::GetBarcode clusterSrv;
  //clusterSrv.request.s = "get";
  ROS_INFO_STREAM("ServiceClient initialized");
  // run until service gets shut down
  while(true)
  {
    boost::this_thread::sleep(boost::posix_time::milliseconds(1)); // catch ctrl+c
    if (clusterClient.call(clusterSrv))
    {
      ROS_INFO("------------- Call successful. Vector size: %ld -------------", 
               (long int)clusterSrv.response.barcodes.size() );

      // wait a sec if list is empty. service may not be ready yet
      if((long int)clusterSrv.response.barcodes.size() == 0)
      {
        ROS_WARN_STREAM("No barcode found.");
        continue;
      }

      for(int i=0; i < clusterSrv.response.barcodes.size(); i++ ) {
        ROS_INFO("Type: %s Code: %s", clusterSrv.response.barcodes[i].type.c_str(), 
          clusterSrv.response.barcodes[i].code.c_str());
      }
    }
    else
    {
      ROS_ERROR("Failed to call service /suturo/GetBarcode");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}
