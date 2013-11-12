#include "ros/ros.h"
#include <boost/signals2/mutex.hpp>

#include "suturo_perception.h"
#include "suturo_perception_msgs/GetClusters.h"


class SuturoPerceptionROSNode
{
public:
    /*
     * Constructor
     */
    SuturoPerceptionROSNode(ros::NodeHandle& n) : nh(n)
    {
        processing = false;
    }

    /*
     * Receive callback for the /camera/depth_registered/points subscription
     */
    void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
    {
        // process only one cloud
        if(processing)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*inputCloud,*cloud_in);

            sp.process_cloud(cloud_in);
            processing = false;

            ROS_INFO("Received a new point cloud: size = %lu",cloud_in->points.size());
        }
    }

    /*
     * Implementation of the GetClusters Service.
     *
     * This method will subscribe to the /camera/depth_registered/points topic, 
     * wait for the processing of a single point cloud, and return the result from
     * the calulations as a list of PerceivedObjects.
     */
    bool getClusters(suturo_perception_msgs::GetClusters::Request &req,
                     suturo_perception_msgs::GetClusters::Response &res)
    {
        ros::Subscriber sub;
        processing = true;

        // signal failed call, if request string does not match
        if (req.s.compare("get") != 0)
        {
          return false;
        }

        // Subscribe to the depth information topic
        sub = nh.subscribe("/camera/depth_registered/points", 1, 
          &SuturoPerceptionROSNode::receive_cloud, this);

        ROS_INFO("Waiting for processed cloud");
            ros::Rate r(20); // 20 hz
        while(processing){
                ros::spinOnce();
                r.sleep();
            }
        ROS_INFO("Cloud processed. Lock buffer and return the results");

        mutex.lock();
        res.perceivedObjs = perceivedObjects;
        mutex.unlock();

        return true;
    }

private:
    bool processing; // processing flag
    SuturoPerception sp;
    std::vector<suturo_perception_msgs::PerceivedObject> perceivedObjects;
    ros::NodeHandle nh;
    boost::signals2::mutex mutex;   
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "suturo_perception_rosnode");
    ros::NodeHandle nh;
    SuturoPerceptionROSNode spr(nh);
    ROS_INFO("suturo_perception READY");
    //sp.sayHi();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return (0);
}
