#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <sstream>

void writeCloudToDisk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename)
{
	pcl::PCDWriter writer;
	writer.write(filename, *cloud);
}

void convertBag(std::string filename) 
{
  rosbag::Bag bag(filename.c_str());
  rosbag::View view(bag, rosbag::TopicQuery("/kinect_head/depth_registered/points"));
  int i = 0;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    sensor_msgs::PointCloud2::ConstPtr inputCloud = m.instantiate<sensor_msgs::PointCloud2>();
    if (inputCloud == NULL)
    {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*inputCloud,*cloud_in);

    std::stringstream ss;
    ss << filename << "." << i << ".pcd";
    std::string pcd_filename = ss.str();
    writeCloudToDisk(cloud_in, pcd_filename);
    i++;
    std::cout << "wrote pcd: " << pcd_filename << "\n";
  }
  bag.close();
}

int main(int argc, char **argv)
{
  convertBag("/tmp/test.bag");
}

