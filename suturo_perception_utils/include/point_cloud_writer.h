#ifndef POINT_CLOUD_WRITER_H
#define POINT_CLOUD_WRITER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace suturo_perception_utils
{
	/**
	 * Helper class to write several PointCloud Types to disk
	 */
  class PointCloudWriter
  {
    public:
      // Write a standard XYZRGB PointCloud to Disk
      static void writeCloudToDisk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, std::string filename);
      // Write a Vector of Pointclouds to Disk
      // The used filename for the clusters is
      // "debug_pcd_X.pcd" where X is the index of the object
      static void writeCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects);
      // Write a Vector of Pointclouds to Disk
      static void writeFirstCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects, std::string filename);
	};
}

#endif
