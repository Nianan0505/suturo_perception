#include "vfh_estimation.h"

#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

using namespace suturo_perception_vfh_estimation;

VFHEstimation::VFHEstimation(PerceivedObject &obj) : Capability(obj)
{
  logger = suturo_perception_utils::Logger("vfh_estimation");
}

pcl::VFHSignature308 
VFHEstimation::estimateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  printf("vfhestimation got got cloud with size = %d\n", cloud->size());
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (cloud_normals);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr vfh_tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  vfh.setSearchMethod (vfh_tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
  vfh.compute (*vfhs);

  if (vfhs->points.size() == 1)
    return vfhs->points[0];
}

void
VFHEstimation::execute()
{
  pcl::VFHSignature308 sig = estimateCloud(perceivedObject.get_pointCloud());
  perceivedObject.set_c_vfhs(sig);
}
