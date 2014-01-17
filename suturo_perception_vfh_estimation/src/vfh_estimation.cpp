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
  printf("a");
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  printf("b");
  ne.setInputCloud (cloud);
  printf("c");


  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  printf("d");
  ne.setSearchMethod (tree);
  printf("e");

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  printf("f");

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);
  printf("g");

  // Compute the features
  ne.compute (*cloud_normals);

  printf("estimated normals\n");

  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
  printf("1");
  vfh.setInputCloud (cloud);
  printf("2");
  vfh.setInputNormals (cloud_normals);
  printf("3");

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr vfh_tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  printf("4");
  vfh.setSearchMethod (vfh_tree);
  printf("5");

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
  printf("6");

  // Compute the features
  vfh.compute (*vfhs);
  printf("7");

  printf("computed vfh\n");

  if (vfhs->points.size() == 1)
    return vfhs->points[0];

  printf("bad bad shit\n");
}

void
VFHEstimation::execute()
{
  pcl::VFHSignature308 sig = estimateCloud(perceivedObject.get_pointCloud());
  perceivedObject.set_c_vfhs(sig);
}
