/**
 * This class can be used to extract the VFH descriptor
 * from a given PointCloud
 */
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <ros/package.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  std::string package_path = ros::package::getPath("vfh_prototype");

  // This file is provided by the VFH data set
  // Please read the README file in the vfh_prototype package
  // to get this data.
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (package_path + "/data/000.580.67/1258250240333_cluster_0_nxyz.pcd", *cloud) == -1)

  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

// Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (cloud_normals);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr vfh_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (vfh_tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
  vfh.compute (*vfhs);
  pcl::io::savePCDFile(package_path + "/estimated_descriptor_vfh.pcd", *vfhs);
  // vfhs->points.size () should be of size 1*
  return 0;
}