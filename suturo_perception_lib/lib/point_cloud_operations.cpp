#include "point_cloud_operations.h"

using namespace suturo_perception_lib;
using namespace suturo_perception_utils;

    // float zAxisFilterMin;
    // float zAxisFilterMax;
    // // leafzise for downsampling
    // float downsampleLeafSize;
    // // plane fitting parameters
    // int planeMaxIterations;
    // double planeDistanceThreshold;
    // // extract object cluster parameters
    // double ecClusterTolerance;
    // int ecMinClusterSize;
    // int ecMaxClusterSize;
    // double prismZMin;
    // double prismZMax;
    // // extract objects parameters
    // double ecObjClusterTolerance;
    // int ecObjMinClusterSize;
    // int ecObjMaxClusterSize;
/*
 * Remove NaNs from given pointcloud. 
 * Return the nanles cloud.
 */
void PointCloudOperations::removeNans(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles)
{
  Logger logger("point_cloud_operations");
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  std::vector<int> nans;
  pcl::removeNaNFromPointCloud(*cloud_in,*cloud_nanles,nans);

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "removeNans()");
}

/**
 * Filter cloud on z-axis. aka cut off cloud at given distance.
 * This method will use setKeepOrganized on the given PassThrough Filter.
 *
 * Return the filtered cloud.
 */
void 
 PointCloudOperations::filterZAxis(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::PassThrough<pcl::PointXYZRGB> &pass,
    float zAxisFilterMin, float zAxisFilterMax)
{
  Logger logger("point_cloud_operations");

  if(cloud_in->points.size() == 0)
  {
    logger.logError("Could not filter on Z Axis. input cloud empty");
    return;
  }

  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(zAxisFilterMin, zAxisFilterMax);
  pass.setKeepOrganized(true);
  pass.filter(*cloud_out);

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "filterZAxis()");
}

/*
 * Downsample the input cloud with a pcl::VoxelGrid
 * Return the filtered cloud.
 */
void 
 PointCloudOperations::downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, float downsampleLeafSize)
{
  Logger logger("point_cloud_operations");
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  pcl::VoxelGrid <pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(downsampleLeafSize,downsampleLeafSize,downsampleLeafSize);
  vg.filter(*cloud_out);

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "downsample()");
}
// 
// /*
//  * Fit plane to the input cloud
//  * Return the inliers.
//  */
// void 
//  fitPlanarModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
//     pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
// {
//   Logger logger("point_cloud_operations");
//   boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
// 
//   if(cloud_in->points.size() == 0)
//   {
//     logger.logError("Could not estimate a planar model for the given dataset. input cloud empty");
//     return;
//   }
// 
//   pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//   seg.setModelType(pcl::SACMODEL_PLANE); // TODO: parameterize
//   seg.setMethodType(pcl::SAC_RANSAC);    // TODO: parameterize
//   seg.setMaxIterations(planeMaxIterations);
//   seg.setDistanceThreshold(planeDistanceThreshold);
//   seg.setInputCloud(cloud_in);
//   seg.segment(*inliers,*coefficients);
//   if (inliers->indices.size () == 0)
//   {
//     logger.logError("Could not estimate a planar model for the given dataset. The inlier size is 0");
//   }
// 
//   boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
//   logger.logTime(s, e, "fitPlanarModel()");
// }
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
