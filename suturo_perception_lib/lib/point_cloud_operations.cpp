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

/*
 * Fit plane to the input cloud
 * Return the inliers.
 */
void 
 PointCloudOperations::fitPlanarModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
    pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, 
    int planeMaxIterations, 
    double planeDistanceThreshold)
{
  Logger logger("point_cloud_operations");
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  if(cloud_in->points.size() == 0)
  {
    logger.logError("Could not estimate a planar model for the given dataset. input cloud empty");
    return;
  }

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setModelType(pcl::SACMODEL_PLANE); // TODO: parameterize
  seg.setMethodType(pcl::SAC_RANSAC);    // TODO: parameterize
  seg.setMaxIterations(planeMaxIterations);
  seg.setDistanceThreshold(planeDistanceThreshold);
  seg.setInputCloud(cloud_in);
  seg.segment(*inliers,*coefficients);
  if (inliers->indices.size () == 0)
  {
    logger.logError("Could not estimate a planar model for the given dataset. The inlier size is 0");
  }

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "fitPlanarModel()");
}

void PointCloudOperations::extractInliersFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, bool setNegative)
{
  Logger logger("point_cloud_operations");
  // Input cloud can't be null
  if(inliers->indices.size () == 0)
  {
    logger.logError("extractInliersFromPointCloud can't work with an empty set of indices. Exiting....");
    return;
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_p;
  extract_p.setInputCloud(cloud_in);
  extract_p.setIndices(inliers);
  extract_p.filter(*cloud_out);
  extract_p.setNegative(setNegative);
}

/* Extract the biggest cluster in PointCloud cloud_in
* The method returns true, if a cluster has been found.
* If no Cluster could be extracted, or an error occured, the method returns false.
* If you want to map the original inliers (which correspond to the input cloud_in), you can pass in a Pointer to the old inliers and receive the new inliers after the clustering in new_inliers.
* When both inlier pointers are NOT NULL, the mapping will be calculated and put into new_inliers.
*
* ecObjClusterTolerance sets the ClusterTolance in pcl::EuclideanClusterExtraction.
* ecMinClusterSize ist the minimum size of a cluster, while ecMaxClusterSize is the maximum size of the cluster.
*/
bool PointCloudOperations::extractBiggestCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, const pcl::PointIndices::Ptr old_inliers, pcl::PointIndices::Ptr new_inliers,
    double ecClusterTolerance,
    int ecMinClusterSize,
    int ecMaxClusterSize)
{
  Logger logger("point_cloud_operations");
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  // Should we map the extracted points to the inliers in the input cloud?
  bool map_indices=false;
  if(old_inliers != NULL && new_inliers != NULL)
    map_indices = true;

  if(cloud_in->points.size() == 0)
  {
    logger.logError("Could not extract biggest cluster. input cloud empty");
    return false;
  }

  // Use cluster extraction to get rid of the outliers of the segmented table
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeTable (new pcl::search::KdTree<pcl::PointXYZRGB>);
  treeTable->setInputCloud (cloud_in);  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecTable;
  ecTable.setClusterTolerance (ecClusterTolerance); // 2cm
  ecTable.setMinClusterSize (ecMinClusterSize);
  ecTable.setMaxClusterSize (ecMaxClusterSize);
  ecTable.setSearchMethod (treeTable);
  ecTable.setInputCloud (cloud_in);
  ecTable.extract (cluster_indices);

  std::vector<int> cluster_get_indices;
  cluster_get_indices = *ecTable.getIndices();

  logger.logInfo((boost::format("cluster_indices vector size: %s") % cluster_indices.size()).str());

  if(cluster_indices.size() == 0)
  {
    logger.logError("No suitable cluster found for extraction. Skip ...");
    return false;
  }


  // Extract the biggest cluster (e.g. the table) in the plane cloud
  std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
    cloud_out->points.push_back (cloud_in->points[*pit]); 

    if(map_indices)
      new_inliers->indices.push_back(cluster_get_indices.at(*pit)); // Map the indices
                                                                  // from the cluster extraction
                                                                  // to a proper PointIndicies
                                                                  // instance relative to our
                                                                  // original plane
  }
  cloud_out->width = cloud_out->points.size ();
  cloud_out->height = 1;
  cloud_out->is_dense = true;

  // logger.logError("New Inliers calculated: " << new_inliers->indices.size());

  // if(writer_pcd) writer.write ("cloud_out.pcd", *cloud_out, false);

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "extractBiggestCluster()");
  return true;
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
