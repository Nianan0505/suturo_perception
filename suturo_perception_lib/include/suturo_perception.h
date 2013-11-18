#ifndef SUTURO_PERCEPTION_H
#define SUTURO_PERCEPTION_H

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include "perceived_object.h"

namespace suturo_perception_lib
{
  class SuturoPerception
  {
    public:
    SuturoPerception();
    void processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    std::vector<PerceivedObject> getPerceivedObjects();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterZAxis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    pcl::PointIndices::Ptr fitPlanarModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  extractObjectCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
                               pcl::PointIndices::Ptr inliers);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    
    private:
    // === Parameters ===
    // min and max values for z-axis filtering
    float zAxisFilterMin;
    float zAxisFilterMax;
    // leafzise for downsampling
    float downsampleLeafSize;
    // plane fitting parameters
    int planeMaxIterations;
    double planeDistanceThreshold;
    // extract object parameters
    double ecClusterTolerance;
    int ecMinClusterSize;
    int ecMaxClusterSize;
    double prismZMin;
    double prismZMax;

    // ID counter for the perceived objects
    int objectID;
    // Buffer for the last perceived objects
    std::vector<PerceivedObject> perceivedObjects;
    
    // Mutex for buffer locking
    boost::signals2::mutex mutex;
  };
}

#endif
