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

    // debug
    void writeCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects);

    // shape detection
    void detectShape(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn);

    // getters and setters
    void setZAxisFilterMin(float v) {zAxisFilterMin = v;};
    void setZAxisFilterMax(float v) {zAxisFilterMax = v;};
    void setDownsampleLeafSize(float v) {downsampleLeafSize = v;};
    void setPlaneMaxIterations(int v) {planeMaxIterations = v;};
    void setPlaneDistanceThreshold(double v) {planeDistanceThreshold = v;};
    void setEcClusterTolerance(double v) {ecClusterTolerance = v;};
    void setEcMinClusterSize(int v) {ecMinClusterSize = v;};
    void setEcMaxClusterSize(int v) {ecMaxClusterSize = v;};
    void setPrismZMin(float v) {prismZMin = v;};
    void setPrismZMax(float v) {prismZMax = v;};
    void setEcObjClusterTolerance(double v) {ecObjClusterTolerance = v;};
    void setEcObjMinClusterSize(int v) {ecObjMinClusterSize = v;};
    void setEcObjMaxClusterSize(int v) {ecObjMaxClusterSize = v;};

    float getZAxisFilterMin() {return zAxisFilterMin;};
    float getZAxisFilterMax() {return zAxisFilterMax;};
    float getDownsampleLeafSize() {return downsampleLeafSize;};
    int getPlaneMaxIterations() {return planeMaxIterations;};
    double getPlaneDistanceThreshold() {return planeDistanceThreshold;};
    double getEcClusterTolerance() {return ecClusterTolerance;};
    int getEcMinClusterSize() {return ecMinClusterSize;};
    int getEcMaxClusterSize() {return ecMaxClusterSize;};
    int getPrismZMin() {return prismZMin;};
    int getPrismZMax() {return prismZMax;};
    double getEcObjClusterTolerance() {return ecObjClusterTolerance;};
    int getEcObjMinClusterSize() {return ecObjMinClusterSize;};
    int getEcObjMaxClusterSize() {return ecObjMaxClusterSize;};

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
    // extract object cluster parameters
    double ecClusterTolerance;
    int ecMinClusterSize;
    int ecMaxClusterSize;
    double prismZMin;
    double prismZMax;
    // extract objects parameters
    double ecObjClusterTolerance;
    int ecObjMinClusterSize;
    int ecObjMaxClusterSize;

    // ID counter for the perceived objects
    int objectID;
    // Buffer for the last perceived objects
    std::vector<PerceivedObject> perceivedObjects;
    
    // Mutex for buffer locking
    boost::signals2::mutex mutex;

    // debug var for time profiling
    bool debug;
    // log time for profiling
    void logTime(boost::posix_time::ptime s, boost::posix_time::ptime e, std::string text);
  };
}

#endif
