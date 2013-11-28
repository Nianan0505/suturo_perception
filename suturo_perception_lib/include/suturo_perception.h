#ifndef SUTURO_PERCEPTION_H
#define SUTURO_PERCEPTION_H

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include "perceived_object.h"
#include "opencv2/core/core.hpp"

namespace suturo_perception_lib
{
  class SuturoPerception
  {
    public:

    SuturoPerception();
    void processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
		void processCloudWithProjections(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    std::vector<PerceivedObject> getPerceivedObjects();
    std::vector<cv::Mat> getPerceivedClusterImages();

    // Get the cloud that is the basis for the object extraction
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlaneCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getObjectsOnPlaneCloud();

		void clusterFromProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<cv::Mat> &extracted_images);

    void removeNans(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles);
    void filterZAxis(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
    void downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
    void fitPlanarModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                        pcl::PointIndices::Ptr indices_out);
    void extractObjectCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
                              const pcl::PointIndices::Ptr inliers, 
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
    void extractObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& extractedObjects);
    void getObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& extractedObjects);
    uint32_t getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);

    // debug
    void writeCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects);
		void writeCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects, std::string filename);
		void writeCloudToDisk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, std::string filename);

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

    // dirty hack collision_objects
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> collision_objects;

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
    std::vector<cv::Mat> perceived_cluster_images_;

    // The cloud of the extracted plane in the segmentation process
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_;

    // The cloud of the extracted objects above the plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_on_plane_cloud_;

    // Set this flag to true to write partial pcds
    // while processing a cloud
    bool writer_pcd;

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
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
