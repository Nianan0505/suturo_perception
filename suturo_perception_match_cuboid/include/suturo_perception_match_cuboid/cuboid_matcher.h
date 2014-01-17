#ifndef CUBOID_MATCHER_H
#define CUBOID_MATCHER_H

#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/distances.h>
#include <point_cloud_operations.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <suturo_perception_match_cuboid/detected_plane.h>

// This class implements the main functionality of the Cuboid
// Matching process.
// It finds the two dominant planes from a given PointCloud
// and aligns them to the cameras base frame.
// When the cuboid has been aligned, a bounding box will
// be calculated
// By following these transformations backwards, we get the orientation
// and the Cuboid

class CuboidMatcher
{
  public:
    CuboidMatcher();
    // Return a pointer to the detected plane list
    std::vector<DetectedPlane> *getDetectedPlanes();
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    // Set this to true to save every transformed pointcloud
    // which will be generated during the fitting process
    // This is helpful, if you want to visualize the intermediate
    // results to understand the algorithm
    void setSaveIntermediateResults(bool b);

    // This method method checks the angle
    // between v1 and v2.
    // If it's higher than 90 DEG or M_PI/2, a 180 DEG rotated version
    // of v2 will be returned.
    // This helps during the alignment process, to rotate only
    // by small portions
    // If the angle is below 90 DEG, the unmodified v2
    // will be returned
    static Eigen::Vector3f reduceNormAngle(Eigen::Vector3f v1, Eigen::Vector3f v2);

  private:
    bool save_intermediate_results_;
    // The input cloud.
    // This pointcloud will be modified during execution!
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
    std::vector<DetectedPlane> detected_planes_;
    // A list with all used transformation
    // matrices 
    std::vector<Eigen::Matrix< float, 4, 4 > > transformations;
};
#endif
