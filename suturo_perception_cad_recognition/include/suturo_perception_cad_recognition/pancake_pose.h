#ifndef SUTURO_PERCEPTION_PANCAKE_POSE
#define SUTURO_PERCEPTION_PANCAKE_POSE

/*
 * This node tries is part of a pipeline
 * to align a CAD model to a (partial) pointcloud
 * of a segmented object
 *
 * The CAD model has to be subsampled as a Pointcloud and be
 * passed to this node.
 * You can subsample a CAD model with CloudCompare (http://www.danielgm.net/cc/)
 * Future releases of this software may automate this step.
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <boost/program_options.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <suturo_perception_utils.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <suturo_perception_match_cuboid/cuboid_matcher.h>

using namespace boost;
using namespace std;

class PancakePose
{

  protected:
    Eigen::Vector4f _table_normal;

  public:
  // Attributes
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _model_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_model;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_object;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_object_s1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_object_s2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_object_s3;
    std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > rotations_;
    std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > translations_;

    PancakePose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud, Eigen::Vector4f table_normal)
    {
    }
    // Eigen::Matrix<float, 4, 4>  getTransformation(); // Available after execution

    // Methods
    Eigen::Matrix< float, 4, 4 > rotateAroundCrossProductOfNormals(
        Eigen::Vector3f base_normal,
        Eigen::Vector3f normal_to_rotate,
        bool store_transformation=false);

    Cuboid computeCuboidFromBorderPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points);

    void computeCuboidCornersWithMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points);

    Eigen::Matrix< float, 4, 4> getTranslationMatrix(
        float x, float y, float z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr execute();

    Eigen::Matrix<float, 4, 4>  getRotation();

    Eigen::Matrix<float, 4, 4>  getTranslation();

    Eigen::Quaternionf getOrientation(); 
};
#endif

