#ifndef SUTURO_PERCEPTION_POINT_CLOUD_OPERATIONS_H
#define SUTURO_PERCEPTION_POINT_CLOUD_OPERATIONS_H

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include "perceived_object.h"
#include "opencv2/core/core.hpp"
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>

#include "suturo_perception_utils.h"
#include "color_analysis.h"
#include "roi.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

namespace suturo_perception_lib
{
  class PointCloudOperations
  {
    public:
      PointCloudOperations();

      void removeNans(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles);
      static void filterZAxis(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
          pcl::PassThrough<pcl::PointXYZRGB> &pass,
          float zAxisFilterMin, float zAxisFilterMax);
      static void downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, float downsampleLeafSize);
      // void fitPlanarModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
  };
}

#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
