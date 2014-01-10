#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "basic_io_pcd.h"

class SegmentPlaneIO : public BasicIOPCD
{
  public:
    SegmentPlaneIO(int argc, char** argv) : BasicIOPCD(argc, argv)
    {
    }
    void execute()
    {
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);

      seg.setInputCloud (input_cloud_);
      seg.segment (*inliers, *coefficients);

      // draw the cloud and the box
      pcl::visualization::PCLVisualizer viewer;
      viewer.addPointCloud(input_cloud_);
      viewer.addPlane (*coefficients, "plane");
      viewer.spin();
    }
};

int
main (int argc, char** argv)
{
  SegmentPlaneIO sio(argc,argv);
  sio.execute();
  return (0);
}


