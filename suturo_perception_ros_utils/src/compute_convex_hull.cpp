#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include "basic_io_pcd.h"

class ComputeHullIO : public BasicIOPCD
{
  public:
    ComputeHullIO(int argc, char** argv) : BasicIOPCD(argc, argv)
    {
    }
    void execute()
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());

      pcl::ConvexHull<pcl::PointXYZRGB> hull;
      hull.setInputCloud(input_cloud_);
      hull.setDimension(3);
      hull.reconstruct (*output_cloud_);
    }
};

int
main (int argc, char** argv)
{
  ComputeHullIO cio(argc,argv);
  cio.execute();
  cio.write_pcd();
  return (0);
}
