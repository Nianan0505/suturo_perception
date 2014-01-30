#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include "basic_io_pcd.h"

class SmoothSurfaceIO : public BasicIOPCD
{
  public:
    SmoothSurfaceIO(int argc, char** argv) : BasicIOPCD(argc, argv)
    {
    }
    void execute()
    {
      // Create a KD-Tree
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

      pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
      // Set parameters
      mls.setInputCloud (input_cloud_);
      mls.setPolynomialFit (true);
      mls.setSearchMethod (tree);
      mls.setSearchRadius (0.03);

      // Reconstruct
      mls.process (*output_cloud_);
    }
};

int
main (int argc, char** argv)
{
  SmoothSurfaceIO sio(argc,argv);
  sio.execute();
  sio.write_pcd();
  return (0);
}
