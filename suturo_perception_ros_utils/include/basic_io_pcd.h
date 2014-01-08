#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>

class BasicIOPCD {
  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
  public:
    BasicIOPCD(int argc, char** argv);
    void execute();
};
