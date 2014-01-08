#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include "basic_io_pcd.h"

class RemoveNansIO : public BasicIOPCD
{
  public:
    RemoveNansIO(int argc, char** argv) : BasicIOPCD(argc, argv)
    {
    }
    void execute()
    {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*input_cloud_,*output_cloud_, indices);
    }
};

int
main (int argc, char** argv)
{
  RemoveNansIO rio(argc,argv);
  rio.execute();
  rio.write_pcd();
  return (0);
}
