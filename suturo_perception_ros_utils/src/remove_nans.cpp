#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>

int
main (int argc, char** argv)
{
  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Usage: input_file_path.pcd output_file_path.pcd\n";
    exit (-1);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[filenames.at(0)], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input file\n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from input pcd" << std::endl;
  //remove NAN points from the cloud
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

  // write pcd
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << argv[filenames.at(1)];
  writer.write(ss.str(), *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points" << std::endl;
  return (0);
}
