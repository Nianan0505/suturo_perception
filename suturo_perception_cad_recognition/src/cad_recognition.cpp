/*
 * This node tries is part of a pipeline
 * to align a CAD model to a (partial) pointcloud
 * of an segmented object
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <suturo_perception_utils.h>

namespace po = boost::program_options;
using namespace boost;
using namespace std;


int main(int argc, char** argv){
  ros::init(argc, argv, "cad_recognition");
  ros::NodeHandle node;
  
  std::string cad_model_pc_filename;
  std::string input_pc_filename;

  // "HashMap" for program parameters
  po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("input-pc,i", po::value<std::string>(&input_pc_filename)->required(), "The  filename of the input pointcloud.")
      ("cad-model-pc,m", po::value<std::string>(&cad_model_pc_filename)->required(), "A pointcloud of the CAD-Model to match. You can get a Pointcloud of your CAD-Model with CloudCompare.")
    ;

    po::positional_options_description p;
    po::store(po::command_line_parser(argc, argv).
    options(desc).positional(p).run(), vm); 

    if (vm.count("help")) {
      std::cout << "Usage: cad_recognition [-t topic]" << endl << endl;
      std::cout << desc << "\n";
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

  }
  catch(std::exception& e)
  {
    std::cout << "Usage: cad_recognition -i input_cloud.pcd -m cad_model_cloud.pcd" << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  boost::posix_time::ptime file_load_start = boost::posix_time::microsec_clock::local_time();
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_pc_filename, *input_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (cad_model_pc_filename, *model_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read cad model file\n");
    exit (-1);
  }
  boost::posix_time::ptime file_load_end = boost::posix_time::microsec_clock::local_time();
  suturo_perception_utils::Logger l("cad_recognition");
  l.logTime(file_load_start,file_load_end,"File loading");

  boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(model_cloud);
  icp.setInputTarget(input_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  l.logTime(start,end,"ICP");
  // Spin
  // ros::spin ();

  return 0;
};
