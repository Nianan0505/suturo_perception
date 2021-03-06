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
#include <suturo_perception_cad_recognition/pancake_pose.h>

namespace po = boost::program_options;
using namespace boost;
using namespace std;

void drawNormalizedVector(pcl::visualization::PCLVisualizer &viewer, Eigen::Vector3f origin, Eigen::Vector3f vector, std::string identifier, int &viewport)
{
  pcl::PointXYZ p_origin(origin[0], origin[1], origin[2] );
  pcl::PointXYZ p_vector(vector[0], vector[1], vector[2] );

  viewer.addLine<pcl::PointXYZ> (p_origin, p_vector, identifier, viewport);
}

int main(int argc, char** argv){
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
  pcl::PolygonMesh::Ptr model_mesh (new pcl::PolygonMesh);

  boost::posix_time::ptime file_load_start = boost::posix_time::microsec_clock::local_time();
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_pc_filename, *input_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (cad_model_pc_filename, *model_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read cad model file (points)\n");
    exit (-1);
  }

  if (pcl::io::loadPolygonFileSTL("test_data/pancake_mix.stl", *model_mesh) == -1) //* load the CAD model file
  {
    PCL_ERROR ("Couldn't read cad model file (mesh)\n");
    exit (-1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_voxeled (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_voxeled (new pcl::PointCloud<pcl::PointXYZ>);

  // Downsample both clouds
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (input_cloud);
  // #define LEAF_SIZE 0.01f
  #define LEAF_SIZE 0.005f
  sor.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  sor.filter (*input_cloud_voxeled);

  sor.setInputCloud (model_cloud);
  sor.filter (*model_cloud_voxeled);

  boost::posix_time::ptime file_load_end = boost::posix_time::microsec_clock::local_time();
  suturo_perception_utils::Logger l("cad_recognition");
  l.logTime(file_load_start,file_load_end,"File loading and voxeling done");

  boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
  Eigen::Vector4f table_normal(0.0118185, 0.612902, 0.79007, -0.917831); // pancake 
  // Eigen::Vector4f table_normal(0.00924593, 0.697689, 0.716341, -0.914689); // pancake 0deg moved
  // Eigen::Vector4f table_normal(0.0102382,0.6985,0.715537,-0.914034); // pancake 0deg moved
  PancakePose ria(input_cloud_voxeled, model_cloud_voxeled, table_normal);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_initial_aligned = ria.execute();

  Eigen::Matrix<float, 4, 4> initial_alignment_rotation = 
    ria.getRotation();

  Eigen::Matrix<float, 4, 4> initial_alignment_translation = 
    ria.getTranslation();

  input_cloud = input_cloud_voxeled;
  model_cloud = model_cloud_voxeled;


  // Refine the result with ICP

  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(ria._upwards_object);
  icp.setInputTarget(ria._upwards_model);
  icp.setEuclideanFitnessEpsilon (0.000001f);
  // icp.setMaxCorrespondenceDistance (0.55);
  // icp.setRANSACOutlierRejectionThreshold(0.10f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  l.logTime(start,end,"Initial Alignment and ICP");


  // Check the result of the calculated transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_initial_transformed (new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix<float, 4, 4> icp_transform = icp.getFinalTransformation();
  Eigen::Matrix<float, 4, 4> icp_transform_inverse = icp.getFinalTransformation().inverse();
  pcl::transformPointCloud(*model_cloud_voxeled, *model_initial_transformed, ria.rotations_.at(0) );
  pcl::transformPointCloud(*model_initial_transformed, *model_initial_transformed,icp_transform_inverse );
  pcl::transformPointCloud(*model_initial_transformed, *model_initial_transformed, initial_alignment_translation );
  pcl::transformPointCloud(*model_initial_transformed, *model_initial_transformed, ria.rotations_.at(1) );


  pcl::visualization::PCLVisualizer viewer;
  int v1,v2,v3,v4;
  // viewer.createViewPort(0.0,0, 0.25,1, v1 );
  // viewer.addText("Input Cloud", 0.1, 0.1 , "input_cloud_text_id", v1 );
  viewer.createViewPort(0.0,0, 0.33,1, v2 );
  viewer.addText("Model vs. Input Cloud - Roughly aligned", 0.1, 0.1 , "model_cloud_text_id", v2 );
  viewer.addCoordinateSystem(0.3,v2);
  // pcl::PointXYZ a(0,1,0);
  // viewer.addSphere(a,0.5,"sphere",v2);
  viewer.createViewPort(0.33,0, 0.66  ,1, v3 );
  viewer.addCoordinateSystem(0.3,v3);
  viewer.addText("ICP", 0.1, 0.1 , "icp_text", v3 );
  viewer.createViewPort(0.66,0, 1  ,1, v4 );
  viewer.addCoordinateSystem(0.3,v4);
  viewer.addText("Pose estimation", 0.1, 0.1 , "pose_text", v4 );

  // Viewport 2
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(input_cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(model_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upwards_color(ria._upwards_model, 255, 125, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upwards_object(ria._upwards_object, 128, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color, "input_cloud_id2",v2);
  viewer.addPointCloud<pcl::PointXYZ> (model_cloud, red_color, "model_cloud_id",v2);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_color",v2);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object, upwards_object, "upwards_object",v2);
  drawNormalizedVector(viewer, Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0,1), "norm_at_origin", v2);
  drawNormalizedVector(viewer, Eigen::Vector3f(0,0,0),
      Eigen::Vector3f(table_normal[0],table_normal[1],table_normal[2])
      ,"table_normal", v2);


  // Viewport 3
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow_color(Final, 255, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (Final, yellow_color,"refined_aligned_cloud_id",v3);
  viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color,"original_cloud_vs_refined_aligned_id",v3);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_model_vs_refined_cloud",v3);

  // Viewport 4
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pink_color(ria._upwards_object_s2, 255, 187, 255);
  // viewer.addPointCloud<pcl::PointXYZ> (Final, yellow_color,"refined_aligned_cloud_id",v3);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_init(model_initial_aligned, 255, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color,"original_cloud_vs_pose",v4);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object_s1, green_color,"upwards object s1",v4);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object_s2, pink_color,"upwards object s2",v4);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object_s3, green_color,"upwards object s3",v4);
  viewer.addPointCloud<pcl::PointXYZ> (model_initial_transformed, red_init, "model_cloud_id_v4",v4);
  // viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_model_vs_refined_cloud",v3);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_color_v4",v4);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object, upwards_object, "upwards_object_v4",v4);

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_icp_transformed (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*ria._upwards_model, *model_icp_transformed,icp_transform_inverse );
  viewer.addPointCloud<pcl::PointXYZ> (model_icp_transformed, "model_icp_transformed",v4);

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_icp_transformed_s2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*ria._upwards_model, *model_icp_transformed_s2,icp_transform_inverse );
  pcl::transformPointCloud(*model_icp_transformed_s2, *model_icp_transformed_s2, ria.translations_.at(0) );
  viewer.addPointCloud<pcl::PointXYZ> (model_icp_transformed_s2, "model_transformed_s2",v4);

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_icp_transformed_s3 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*ria._upwards_model, *model_icp_transformed_s3,icp_transform_inverse );
  pcl::transformPointCloud(*model_icp_transformed_s3, *model_icp_transformed_s3, ria.translations_.at(0) );
  pcl::transformPointCloud(*model_icp_transformed_s3, *model_icp_transformed_s3, ria.translations_.at(1) );
  viewer.addPointCloud<pcl::PointXYZ> (model_icp_transformed_s3, "model_transformed_s3",v4);

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_icp_transformed_s4 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*ria._upwards_model, *model_icp_transformed_s4, 
      ria.rotations_.at(1) *
      ria.translations_.at(1) * ria.translations_.at(0) * icp_transform_inverse );
  viewer.addPointCloud<pcl::PointXYZ> (model_icp_transformed_s4, "model_transformed_s4",v4);

  // Transform object center
  pcl::PointXYZ a(0,0,0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr origin (new pcl::PointCloud<pcl::PointXYZ>);
  origin->push_back(a);
  pcl::transformPointCloud(*origin, *origin, 
      ria.rotations_.at(1) *
      ria.translations_.at(1) * ria.translations_.at(0) * icp_transform_inverse );
  viewer.addSphere(origin->at(0),0.03,"sphere2",v4);

  viewer.spin();

  return 0;
};
