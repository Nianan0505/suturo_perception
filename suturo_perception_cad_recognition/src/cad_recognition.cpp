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
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <suturo_perception_utils.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <suturo_perception_match_cuboid/cuboid_matcher.h>

namespace po = boost::program_options;
using namespace boost;
using namespace std;

typedef pcl::FPFHSignature33 EstimationFeature;

class InitialAlignment
{
  public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _model_cloud;

    InitialAlignment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud) : _cloud_in(cloud_in), _model_cloud(model_cloud)
    {
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr execute(); // Return an initially aligned cloud
    Eigen::Matrix<float, 4, 4>  getTransformation(); // Available after execution
};

class RANSACInitialAlignment : public InitialAlignment
{

  protected:
      pcl::PointCloud<pcl::Normal>::Ptr _normals1;
      pcl::PointCloud<pcl::Normal>::Ptr _normals2;
      pcl::PointCloud<EstimationFeature>::Ptr _features1;
      pcl::PointCloud<EstimationFeature>::Ptr _features2;

      void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr output_normals){
        pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMethod(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud(cloud);
        norm_est.setSearchMethod(searchMethod);
        norm_est.setRadiusSearch(0.02);
        norm_est.compute(*output_normals); 
      }

      // Take a input cloud (param: cloud)
      // and compute the corresponding normals and features (in this case, FPFH).
      void estimateFeaturesAndNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr output_normals,pcl::PointCloud<pcl::FPFHSignature33>::Ptr output_features  ){
        estimateNormals(cloud,output_normals);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMethod(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(cloud);
        fpfh_est.setInputNormals(output_normals);
        fpfh_est.setSearchMethod(searchMethod);
        fpfh_est.setRadiusSearch(0.02);
        fpfh_est.compute(*output_features);
      }

  public:
    RANSACInitialAlignment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud) : InitialAlignment(cloud_in,model_cloud)
    {
      // Estimate the normals and features for the initial estimate that will be done later
      _normals1 = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
      _normals2 = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
      _features1 = pcl::PointCloud<EstimationFeature>::Ptr (new pcl::PointCloud<EstimationFeature>);
      _features2 = pcl::PointCloud<EstimationFeature>::Ptr (new pcl::PointCloud<EstimationFeature>);
    }

    // You MUST call this function to compute the necessary features first
    void computeFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud)
    {
      estimateFeaturesAndNormals(cloud_in, _normals1, _features1);
      estimateFeaturesAndNormals(model_cloud, _normals2, _features2);
    }

    // @override
    pcl::PointCloud<pcl::PointXYZ>::Ptr execute()
    {
      // Compute a rough alignment
      pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, EstimationFeature> sac_ia;
      sac_ia.setMinSampleDistance(0.05f);
      // sac_ia.setMaxCorrespondenceDistance(0.01f*0.01f);
      sac_ia.setMaxCorrespondenceDistance(0.03f*0.03f); // works pretty well
      // sac_ia.setMaxCorrespondenceDistance(0.08f*0.08f);
      sac_ia.setMaximumIterations(1500);

      sac_ia.setInputCloud(_model_cloud);
      sac_ia.setSourceFeatures(_features2);
      sac_ia.setInputTarget(_cloud_in);
      sac_ia.setTargetFeatures(_features1);

      pcl::PointCloud<pcl::PointXYZ>::Ptr model_initial_aligned (new pcl::PointCloud<pcl::PointXYZ>);
      sac_ia.align(*model_initial_aligned);
      return model_initial_aligned;
    }
};

class TableInitialAlignment : public InitialAlignment
{
  protected:
    Eigen::Vector4f _table_normal;

  public:
  TableInitialAlignment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud, Eigen::Vector4f table_normal) : InitialAlignment(cloud_in,model_cloud), _table_normal(table_normal)
  {

  }
  
  // Attributes
  pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_model;

  // Methods
  Eigen::Matrix< float, 4, 4 > rotateAroundCrossProductOfNormals(
      Eigen::Vector3f base_normal,
      Eigen::Vector3f normal_to_rotate)
  {
    normal_to_rotate *= -1; // The model is standing upside down, rotate the normal by 180 DEG
    float costheta = normal_to_rotate.dot(base_normal) / (normal_to_rotate.norm() * base_normal.norm() );

    Eigen::Vector3f axis;
    Eigen::Vector3f firstAxis = normal_to_rotate.cross(base_normal);
    firstAxis.normalize();
    axis=firstAxis;
    float c = costheta;
    std::cout << "rotate COSTHETA: " << acos(c) << " RAD, " << ((acos(c) * 180) / M_PI) << " DEG" << std::endl;
    float s = sqrt(1-c*c);
    float CO = 1-c;

    float x = axis(0);
    float y = axis(1);
    float z = axis(2);

    Eigen::Matrix< float, 4, 4 > rotationBox;
    rotationBox(0,0) = x*x*CO+c;
    rotationBox(1,0) = y*x*CO+z*s;
    rotationBox(2,0) = z*x*CO-y*s;

    rotationBox(0,1) = x*y*CO-z*s;
    rotationBox(1,1) = y*y*CO+c;
    rotationBox(2,1) = z*y*CO+x*s;

    rotationBox(0,2) = x*z*CO+y*s;
    rotationBox(1,2) = y*z*CO-x*s;
    rotationBox(2,2) = z*z*CO+c;
    // Translation vector
    rotationBox(0,3) = 0;
    rotationBox(1,3) = 0;
    rotationBox(2,3) = 0;

    // The rest of the 4x4 matrix
    rotationBox(3,0) = 0;
    rotationBox(3,1) = 0;
    rotationBox(3,2) = 0;
    rotationBox(3,3) = 1;

    // if(store_transformation)
    //   transformations_.push_back(rotationBox);

    return rotationBox;
  }

  Cuboid computeCuboidFromBorderPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points)
  {
    Cuboid c;

    // Get the "width": (minx,miny) -> (maxx,miny)
    c.length1 = pcl::distances::l2(corner_points->points.at(0).getVector4fMap(),corner_points->points.at(2).getVector4fMap());
    // Get the "height": (minx,miny) -> (minx,maxy)
    c.length2 = pcl::distances::l2(corner_points->points.at(0).getVector4fMap(),corner_points->points.at(3).getVector4fMap());
    // Get the "depth": (minx,miny,minz) -> (minx,maxy,maxz)
    c.length3 = pcl::distances::l2(corner_points->points.at(0).getVector4fMap(),corner_points->points.at(4).getVector4fMap());

    c.volume = c.length1 * c.length2 * c.length3;
    // Eigen::Vector4f centroid;
    // CuboidMatcher::computeCentroid(corner_points, centroid);
    // c.center = getVector3fFromVector4f(centroid);
    c.corner_points = corner_points;
    return c;
  }

  void computeCuboidCornersWithMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points)
  {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
    // Compute the bounding box edge points
    pcl::PointXYZRGB pt1;
    pt1.x = min_pt.x; pt1.y = min_pt.y; pt1.z = min_pt.z;

    pcl::PointXYZRGB pt2;
    pt2.x = max_pt.x; pt2.y = max_pt.y; pt2.z = max_pt.z;
    pcl::PointXYZRGB pt3;
    pt3.x = max_pt.x,pt3.y = min_pt.y,pt3.z = min_pt.z;
    pcl::PointXYZRGB pt4;
    pt4.x = min_pt.x,pt4.y = max_pt.y,pt4.z = min_pt.z;
    pcl::PointXYZRGB pt5;
    pt5.x = min_pt.x,pt5.y = min_pt.y,pt5.z = max_pt.z;

    pcl::PointXYZRGB pt6;
    pt6.x = min_pt.x,pt6.y = max_pt.y,pt6.z = max_pt.z;
    pcl::PointXYZRGB pt7;
    pt7.x = max_pt.x,pt7.y = max_pt.y,pt7.z = min_pt.z;
    pcl::PointXYZRGB pt8;
    pt8.x = max_pt.x,pt8.y = min_pt.y,pt8.z = max_pt.z;

    corner_points->push_back(pt1);
    corner_points->push_back(pt2);
    corner_points->push_back(pt3);
    corner_points->push_back(pt4);
    corner_points->push_back(pt5);
    corner_points->push_back(pt6);
    corner_points->push_back(pt7);
    corner_points->push_back(pt8);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr execute()
  {
    // Get the dimensions of the object
    // Rotate the model upwards, to get the proper dimensions
    _upwards_model = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix< float, 4, 4 > upwardRotationBox = 
      rotateAroundCrossProductOfNormals(Eigen::Vector3f(0,-1,0), Eigen::Vector3f(0,0,1));

    pcl::transformPointCloud (*_model_cloud, *_upwards_model, upwardRotationBox);
    // Get the (square) dimensions with min max 3d
    pcl::PointXYZ min_pt, max_pt;
    // pcl::getMinMax3D(*_upwards_model, min_pt, max_pt);
    // CuboidMatcher cm;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points (new pcl::PointCloud<pcl::PointXYZRGB>);
    computeCuboidCornersWithMinMax3D(_upwards_model, corner_points);
    Cuboid c = 
      computeCuboidFromBorderPoints(corner_points);
    float model_height = c.length2;
    float model_width  = c.length1;
    float model_depth  = c.length3;
    std::cout << "Dimensions of the object: ";
    std::cout << model_height << " "; 
    std::cout << model_width << " "; 
    std::cout << model_depth << std::endl; 

    // Rotate the object first
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3f table_normal(
        _table_normal[0],
        _table_normal[1],
        _table_normal[2]);

    Eigen::Matrix< float, 4, 4 > rotationBox = 
      // rotateAroundCrossProductOfNormals( Eigen::Vector3f(0,0,1), table_normal );
      rotateAroundCrossProductOfNormals(table_normal, Eigen::Vector3f(0,0,1));

    pcl::transformPointCloud (*_model_cloud, *transformed_cloud, rotationBox);   

    // Compute the centroids of both clouds and bring them closer together
    // for an rough initial alignment.
    
    Eigen::Vector4f input_cloud_centroid, model_cloud_centroid, diff_of_centroids;
    pcl::compute3DCentroid(*_cloud_in, input_cloud_centroid); 
    pcl::compute3DCentroid(*transformed_cloud, model_cloud_centroid); 
    diff_of_centroids = input_cloud_centroid - model_cloud_centroid;
    Eigen::Affine3f transform = pcl::getTransformation(diff_of_centroids[0],
        diff_of_centroids[1], diff_of_centroids[2],0,0,0);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud,transform);

    return transformed_cloud;
  }

  // Eigen::Matrix<float, 4, 4>  getTransformation(); // TO IMPLEMENT
};


void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr output_normals){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMethod(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setInputCloud(cloud);
  norm_est.setSearchMethod(searchMethod);
  norm_est.setRadiusSearch(0.02);
  norm_est.compute(*output_normals); 
}

// Take a input cloud (param: cloud)
// and compute the corresponding normals and features (in this case, FPFH).
void estimateFeaturesAndNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr output_normals,pcl::PointCloud<pcl::FPFHSignature33>::Ptr output_features  ){
  estimateNormals(cloud,output_normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMethod(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud(cloud);
  fpfh_est.setInputNormals(output_normals);
  fpfh_est.setSearchMethod(searchMethod);
  fpfh_est.setRadiusSearch(0.02);
  fpfh_est.compute(*output_features);

}
// 
// void estimateFeaturesAndNormalsSHOT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr output_normals,pcl::PointCloud<pcl::FPFHSignature33>::Ptr output_features  ){
//   estimateNormals(cloud,output_normals);
// 
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMethod(new pcl::search::KdTree<pcl::PointXYZ>);
//   pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
//   fpfh_est.setInputCloud(cloud);
//   fpfh_est.setInputNormals(output_normals);
//   fpfh_est.setSearchMethod(searchMethod);
//   fpfh_est.setRadiusSearch(0.02);
//   fpfh_est.compute(*output_features);
// 
// }
void drawNormalizedVector(pcl::visualization::PCLVisualizer &viewer, Eigen::Vector3f origin, Eigen::Vector3f vector, std::string identifier, int &viewport)
{
  pcl::PointXYZ p_origin(origin[0], origin[1], origin[2] );
  pcl::PointXYZ p_vector(vector[0], vector[1], vector[2] );

  viewer.addLine<pcl::PointXYZ> (p_origin, p_vector, identifier, viewport);
}

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
  // Initial Alignment
  // RANSACInitialAlignment ria(input_cloud_voxeled, model_cloud_voxeled);
  Eigen::Vector4f table_normal(0.0118185, 0.612902, 0.79007, -0.917831);
  TableInitialAlignment ria(input_cloud_voxeled, model_cloud_voxeled, table_normal);
  // 0.0118185 0.612902 0.79007 -0.917831  // pancake normal
  // ria.computeFeatures(input_cloud, model_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_initial_aligned = ria.execute();

  input_cloud = input_cloud_voxeled;
  model_cloud = model_cloud_voxeled;


  // TODO
  // - Use the table normal to fit the rotation
  // - Use the highest and lowest point to align the Pointcloud better in height


  // Refine the result with ICP

  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(input_cloud);
  // icp.setRANSACOutlierRejectionThreshold(0.10f);
  // icp.setInputCloud(model_initial_aligned);
  icp.setInputTarget(model_initial_aligned);
  // icp.setInputTarget(input_cloud);
  icp.setEuclideanFitnessEpsilon (0.000001f);
  // icp.setMaxCorrespondenceDistance (0.55);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  l.logTime(start,end,"Initial Alignment and ICP");





  pcl::visualization::PCLVisualizer viewer;
  int v1,v2,v3,v4;
  // viewer.createViewPort(0.0,0, 0.25,1, v1 );
  // viewer.addText("Input Cloud", 0.1, 0.1 , "input_cloud_text_id", v1 );
  viewer.createViewPort(0.0,0, 0.33,1, v2 );
  viewer.addText("Model vs. Input Cloud", 0.1, 0.1 , "model_cloud_text_id", v2 );
  viewer.createViewPort(0.33,0, 0.66  ,1, v3 );
  viewer.addText("Initial Aligned Cloud", 0.1, 0.1 , "initial_aligned_cloud_text_id", v3 );
  viewer.createViewPort(0.66,0, 1  ,1, v4 );
  viewer.addText("Refined Aligned Cloud", 0.1, 0.1 , "refined_aligned_cloud_text_id", v4 );

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(input_cloud, 0, 255, 0);
  // viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color, "input_cloud_id",v1);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(model_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upwards_color(ria._upwards_model, 255, 125, 0);
  viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color, "input_cloud_id2",v2);
  viewer.addPointCloud<pcl::PointXYZ> (model_cloud, red_color, "model_cloud_id",v2);
  viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_color",v2);
  drawNormalizedVector(viewer, Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0,1), "norm_at_origin", v2);
  drawNormalizedVector(viewer, Eigen::Vector3f(0,0,0),
      Eigen::Vector3f(table_normal[0],table_normal[1],table_normal[2])
      ,"table_normal", v2);
  viewer.addCoordinateSystem(0.5,v2);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color2(model_initial_aligned, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ> (model_initial_aligned, red_color2,"initial_aligned_cloud_id",v3);
  viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color,"original_cloud_vs_initial_aligned_id",v3);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color3(model_initial_aligned, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow_color(Final, 255, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (Final, yellow_color,"refined_aligned_cloud_id",v4);
  viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color,"original_cloud_vs_refined_aligned_id",v4);
  viewer.addPointCloud<pcl::PointXYZ> (model_initial_aligned, red_color3, "initial_aligned_vs_refined_aligned_id",v4);



  viewer.spin();
  // Spin
  // ros::spin ();

  return 0;
};
  // Estimate the normals and features for the initial estimate that will be done later
  // pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
  // pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
  // pcl::PointCloud<EstimationFeature>::Ptr features1(new pcl::PointCloud<EstimationFeature>);
  // pcl::PointCloud<EstimationFeature>::Ptr features2(new pcl::PointCloud<EstimationFeature>);

  // estimateFeaturesAndNormals(input_cloud, normals1, features1);
  // estimateFeaturesAndNormals(model_cloud, normals2, features2);

  // Compute a rough alignment
  /*
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, EstimationFeature> sac_ia;
  sac_ia.setMinSampleDistance(0.05f);
  // sac_ia.setMaxCorrespondenceDistance(0.01f*0.01f);
  sac_ia.setMaxCorrespondenceDistance(0.03f*0.03f); // works pretty well
  // sac_ia.setMaxCorrespondenceDistance(0.08f*0.08f);
  sac_ia.setMaximumIterations(1500);

  sac_ia.setInputCloud(model_cloud);
  sac_ia.setSourceFeatures(features2);
  sac_ia.setInputTarget(input_cloud);
  sac_ia.setTargetFeatures(features1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_initial_aligned (new pcl::PointCloud<pcl::PointXYZ>);
  sac_ia.align(*model_initial_aligned);
  */

