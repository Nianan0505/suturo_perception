#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{
  // vector of filename indices
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 1)
  {
    std::cout << "Usage: input_file_path.pcd\n";
    exit (-1);
  }
  std::string input_filename = argv[filenames.at(0)];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_filename, *original_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }
  std::cout << "Loaded "
            << original_cloud->width * original_cloud->height
            << " data points from input pcd" << std::endl;

  // Copy the original cloud to input cloud, which can be modified later during plane extraction
  pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*original_cloud, *input_cloud);

  // Estimate normals on the original file
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (input_cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);


  // Fit planes with RANSAC
  // and store their points and coefficients
  std::vector<pcl::ModelCoefficients::Ptr> vecPlaneCoefficients;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vecPlanePoints;
  // Running index for the plane vectors
  int planeIdx = 0;

  // For each Extraction
  vecPlanePoints.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>() ));
  vecPlaneCoefficients.push_back(pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients));

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.005);

  seg.setInputCloud (input_cloud);
  seg.segment (*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers
  extract.setInputCloud (input_cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*final);

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*input_cloud);
  // cloud_filtered.swap (cloud_f);

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //
  // Store the possible different colors for the segmented planes
  std::vector<std::vector<int> > color_sequence;
  std::vector<int> green;
  green.push_back(0);
  green.push_back(255);
  green.push_back(0);
  std::vector<int> red;
  green.push_back(255);
  green.push_back(0);
  green.push_back(0);
  std::vector<int> blue;
  green.push_back(0);
  green.push_back(0);
  green.push_back(255);
  color_sequence.push_back(green);
  color_sequence.push_back(red);
  color_sequence.push_back(blue);
  int color_idx = 0;

  pcl::visualization::PCLVisualizer viewer;

  // Visualize original pointcloud
  int v1(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.addCoordinateSystem(1.0,v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(original_cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (original_cloud, rgb, "sample cloud1", v1);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud1");
  // viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (input_cloud, cloud_normals1, 10, 0.05, "normals1", v1);

  // Visualize the found inliers
  int v2(1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer.addCoordinateSystem(1.0,v2);
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_f(final);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (final, color_sequence.at(color_idx).at(0), color_sequence.at(color_idx).at(1), color_sequence.at(color_idx).at(2));
  viewer.addPointCloud<pcl::PointXYZRGB> (final, single_color, "sample cloud2", v2);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud2");
  viewer.addPlane (*coefficients, "plane", v2);
  viewer.spin();
  return (0);
}
