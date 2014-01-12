#include <iostream>
#include <cmath>
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
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

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

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);

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
  for (int i = 0; i < 2; i++) // Hack, extract two planes
  {
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
    seg.setDistanceThreshold (0.005); // Tolerance is 0.5 cm

    seg.setInputCloud (input_cloud);
    // seg.segment (*inliers, *coefficients);
    seg.segment (*inliers, *vecPlaneCoefficients.at(planeIdx));

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Extract the inliers
    extract.setInputCloud (input_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*vecPlanePoints.at(planeIdx));

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*input_cloud);
    // cloud_filtered.swap (cloud_f);
    planeIdx ++ ;
  }
  
  for (int i = 0; i < vecPlanePoints.size(); i++)
  {
    for (int j = 0; j < vecPlanePoints.size(); j++)
    {
      if(i==j) continue;
      std::cout << "Angle between Normal " << i << " and Normal " << j;
      Eigen::Vector3f v1(
          vecPlaneCoefficients.at(i)->values.at(0),
          vecPlaneCoefficients.at(i)->values.at(1),
          vecPlaneCoefficients.at(i)->values.at(2)
      );


      Eigen::Vector3f v2(
          vecPlaneCoefficients.at(j)->values.at(0),
          vecPlaneCoefficients.at(j)->values.at(1),
          vecPlaneCoefficients.at(j)->values.at(2)
      );

      float dotproduct = v1.dot(v2);
      std::cout << ": " << acos(dotproduct) << " RAD, " << ((acos(dotproduct) * 180) / M_PI) << " DEG";
      std::cout << std::endl;

    }
  }
  std::cout << "Algorithm done. Visualize results" << std::endl;
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
  red.push_back(255);
  red.push_back(0);
  red.push_back(0);
  std::vector<int> blue;
  blue.push_back(0);
  blue.push_back(0);
  blue.push_back(255);
  color_sequence.push_back(green);
  color_sequence.push_back(red);
  color_sequence.push_back(blue);
  
  /*
  Eigen::Matrix< float, 4, 4 > rotationBox;
  // Rotate 90° DEG = PI/2 RAD around X
  rotationBox(0,0) = 1;
  rotationBox(1,0) = 0;
  rotationBox(2,0) = 0;

  rotationBox(0,1) = 0;
  rotationBox(1,1) = cos(M_PI/2);
  rotationBox(2,1) = sin(M_PI/2);

  rotationBox(0,2) = 0;
  rotationBox(1,2) = -sin(M_PI/2);
  rotationBox(2,2) = cos(M_PI/2);

  // Translation vector
  rotationBox(0,3) = 0;
  rotationBox(1,3) = 0;
  rotationBox(2,3) = 0;

  // The rest of the 4x4 matrix
  rotationBox(3,0) = 0;
  rotationBox(3,1) = 0;
  rotationBox(3,2) = 0;
  rotationBox(3,3) = 1;
*/
  // float rot_x     = 0;
  // float rot_y     = 0;
  // float rot_z     = 0;
  // float rot_roll  = 1.63; // Rotate around X axis
  // float rot_pitch = 0;
  // float rot_yaw   = 0;
  // Eigen::Affine3f t;


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Rotate with Affine3f and euler angles
  /*
  pcl::getTransformation (rot_x, rot_y, rot_z, rot_roll, rot_pitch, rot_yaw, t);
  pcl::transformPointCloud (*original_cloud, *rotated_cloud, t);   
  */
  // pcl::transformPointCloud (*original_cloud, *rotated_cloud, rotationBox);   

  pcl::visualization::PCLVisualizer viewer;
  // Visualize original pointcloud
  int v1(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.addCoordinateSystem(1.0,v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(original_cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (original_cloud, rgb, "sample cloud1", v1);
  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud1");
  // viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (input_cloud, cloud_normals1, 10, 0.05, "normals1", v1);

  // Visualize the found inliers
  int v2(1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer.addCoordinateSystem(1.0,v2);

  // For each segmented plane
  for (int i = 0; i < vecPlanePoints.size(); i++) {
    // Select a color from the color_sequence vector for the discovered plane
    int color_idx = i % 3;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (vecPlanePoints.at(i), color_sequence.at(color_idx).at(0), color_sequence.at(color_idx).at(1), color_sequence.at(color_idx).at(2));

    std::stringstream cloud_name("plane_cloud_");
    cloud_name << i;
    std::stringstream plane_name("model_plane_");
    plane_name << i;
    viewer.addPointCloud<pcl::PointXYZRGB> (vecPlanePoints.at(i), single_color, cloud_name.str(), v2);
    viewer.addPlane (*vecPlaneCoefficients.at(i), plane_name.str(), v2);

    

    // Display the plane normal with the camera origin as the origin
    {
      pcl::PointXYZRGB origin;
      origin.x=0;
      origin.y=0;
      origin.z=0;

      // Get the normal from the plane
      pcl::PointXYZRGB dest;
      dest.x = vecPlaneCoefficients.at(i)->values.at(0);
      dest.y = vecPlaneCoefficients.at(i)->values.at(1);
      dest.z = vecPlaneCoefficients.at(i)->values.at(2);


      // 

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      origin_cloud->push_back(origin);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
      // Project the origin point to the plane
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud (origin_cloud);
      proj.setModelCoefficients (vecPlaneCoefficients.at(i));
      proj.filter (*origin_cloud_projected);
      if(origin_cloud_projected->points.size()!=1)
      {
        std::cout << "Projection fail" << std::endl;
        return 0;
      }

      std::stringstream plane_name("norm_plane_");
      plane_name << i;
      viewer.addLine(origin_cloud_projected->points.at(0),dest, color_sequence.at(color_idx).at(0), color_sequence.at(color_idx).at(1), color_sequence.at(color_idx).at(2), plane_name.str(),v2);
    }

  }

  //
  // ROTATE THE OBJECT to align it with the x-y and the x-z plane
  // TODO: dynamic
  //
  {



    // M
    Eigen::Vector3f plane_normal(dest.x, dest.y, dest.z);

    // Compute the necessary rotation to align a face of the object with the camera's
    // imaginary image plane
    // N
    Eigen::Vector3f camera_normal;
    camera_normal(0)=0;
    camera_normal(1)=0;
    camera_normal(2)=1;
    // Eigen::Vector3f camera_normal_normalized = camera_normal.normalized();
    float costheta = plane_normal.dot(camera_normal) / (plane_normal.norm() * camera_normal.norm() );

    Eigen::Vector3f axis;
    axis = plane_normal.cross(camera_normal) / (plane_normal.norm() * camera_normal.norm() );
    float c = costheta;
    float s = sqrt(1-c*c);
    float CO = 1-c;


    Eigen::Matrix< float, 4, 4 > rotationBox;
    float x = axis(0);
    float y = axis(1);
    float z = axis(2);
    // Rotate 90° DEG = PI/2 RAD around X

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

    // pcl::transformPointCloud (*original_cloud, *rotated_cloud, rotationBox);   
    pcl::transformPointCloud (*original_cloud, *rotated_cloud, rotationBox);   
    // Draw the rotated object
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_r(rotated_cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (rotated_cloud, rgb_r, "rotated_cloud", v2);
  }

  // // Rotate cuboid to align top with x-z plane
  // {
  //   pcl::PointXYZRGB dest;
  //   dest.x = vecPlaneCoefficients.at(1)->values.at(0);
  //   dest.y = vecPlaneCoefficients.at(1)->values.at(1);
  //   dest.z = vecPlaneCoefficients.at(1)->values.at(2);
  //   // M
  //   Eigen::Vector3f plane_normal(dest.x, dest.y, dest.z);

  //   // Compute the necessary rotation to align a face of the object with the camera's
  //   // imaginary image plane
  //   // N
  //   Eigen::Vector3f camera_normal;
  //   camera_normal(0)=0; // x
  //   camera_normal(1)=1; // y
  //   camera_normal(2)=0; // z
  //   // Eigen::Vector3f camera_normal_normalized = camera_normal.normalized();
  //   float costheta = plane_normal.dot(camera_normal) / (plane_normal.norm() * camera_normal.norm() );

  //   Eigen::Vector3f axis;
  //   axis = plane_normal.cross(camera_normal) / (plane_normal.norm() * camera_normal.norm() );
  //   float c = costheta;
  //   float s = sqrt(1-c*c);
  //   float CO = 1-c;


  //   Eigen::Matrix< float, 4, 4 > rotationBox;
  //   float x = axis(0);
  //   float y = axis(1);
  //   float z = axis(2);
  //   // Rotate 90° DEG = PI/2 RAD around X

  //   rotationBox(0,0) = x*x*CO+c;
  //   rotationBox(1,0) = y*x*CO+z*s;
  //   rotationBox(2,0) = z*x*CO-y*s;

  //   rotationBox(0,1) = x*y*CO-z*s;
  //   rotationBox(1,1) = y*y*CO+c;
  //   rotationBox(2,1) = z*y*CO+x*s;

  //   rotationBox(0,2) = x*z*CO+y*s;
  //   rotationBox(1,2) = y*z*CO-x*s;
  //   rotationBox(2,2) = z*z*CO+c;

  //   // Translation vector
  //   rotationBox(0,3) = 0;
  //   rotationBox(1,3) = 0;
  //   rotationBox(2,3) = 0;

  //   // The rest of the 4x4 matrix
  //   rotationBox(3,0) = 0;
  //   rotationBox(3,1) = 0;
  //   rotationBox(3,2) = 0;
  //   rotationBox(3,3) = 1;

  //   pcl::transformPointCloud (*rotated_cloud, *rotated_cloud, rotationBox);   
  //   // Draw the rotated object
  //   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_r(rotated_cloud);
  //   viewer.addPointCloud<pcl::PointXYZRGB> (rotated_cloud, rgb_r, "rotated_cloud", v2);
  // }

  // 
  // Create a plane x-y plane that originates in the kinect camera frame
  // 
  pcl::ModelCoefficients::Ptr kinect_plane_coefficients (new pcl::ModelCoefficients);
  // Let the Normal of the plane point along the z-Axis
  kinect_plane_coefficients->values.push_back(0); // x
  kinect_plane_coefficients->values.push_back(0); // y
  kinect_plane_coefficients->values.push_back(1); // z 
  kinect_plane_coefficients->values.push_back(0); // d 

  viewer.addPlane (*kinect_plane_coefficients, "kinect_plane", v2);
  viewer.spin();


  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud2");
  return (0);
}

/*
    // Create a convex hull to get the centroid of the object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(original_cloud);
    hull.setDimension(2);
    hull.reconstruct (*hull_points);

    // Centroid calulcation
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*hull_points, centroid);  

    pcl::PointXYZRGB dest;
    dest.x = vecPlaneCoefficients.at(0)->values.at(0);
    dest.y = vecPlaneCoefficients.at(0)->values.at(1);
    dest.z = vecPlaneCoefficients.at(0)->values.at(2);


    Eigen::Matrix< float, 4, 4 > translateBox;
    // Rotate 90° DEG = PI/2 RAD around X

    translateBox(0,0) = 1;
    translateBox(1,0) = 0;
    translateBox(2,0) = 0;

    translateBox(0,1) = 0;
    translateBox(1,1) = 1;
    translateBox(2,1) = 0;

    translateBox(0,2) = 0;
    translateBox(1,2) = 0;
    translateBox(2,2) = 1;

    // Translation vector
    translateBox(0,3) = -centroid(0);
    translateBox(1,3) = -centroid(1);
    translateBox(2,3) = -centroid(2);

    // The rest of the 4x4 matrix
    translateBox(3,0) = 0;
    translateBox(3,1) = 0;
    translateBox(3,2) = 0;
    translateBox(3,3) = 1;

    pcl::transformPointCloud (*original_cloud, *rotated_cloud, translateBox);   
*/
