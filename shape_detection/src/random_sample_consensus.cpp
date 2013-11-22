#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int
main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
 
  bool none = false;
  bool cerealBox = false;
  bool box = false;
  bool sphere = false;
  bool cylinder = false;
  bool cone = false;
// populate our PointCloud with points; just a sphere and a plane for testing
// without real data. In addition these are pretty much ''noise-free''

// load a point cloud

if (pcl::io::loadPCDFile<pcl::PointXYZ> ("file1.pcd", *cloud1) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  std::vector<int> inliers;

  // ONLY NEEDED FOR SampleConsensusModelCylinder (and cone)
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_cyl;
  ne_cyl.setInputCloud (cloud1);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne_cyl.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne_cyl.setRadiusSearch (0.03);

  // Compute the features
  ne_cyl.compute (*cloudNormals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*


/**
 *  Initialize the three RANSAC-Models that we try to fit onto the perceived
 *  object's point cloud.
 *  The three different shapes that can be recognized are spheres, cylinders,
 *  and boxes (represented through perpendicular planes).
 */
  // created RandomSampleConsensus object and compute the appropriat model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    modelSphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud1));
  pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr 
      p_modelcustom (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (cloud1));
  pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr 
      modelCylinder (new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> (cloud1));
  modelCylinder->setInputNormals(cloudNormals);
  pcl::SampleConsensusModelCone<pcl::PointXYZ, pcl::Normal>::Ptr 
      modelCone (new pcl::SampleConsensusModelCone<pcl::PointXYZ, pcl::Normal> (cloud1));
  modelCone->setInputNormals(cloudNormals);
    
    /* If the program is run without any arguments the loaded cloud1 is
     * displayed without the RANSAC stuff applied.
     *
     *  -f      Display the generated plane after RANSAC
     *  -sf     Display the generated sphere after RANSAC
     *  -c      Display the loaded pointcloud after RANSAC with planemodel
     *
     * */

  if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (modelSphere);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }
  else if (pcl::console::find_argument (argc, argv, "-c") >= 0)
  {
    //get the number of points in the input cloud
    int pcCount = cloud1->points.size();
   
    // try to ransac-fit a cylinder model onto the point clou, PointNTd
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransacCylinder(modelCylinder); 
    ransacCylinder.setDistanceThreshold (.01);
    ransacCylinder.computeModel();
    ransacCylinder.getInliers(inliers);
    float inCountCylinder = inliers.size();
    float percCylinder = inCountCylinder / pcCount;

    // try to ransac-fit a sphere model onto the point cloud
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransacSphere (modelSphere);
    ransacSphere.setDistanceThreshold (.01);
    ransacSphere.computeModel();
    ransacSphere.getInliers(inliers);
    float inCountSphere = inliers.size();
    float percSphere = inCountSphere / pcCount;
/**
    // try to ransac-fit a sphere model onto the point cloud
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransacCone (modelCone);
    ransacCone.setDistanceThreshold (.01);
    ransacCone.computeModel();
    ransacCone.getInliers(inliers);
    float inCountCone = inliers.size();
    float percCone = inCountCone / pcCount;
**/
/**
 *  The following tries to find a "box" in the point cloud.
 *  To do this, we are trying to find two planes perpendicular to the Y or Z
 *  axis. This then represents a cuboid which most probably in our context is
 *  a box.
 *  If the point cloud is found to contain a box, the kind of box is
 *  estimated. 
 *  If the plane perpendicular to the Z axis contains 3-4x the amount of
 *  points the Y-plane contains, it is propable that the recorded object represented
 *  in this point cloud was a cereal box.
 *  If not, it is classified as a "generic box"
 */

    // find a plane perpendicular to the z axis, accepting points up to 30°
    p_modelcustom->setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
    p_modelcustom->setEpsAngle (0.524); 
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransacBox (p_modelcustom);
    ransacBox.setDistanceThreshold (.01);
    ransacBox.computeModel();
    ransacBox.getInliers(inliers);
    int inCountY = inliers.size();
    std::cout << pcCount << "\n" << inCountY << "\n";
    
    // find a plane perpendicular to the x axis, accepting points up to 30°
    p_modelcustom->setAxis (Eigen::Vector3f (0.0, 1.0, 0.0));
    p_modelcustom->setEpsAngle (0.524);
    ransacBox.setDistanceThreshold (.01);
    ransacBox.computeModel();
    ransacBox.getInliers(inliers); 
    int inCountZ = inliers.size();

    std::cout << "inCountY: " << inCountY << "\n";
    float boxCount = inCountY + inCountZ;
    float percBox = boxCount / pcCount;
    
    std::cout << "inCountBox: " << boxCount << " percBox: " << percBox << "\n";
    std::cout << "inCountSphere: " << inCountSphere << " percSphere: " << percSphere << "\n";
    std::cout << "inCountCylinder: " << inCountCylinder << " percCylinder: " << percCylinder <<"\n";
//    std::cout << "inCountCone: " << inCountCone << " percCone: " << percCone << "\n";

    if (percCylinder > percBox && percCylinder > percSphere && percCylinder > 0.70)
    {
        cylinder = true;
        std::cout << "Is a cylinder" << "\n" << "percCylinder:  " << percCylinder;
    }
    else if(percBox > percCylinder && percBox > percSphere && percBox > 0.70)
    {
        if(inCountY > 3 * inCountZ) cerealBox = true;
        else box = true;
        std::cout << "Is a Box" << "\n" << percBox << " vs " << percSphere;  
    }
    else if (percSphere > percBox && percSphere > percCylinder && percSphere > 0.70)
    {
        sphere = true;
        std::cout << "Is a sphere" << "\n" << percSphere << " vs " << percBox;
    }
/*    else if (percCone > percBox && percCone > percCylinder && percCone > percSphere && percCone > 0.70)
    {
        cone = true;
        std::cout << "Is a cone" << "\n" << percCone;
    } */
    else
    {
        none = true;
        std::cout << "No shape could be extracted";
    }

  }

 }
