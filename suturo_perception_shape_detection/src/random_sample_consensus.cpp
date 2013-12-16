#include "random_sample_consensus.h"
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

using namespace suturo_perception_shape_detection;

RandomSampleConsensus::RandomSampleConsensus(suturo_perception_lib::PerceivedObject &obj) : suturo_perception_lib::Capability(obj)
{
 shape = None;
 logger = suturo_perception_utils::Logger("shape_detection");
}    

Shape
RandomSampleConsensus::getShape()
{
    return shape;
}

void
RandomSampleConsensus::detectShape(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*cloudIn));
    
  std::vector<int> inliers;

  // ONLY NEEDED FOR SampleConsensusModelCylinder
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> neCyl;
  // The output data
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
  // Empty kdtree used for normal estimation
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
 
  // Set the input cloud for which the normals are to be calculated
  neCyl.setInputCloud (cloud);

  // Pass the kdtree to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given)
  neCyl.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  neCyl.setRadiusSearch (0.03);

  // Compute the features
  neCyl.compute (*cloudNormals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*


/**
 *  Initialize the three RANSAC-Models that we try to fit onto the perceived
 *  object's point cloud.
 *  The three different shapes that can be recognized are spheres, cylinders,
 *  and boxes (represented through perpendicular planes).
 */
  // created RandomSampleConsensus object and compute the appropriat model
  pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>::Ptr
    modelSphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> (cloud));
  pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>::Ptr 
      p_modelcustom (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB> (cloud));
  pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB, pcl::Normal>::Ptr 
      modelCylinder (new pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB, pcl::Normal> (cloud));
  modelCylinder->setInputNormals(cloudNormals);
    
    //get the number of points in the input cloud
    int pcCount = cloud->points.size();
   
    // try to ransac-fit a cylinder model onto the point clou, PointNTd
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransacCylinder(modelCylinder); 
    ransacCylinder.setDistanceThreshold (.01);
    ransacCylinder.computeModel();
    ransacCylinder.getInliers(inliers);
    float inCountCylinder = inliers.size();
    float percCylinder = inCountCylinder / pcCount;

    // try to ransac-fit a sphere model onto the point cloud
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransacSphere (modelSphere);
    ransacSphere.setDistanceThreshold (.01);
    ransacSphere.computeModel();
    ransacSphere.getInliers(inliers);
    float inCountSphere = inliers.size();
    float percSphere = inCountSphere / pcCount;

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
    p_modelcustom->setEpsAngle (0.564); 
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransacBox (p_modelcustom);
    ransacBox.setDistanceThreshold (.01);
    ransacBox.computeModel();
    ransacBox.getInliers(inliers);
    int inCountY = inliers.size();
    std::cout << pcCount << "\n" << inCountY << "\n";
    
    // find a plane perpendicular to the x axis, accepting points up to 30°
    p_modelcustom->setAxis (Eigen::Vector3f (0.0, 1.0, 0.0));
    p_modelcustom->setEpsAngle (0.564);
    ransacBox.setDistanceThreshold (.01);
    ransacBox.computeModel();
    ransacBox.getInliers(inliers); 
    int inCountZ = inliers.size();

    logger.logInfo((boost::format("inCountY: %s") % inCountY).str());
    float boxCount = inCountY + inCountZ;
    float percBox = boxCount / pcCount;
    
    logger.logInfo((boost::format("inCountBox: %s  percBox: %s") % boxCount % percBox).str());
    logger.logInfo((boost::format("inCountSphere: %s  percSphere: %s") % inCountSphere % percSphere).str());
    logger.logInfo((boost::format("inCountCylinder: %s  percCylinder: %s") % inCountCylinder % percCylinder).str());

    if (percCylinder > percBox && percCylinder > percSphere && percCylinder > 0.70)
    {
        shape = Cylinder;
        logger.logInfo((boost::format("Object is a CYLINDER. percCylinder: %s") % percCylinder).str());
    }
    else if(percBox > percCylinder && percBox > percSphere && percBox > 0.70)
    {
        shape = Box;
        logger.logInfo((boost::format("Object is a BOX. percBox: %s") % percBox).str());
    }
    else if (percSphere > percBox && percSphere > percCylinder && percSphere > 0.70)
    {
        shape = Sphere;
        logger.logInfo((boost::format("Object is a SPHERE. percSphere: %s") % percSphere).str());
    }
    else
    {
        shape = None;
        logger.logInfo("No shape could be extracted");
    }

 }

void RandomSampleConsensus::execute()
{
  detectShape(perceivedObject.get_pointCloud());
  perceivedObject.set_c_shape(getShape());
}
// execute
 // Detect the shape of the object
    //rsc.detectShape(*it);
    //ptShape = rsc.getShape();
 //   percObj.set_c_shape(ptShape);

// void 
/*SuturoPerception::detectShape(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn)
{   
    suturo_perception_shape_detection::RandomSampleConsensus rsc;
    rsc.detectShape(cloudIn);
}*/

    /*
    // shape detector to detect shape
  // int to represent the shape
  suturo_perception_shape_detection::RandomSampleConsensus rsc;
  int ptShape;*/