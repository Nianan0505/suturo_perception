// TODO: Load 2d objrec Database file via parameter
//			 Publish the cluster image somewhere .. (we are ROS independent ... :x )	

#include "suturo_perception.h"
#include "perceived_object.h"
#include "point.h"
#include "random_sample_consensus.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <boost/thread/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace suturo_perception_lib;

// Comparator function for PerceivedObject's. PerceivedObjects will be compared by their volume
bool ReceivedObjectGreaterThan(const PerceivedObject& p1, 
    const PerceivedObject& p2)
{
  return p1.c_volume > p2.c_volume;
}

/*
 * Constructor
 */
SuturoPerception::SuturoPerception()
{
  // Set default parameters
  zAxisFilterMin = 0.0;
  zAxisFilterMax = 1.5;
  downsampleLeafSize = 0.01;
  planeMaxIterations = 1000;
  planeDistanceThreshold = 0.01;
  ecClusterTolerance = 0.02; // 2cm
  ecMinClusterSize = 8000;
  ecMaxClusterSize = 200000;  
  prismZMin = 0.02;
  prismZMax = 0.50; // cutoff 50 cm above plane
  ecObjClusterTolerance = 0.03; // 3cm
  ecObjMinClusterSize = 100;
  ecObjMaxClusterSize = 25000;
  debug = true;
  writer_pcd = false;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr SuturoPerception::getObjectsOnPlaneCloud()
{
  return objects_on_plane_cloud_;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SuturoPerception::getPlaneCloud()
{
  return plane_cloud_;
}
/*
 * Remove NaNs from given pointcloud. 
 * Return the nanles cloud.
 */
void
SuturoPerception::removeNans(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles (new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector<int> nans;
  pcl::removeNaNFromPointCloud(*cloud_in,*cloud_nanles,nans);

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "removeNans()");
}

/*
 * Filter cloud on z-axis. aka cut off cloud at given distance.
 * This method will use setKeepOrganized on the given PassThrough Filter.
 *
 * Return the filtered cloud.
 */
void 
SuturoPerception::filterZAxis(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::PassThrough<pcl::PointXYZRGB> &pass)
{

  if(cloud_in->points.size() == 0)
  {
    std::cerr << "Could not filter on Z Axis. input cloud empty" << std::endl;
    return;
  }

  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(zAxisFilterMin, zAxisFilterMax);
  pass.setKeepOrganized(true);
  pass.filter(*cloud_out);

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "filterZAxis()");
}

/*
 * Downsample the input cloud with a pcl::VoxelGrid
 * Return the filtered cloud.
 */
void 
SuturoPerception::downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  pcl::VoxelGrid <pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(downsampleLeafSize,downsampleLeafSize,downsampleLeafSize);
  vg.filter(*cloud_out);

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "downsample()");
}

/*
 * Fit plane to the input cloud
 * Return the inliers.
 */
void 
SuturoPerception::fitPlanarModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
    pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  if(cloud_in->points.size() == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset. input cloud empty" << std::endl;
    return;
  }

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setModelType(pcl::SACMODEL_PLANE); // TODO: parameterize
  seg.setMethodType(pcl::SAC_RANSAC);    // TODO: parameterize
  seg.setMaxIterations(planeMaxIterations);
  seg.setDistanceThreshold(planeDistanceThreshold);
  seg.setInputCloud(cloud_in);
  seg.segment(*inliers,*coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset. The inlier size is 0" << std::endl;
  }

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "fitPlanarModel()");
}

// Extract the given inliers from cloud_in as a PointCloud
// The method does nothing, if the set of inliers is empty.
// The parameter setNegative will be used for ExtractIndices::setNegative
void SuturoPerception::extractInliersFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, bool setNegative=false)
{
  // Input cloud can't be null
  if(inliers->indices.size () == 0)
  {
    std::cerr << "extractInliersFromPointCloud can't work with an empty set of indices. Exiting...." << std::endl;
    return;
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_p;
  extract_p.setInputCloud(cloud_in);
  extract_p.setIndices(inliers);
  extract_p.filter(*cloud_out);
  extract_p.setNegative(setNegative);
  //
  // if(writer_pcd) writer.write ("cloud_plane.pcd", *cloud_plane, false);
}


/* Extract the biggest cluster in PointCloud cloud_in
* The method returns true, if a cluster has been found.
* If no Cluster could be extracted, or an error occured, the method returns false.
* If you want to map the original inliers (which correspond to the input cloud_in), you can pass in a Pointer to the old inliers and receive the new inliers after the clustering in new_inliers.
* When both inlier pointers are NOT NULL, the mapping will be calculated and put into new_inliers.
*/
bool SuturoPerception::extractBiggestCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, const pcl::PointIndices::Ptr old_inliers, pcl::PointIndices::Ptr new_inliers)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  // Should we map the extracted points to the inliers in the input cloud?
  bool map_indices=false;
  if(old_inliers != NULL && new_inliers != NULL)
    map_indices = true;

  if(cloud_in->points.size() == 0)
  {
    std::cerr << "Could not extract biggest cluster. input cloud empty" << std::endl;
    return false;
  }

  // Use cluster extraction to get rid of the outliers of the segmented table
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeTable (new pcl::search::KdTree<pcl::PointXYZRGB>);
  treeTable->setInputCloud (cloud_in);  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecTable;
  ecTable.setClusterTolerance (ecClusterTolerance); // 2cm
  ecTable.setMinClusterSize (ecMinClusterSize);
  ecTable.setMaxClusterSize (ecMaxClusterSize);
  ecTable.setSearchMethod (treeTable);
  ecTable.setInputCloud (cloud_in);
  ecTable.extract (cluster_indices);

  std::vector<int> cluster_get_indices;
  cluster_get_indices = *ecTable.getIndices();

  std::cerr << "cluster_indices vector size: " << cluster_indices.size() << std::endl;

  if(cluster_indices.size() == 0)
  {
    std::cout << "No suitable cluster found for extraction. Skip ...";
    return false;
  }


  // Extract the biggest cluster (e.g. the table) in the plane cloud
  std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
    cloud_out->points.push_back (cloud_in->points[*pit]); 

    if(map_indices)
      new_inliers->indices.push_back(cluster_get_indices.at(*pit)); // Map the indices
                                                                  // from the cluster extraction
                                                                  // to a proper PointIndicies
                                                                  // instance relative to our
                                                                  // original plane
  }
  cloud_out->width = cloud_out->points.size ();
  cloud_out->height = 1;
  cloud_out->is_dense = true;

  // std::cerr << "New Inliers calculated: " << new_inliers->indices.size() << std::endl;

  // if(writer_pcd) writer.write ("cloud_out.pcd", *cloud_out, false);

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "extractBiggestCluster()");
  return true;
}

/**
 * Use EuclideanClusterExtraction on object_clusters to identify seperate objects in the given pointcloud.
 * Create a ConvexHull for every object_cluster and extract everything that's above it (in a given range,
 * see SuturoPerception::prismZMax and SuturoPerception::prismZMin.
 * In the future, this method will also extract 2d images from every object cluster.
 */
void SuturoPerception::clusterFromProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<cv::Mat> &extracted_images)
{

  if(object_clusters->points.size() == 0)
  {
    std::cerr << "clusterFromProjection: object_clusters is empty. Skipping ...." << std::endl;
    return;
  }

  if(original_cloud->points.size() == 0)
  {
    std::cerr << "clusterFromProjection: original_cloud is empty. Skipping ...." << std::endl;
    return;
  }

  mutex.lock();
  // extracted_images = tmpPerceivedObjects; // TODO use temporary list for images
  extracted_images.clear();
  mutex.unlock();

  pcl::PCDWriter writer;
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  // Identify clusters in the input cloud
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (object_clusters);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (ecObjClusterTolerance);
  ec.setMinClusterSize (ecObjMinClusterSize);
  ec.setMaxClusterSize (ecObjMaxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (object_clusters);
  ec.extract(cluster_indices);
  cout << "Got " << cluster_indices.size() << "clusters";

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "filter the objects above the plane");

  int i=0;
  // Iterate over the found clusters and extract single pointclouds
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    // Gather all points for a cluster into a single pointcloud
    boost::posix_time::ptime s1 = boost::posix_time::microsec_clock::local_time();
    cout << "Read object cloud" << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (object_clusters->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cerr << "Cloud Cluster Size is " << cloud_cluster->points.size () << std::endl;
    std::ostringstream fn;
    fn << "2dcluster_" << i << ".pcd";
    if(writer_pcd) writer.write(fn.str(), *cloud_cluster, false);

  
    // Extract every point above the 2d cluster.
    // These points will belong to a single object on the table
    pcl::PointIndices::Ptr object_indices (new pcl::PointIndices); // The extracted indices of a single object above the plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points (new pcl::PointCloud<pcl::PointXYZRGB>());
    extractAllPointsAbovePointCloud(original_cloud, cloud_cluster, object_points, object_indices, 2);
    extracted_objects.push_back(object_points);

    // std::cerr << "After extract" << std::endl;

    boost::posix_time::ptime e1 = boost::posix_time::microsec_clock::local_time();
    logTime(s1, e1, "Extracted Object Points");

    std::ostringstream cl_file;
    cl_file << "2d_Z_cluster_" << i << ".pcd";
    if(writer_pcd) writer.write(cl_file.str(), *object_points, false);
    // cout << "2d_Z_cluster_" << i << " has " << object_points->size() << " points" << endl;

    boost::posix_time::ptime s2 = boost::posix_time::microsec_clock::local_time();


    // TESTING, NO OPENCV
    // TODO Put it back in and debug it's failures
    /*
    // RGB Values for points
    int r,g,b;
    cv::Mat img(cv::Size(original_cloud->width,original_cloud->height),CV_8UC3, cv::Scalar(0,0,0)); // Create a cloud with the size of the original cloud
    // (for now)
    // fill in color of extracted object points
    for (std::vector<int>::const_iterator pit = object_indices->indices.begin(); pit != object_indices->indices.end(); pit++)
    {
      int index = removed_indices_filtered->at(*pit);
      img.at<cv::Vec3b>( index / 640, index % 640)[0] = original_cloud->points[index].b;
      img.at<cv::Vec3b>( index / 640, index % 640)[1] = original_cloud->points[index].g;
      img.at<cv::Vec3b>( index / 640, index % 640)[2] = original_cloud->points[index].r;
    }
    extracted_images.push_back(img);
    */
    i++;
  }

  if(writer_pcd) writer.write ("cluster_from_projection_clusters.pcd", *object_clusters, false);

}
/*
 * Extract all Points above a given pointcloud (hull_cloud)
 * A Convex Hull will be calculated around this point cloud.
 * After that, this method will use ExtractPolygonalPrismData to extract everything above the
 * PointCloud / ConvexHull within cloud_in. The indices will be put into object_indices.
 */
void SuturoPerception::extractAllPointsAbovePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
    pcl::PointIndices::Ptr object_indices, 
    int convex_hull_dimension=2)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::ConvexHull<pcl::PointXYZRGB> hull;

  hull.setDimension (convex_hull_dimension); 
  hull.setInputCloud (hull_cloud);
  hull.reconstruct (*hull_points);

  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
  prism.setInputCloud (cloud_in);
  prism.setInputPlanarHull (hull_points);
  prism.setHeightLimits (prismZMin, prismZMax);
  prism.segment (*object_indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers of the prism
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
  extract.setInputCloud (cloud_in);
  extract.setIndices (object_indices);
  extract.setNegative (false);
  extract.filter (*cloud_out);
}

/**
 * Project all points referenced by object_indices in cloud_in to a 2dimensional plane defined by coefficients.
 * The projected cloud will be available in cloud_out after the function call.
 *
 * If the object_indices are empty, the method will do nothing.
 */
void SuturoPerception::projectToPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointIndices::Ptr object_indices, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
  if(object_indices->indices.size() == 0)
  {
    std::cout << "No object indices in projectToPlaneCoefficients. Skip ...";
    return;
  }

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZRGB> proj_objs;
  proj_objs.setModelType (pcl::SACMODEL_PLANE);
  proj_objs.setIndices (object_indices); // project the whole object cloud to the plane
  proj_objs.setInputCloud (cloud_in);
  proj_objs.setModelCoefficients (coefficients); // project to the plane model
  proj_objs.filter (*cloud_out);

}

/*
 * Process a single point cloud.
 * This will include
 *   - Centroid calculation
 *   - Volume calculation
 *
 * The result is a list of PerceivedObject's, which will be put into the buffer perceivedObjects.
 */
void SuturoPerception::processCloudWithProjections(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{

	boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      objects_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  pcl::PCDWriter writer;


  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  // TODO Remove NaNs

  // Build a filter to filter on the Z Axis
  pcl::PassThrough<pcl::PointXYZRGB> pass(true);
  filterZAxis(cloud_in, cloud_filtered, pass);
  std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;

  std::vector<int> removed_indices_filtered;
  removed_indices_filtered = *pass.getIndices();
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "z-filter");


  //voxelizing cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
  downsample(cloud_filtered, cloud_downsampled);
  cloud_filtered = cloud_downsampled; // Use the downsampled cloud now

  // Find the biggest table plane in the scene
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  fitPlanarModel(cloud_filtered, inliers, coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;
  // Table segmentation done
  
  // Extract the plane as a PointCloud from the calculated inliers
  std::cerr << "Table inlier count" << inliers->indices.size();
  extractInliersFromPointCloud(cloud_filtered, inliers, cloud_plane);

  // Take the biggest cluster in the extracted plane. This will be
  // most likely our desired table pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices);
  extractBiggestCluster(cloud_plane, plane_cluster, inliers, new_inliers);

  // NOTE: We need to transform the inliers from table_cluster_indices to inliers
  inliers = new_inliers;
  
  if(inliers->indices.size () == 0)
  {
    std::cerr << "Second Table Inlier Set is empty. Exiting...." << std::endl;
    return;
  }
  plane_cloud_ = plane_cluster; // save the reference to the segmented and clustered table plane

  // Extract all objects above
  // the table plane
  pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
  extractAllPointsAbovePointCloud(cloud_filtered, plane_cluster, object_clusters, object_indices);
  objects_on_plane_cloud_ = object_clusters;

  // Project the pointcloud above the table onto the table to get a 2d representation of the objects
  // This will cause every point of an object to be at the base of the object
  projectToPlaneCoefficients(cloud_filtered, object_indices, coefficients, objects_cloud_projected);
  if(writer_pcd) writer.write ("objects_cloud_projected.pcd", *objects_cloud_projected, false);

  // Take the projected points, cluster them and extract everything that's above it
  // By doing this, we should get every object on the table and a 2d image of it.
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects;
  clusterFromProjection(objects_cloud_projected, cloud_in, &removed_indices_filtered, extractedObjects, perceived_cluster_images_);
  std::cerr << "extractedObjects Vector size" << extractedObjects.size() << std::endl;
  std::cerr << "extractedImages Vector size" << perceived_cluster_images_.size() << std::endl;
    
  // temporary list of perceived objects
  std::vector<PerceivedObject> tmpPerceivedObjects;

  // shape detector to detect shape
  // int to represent the shape
  suturo_perception_shape_detection::RandomSampleConsensus rsc;
  int ptShape;


  // hack for collision_objects
  collision_objects = extractedObjects;
  int i=0;
  // Iterate over the extracted clusters and write them as a PerceivedObjects to the result list
  for (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = extractedObjects.begin(); 
      it != extractedObjects.end(); ++it)
  {  
    std::cout << "[suturo_perception_lib] Transform cluster " << i << " into a message. Cluster has "
      << (*it)->points.size() << " points" << std::endl;

    if((*it)->points.size()==0)
    {
      std::cerr << "[suturo_perception_lib] Cluster cloud is empty. Skipping ...." << std::endl;
      i++;
      continue;
    }

    // Calculate the volume of each cluster
    // Create a convex hull around the cluster and calculate the total volume
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_points_from_hull (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(*it);
    hull.setDimension(3);
    hull.setComputeAreaVolume(true); // This creates alot of output, but it's necessary for getTotalVolume() ....
    hull.reconstruct (*hull_points);

    // Get average color of the object
    uint32_t averageColor = getAverageColor(*it);

    // Detect the shape of the object
    rsc.detectShape(*it);
    ptShape = rsc.getShape();

    // Centroid calulcation
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*hull_points, centroid);  

    std::cout << "[suturo_perception_lib] Centroid: "
      << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ", " << std::endl;

    // Add the detected cluster to the list of perceived objects
    PerceivedObject percObj;
    percObj.c_id= objectID;
    objectID++;
    Point ptCentroid;
    ptCentroid.x=centroid[0];
    ptCentroid.y=centroid[1];
    ptCentroid.z=centroid[2];
    percObj.c_centroid = ptCentroid;
    percObj.c_shape = ptShape;
    percObj.c_volume = hull.getTotalVolume();
    percObj.c_color_average_r = (averageColor >> 16) & 0x0000ff;
    percObj.c_color_average_g = (averageColor >> 8)  & 0x0000ff;
    percObj.c_color_average_b = (averageColor)       & 0x0000ff;

    tmpPerceivedObjects.push_back(percObj);
    i++;
  }

  // Lock the buffer access to assign the recently perceived objects
  mutex.lock();
  perceivedObjects = tmpPerceivedObjects;
  mutex.unlock();

  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  logTime(start, end, "TOTAL");
}


std::vector<PerceivedObject> SuturoPerception::getPerceivedObjects()
{
  return perceivedObjects;
}

std::vector<cv::Mat> SuturoPerception::getPerceivedClusterImages()
{
  return perceived_cluster_images_;
}

// debug timelog for profiling
void SuturoPerception::logTime(boost::posix_time::ptime s, boost::posix_time::ptime e, std::string text)
{
  if(debug)
  {
    boost::posix_time::time_duration d = e - s;
    float diff = (float)d.total_microseconds() / (float)1000;
    std::cout << "[perception_lib] Time for " << text << ": " << diff << " ms" << std::endl;
  }
}

void SuturoPerception::writeCloudToDisk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, std::string filename="")
{
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ex_obj;
  ex_obj.push_back(point_cloud);
  writeCloudToDisk(ex_obj, filename);
}

 
// debug method
void SuturoPerception::writeCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects)
{
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = extractedObjects.begin();
  pcl::PCDWriter writer;
  int cnt = 0;
  for(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = extractedObjects.begin(); 
      it != extractedObjects.end(); ++it)
  {
    std::stringstream ss;
    ss << "debug_pcd_" << cnt << ".pcd" << std::endl;
    writer.write(ss.str(), **it);
    ++cnt;
  }
}

void SuturoPerception::writeCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects, std::string filename)
{
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = extractedObjects.begin();
  pcl::PCDWriter writer;
  writer.write(filename, **it);
}

void 
SuturoPerception::detectShape(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn)
{   
    suturo_perception_shape_detection::RandomSampleConsensus rsc;
    rsc.detectShape(cloudIn);
}

/*
 * Get average color for the points in the given point cloud
 */
  uint32_t
SuturoPerception::getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  if(cloud_in->points.size() == 0) return 0;

  double average_r = 0;
  double average_g = 0;
  double average_b = 0;
  for(int i = 0; i < cloud_in->points.size(); ++i)
  {
    uint32_t rgb = *reinterpret_cast<int*>(&cloud_in->points[i].rgb);
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8) & 0x0000ff;
    uint8_t b = (rgb) & 0x0000ff;
    average_r += (double)r / (double)cloud_in->points.size();
    average_g += (double)g / (double)cloud_in->points.size();
    average_b += (double)b / (double)cloud_in->points.size();
  }

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "getAverageColor()");

  return ((uint32_t)average_r << 16 | (uint32_t)average_g << 8 | (uint32_t)average_b);
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
