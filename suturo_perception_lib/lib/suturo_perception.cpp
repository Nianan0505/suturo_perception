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
	ecMinClusterSize = 10000;
	ecMaxClusterSize = 200000;  
	prismZMin = 0.;
	prismZMax = 0.50; // cutoff 50 cm above plane
	ecObjClusterTolerance = 0.03; // 3cm
	ecObjMinClusterSize = 100;
	ecObjMaxClusterSize = 25000;
	debug = true;
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
	//return cloud_nanles;
}

/*
 * Filter cloud on z-axis. aka cut off cloud at given distance.
 * Return the filtered cloud.
 */
void 
SuturoPerception::filterZAxis(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
															pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
	boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(zAxisFilterMin, zAxisFilterMax);
	pass.filter(*cloud_out);

	boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
	logTime(s, e, "filterZAxis()");
}

/*
 * Downsample the input cloud
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
																 pcl::PointIndices::Ptr inliers)
{	
	boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

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
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
	logTime(s, e, "fitPlanarModel()");
}

/*
 * Filter out points above biggest plane
 * Return filtered cloud.
 */
void 
SuturoPerception::extractObjectCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
																			 const pcl::PointIndices::Ptr inliers,
																			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{	
	boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

	if(cloud_in->points.size() == 0 || inliers->indices.size() == 0)
	{
		std::cerr << "extractObjectCluster: cloud or inliers empty" << std::endl;
		return;
	}

	// splitting the cloud in two: plane + other
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud_in);
	extract.setIndices(inliers);
	extract.filter(*cloud_plane);

	// Use cluster extraction to get rid of the outliers of the segmented table
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeTable (new pcl::search::KdTree<pcl::PointXYZRGB>);
	treeTable->setInputCloud (cloud_plane);  
	std::vector<pcl::PointIndices> table_cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecTable;
	ecTable.setClusterTolerance (ecClusterTolerance); // 2cm
	ecTable.setMinClusterSize (ecMinClusterSize);
	ecTable.setMaxClusterSize (ecMaxClusterSize);
	ecTable.setSearchMethod (treeTable);
	ecTable.setInputCloud (cloud_plane);
	ecTable.extract (table_cluster_indices);

	// Extract the biggest cluster (e.g. the table) in the plane cloud
	std::vector<pcl::PointIndices>::const_iterator it = table_cluster_indices.begin ();
	if(table_cluster_indices.size() == 0)
	{
		std::cerr << "table indice size 0. skipping" << std::endl;
		return; // return nothing. plane not found
	}
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		plane_cluster->points.push_back (cloud_plane->points[*pit]);
	plane_cluster->width = plane_cluster->points.size ();
	plane_cluster->height = 1;
	plane_cluster->is_dense = true;

	//std::cerr << "Table point cloud " << plane_cluster->points.size () << " data points." << std::endl;
	
	// Remove the plane from the rest of the point cloud
	extract.setNegative(true);
	extract.filter(*cloud_clusters);

	// Use ExtractPolygonalPrism to get all the point clouds above the plane in a given range
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::ConvexHull<pcl::PointXYZRGB> hull;
	pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);

	hull.setDimension (2); 
	hull.setInputCloud (plane_cluster);
	hull.reconstruct (*hull_points);
	
	pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
	prism.setInputCloud (cloud_clusters);
	prism.setInputPlanarHull (hull_points);
	prism.setHeightLimits (prismZMin, prismZMax);
	prism.segment (*object_indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extractObjects;
	// Extract the inliers of the prism
	extract.setInputCloud (cloud_clusters);
	extract.setIndices (object_indices);
	extract.setNegative (false);
	extract.filter (*cloud_out);

	boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
	logTime(s, e, "extractObjectCluster()");
}


/*
 * Filter out points above biggest plane
 * Return filtered cloud.
 */
void 
SuturoPerception::extractObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
																 std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& extractedObjects)
{
	boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	if(cloud_in->size() == 0) {std::cerr<<"extractedObjects inputcloud empty" <<std::endl; return;} // cloud_in was empty

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud_in);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (ecObjClusterTolerance); // 2cm
	ec.setMinClusterSize (ecObjMinClusterSize);
	ec.setMaxClusterSize (ecObjMaxClusterSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_in);
	ec.extract (cluster_indices);

	// temporary list of perceived objects
	std::vector<PerceivedObject> tmpPerceivedObjects;
	// Iterate over the extracted clusters and write them as a PerceivedObjects to the result list
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		    cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		extractedObjects.push_back(cloud_cluster);
		// writeCloudToDisk(hull_points, "hull_points.pcd");
	}

	boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
	logTime(s, e, "extractObjects()");
}

/*
 * Process a single point cloud.
 * This will include
 *   - Centroid calculation
 *   - Volume calculation
 *
 * The result is a list of PerceivedObject's, which will be put into the buffer perceivedObjects.
 */
void SuturoPerception::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
	boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

	//point cloud objects
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects;

	//removing nans from point clouds
	removeNans(cloud_in, cloud_nanles);

	//filtering cloud on z axis
	filterZAxis(cloud_nanles, cloud_filtered);

	downsample(cloud_filtered, cloud_filtered_downsampled);

	//fitting a plane to the filtered cloud
	fitPlanarModel(cloud_filtered_downsampled, inliers);

	// filter out biggest surface and return cloud above it
	extractObjectCluster(cloud_filtered_downsampled, inliers, object_clusters);

	// cluster extraction
	// extract objects from downsampled object cloud
	extractObjects(object_clusters, extractedObjects);
	std::cerr << "extractVector size" << extractedObjects.size() << std::endl;

	// temporary list of perceived objects
	std::vector<PerceivedObject> tmpPerceivedObjects;

	// shape detector to detect shape
  // int to represent the shape
  suturo_perception_shape_detection::RandomSampleConsensus rsc;
  int ptShape;

  // Iterate over the extracted clusters and write them as a PerceivedObjects to the result list
	for (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = extractedObjects.begin(); 
		   it != extractedObjects.end(); ++it)
  {  
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

    // used in getObjects. Left here for reference. incredibly slow
		/*
		std::vector<pcl::Vertices> hull_polygons;
		hull.reconstruct (*hull_points, hull_polygons);

		pcl::CropHull<pcl::PointXYZRGB> crophull;
		crophull.setHullCloud(hull_points);
		crophull.setInputCloud(cloud_in);
		crophull.setHullIndices(hull_polygons);
		crophull.setDim(3);
		crophull.filter(*object);
		boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
		logTime(s, e, "crop hull");*/

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
	}

	// Sort by volume
	std::sort(tmpPerceivedObjects.begin(), tmpPerceivedObjects.end(), ReceivedObjectGreaterThan);

	// Lock the buffer access to assign the recently perceived objects
	mutex.lock();
	perceivedObjects = tmpPerceivedObjects;
	mutex.unlock();

	boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
	logTime(s, e, "TOTAL");
}

std::vector<PerceivedObject> SuturoPerception::getPerceivedObjects()
{
  return perceivedObjects;
}

/*
 * Convenience method to get separated object pointclouds.
 * Return a point cloud vector containing a cloud for each extracted object
 */
void SuturoPerception::getObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
																	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& extractedObjects)
{
	//point cloud objects
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	//removing nans from point clouds
	removeNans(cloud_in, cloud_nanles);

	//downsample(cloud_nanles, cloud_downsampled);
	//filtering cloud on z axis
	filterZAxis(cloud_nanles, cloud_filtered);

	//fitting a plane to the filtered cloud
	fitPlanarModel(cloud_filtered, inliers);

	// filter out biggest surface and return cloud above it
	extractObjectCluster(cloud_filtered, inliers, object_clusters);

	// cluster extraction
	// extract objects from downsampled object cloud
	extractObjects(object_clusters, extractedObjects);
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
