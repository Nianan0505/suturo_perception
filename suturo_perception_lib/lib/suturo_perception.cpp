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
	ecMinClusterSize = 10000;
	ecMaxClusterSize = 200000;  
	prismZMin = 0.;
	prismZMax = 0.50; // cutoff 50 cm above plane
	ecObjClusterTolerance = 0.03; // 3cm
	ecObjMinClusterSize = 100;
	ecObjMaxClusterSize = 25000;
	debug = true;
	writer_pcd = false;
	// // Init cv Window 
	// cv::namedWindow("foobar", CV_WINDOW_AUTOSIZE);
	// cv::destroyWindow("foobar");
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



void SuturoPerception::clusterFromProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<cv::Mat> &extracted_images)
{
  pcl::PCDWriter writer;
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (object_clusters);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.03); // 2cm
	ec.setMinClusterSize (ecObjMinClusterSize);
	ec.setMaxClusterSize (ecObjMaxClusterSize);
  // ec.setMinClusterSize (100);
  // ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (object_clusters);
  ec.extract(cluster_indices);
  cout << "Got " << cluster_indices.size() << "clusters";

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "filter the objects above the plane");

  int i=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    boost::posix_time::ptime s1 = boost::posix_time::microsec_clock::local_time();
    cout << "Read object cloud" << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (object_clusters->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::ostringstream fn;
    fn << "2dcluster_" << i << ".pcd";
    if(writer_pcd) writer.write(fn.str(), *cloud_cluster, false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (cloud_cluster); // The clusters in this method must be 2d
    chull.setDimension(2);
    chull.reconstruct (*cloud_hull);

    pcl::PointIndices::Ptr object_indices (new pcl::PointIndices); // The indices of the objects above the plane

    // Extract everything above the projection of the object
    pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
    prism.setInputCloud (original_cloud);
    prism.setInputPlanarHull (cloud_hull); 
    prism.setHeightLimits (0.01, 0.5);
    prism.segment (*object_indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Extract the inliers of the prism
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points (new pcl::PointCloud<pcl::PointXYZRGB>());
    extract.setInputCloud (original_cloud);
    extract.setIndices (object_indices);
    extract.setNegative (false);
    extract.filter (*object_points);
		extracted_objects.push_back(object_points);

    boost::posix_time::ptime e1 = boost::posix_time::microsec_clock::local_time();
    logTime(s1, e1, "Extracted Object Points");

    std::ostringstream cl_file;
    cl_file << "2d_Z_cluster_" << i << ".pcd";
    if(writer_pcd) writer.write(cl_file.str(), *object_points, false);
    // cout << "2d_Z_cluster_" << i << " has " << object_points->size() << " points" << endl;

    boost::posix_time::ptime s2 = boost::posix_time::microsec_clock::local_time();

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

		// CREATE THE IMAGE CLOUD - KEEP THAT FOR REFERENCE
    // create black cloud
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // image_cloud->resize(640*480);
    // image_cloud->width = 640;
    // image_cloud->height = 480;
    // image_cloud->is_dense = false;
    // for (int x = 0; x < 640; x++) {
    //   for (int y = 0; y < 480; y++) {
    //     image_cloud->at(x,y).rgb = 0;
    //   }
    // }
    // if(removed_indices_filtered == NULL)
    // {
    //   cout << "indice mappings are NULL. Can't calculate image" << endl;
    // }

    // int ex_count = 0;
    // // fill in color of extracted object points
    // for (std::vector<int>::const_iterator pit = object_indices->indices.begin(); pit != object_indices->indices.end(); pit++)
    // {
    //   int index = removed_indices_filtered->at(*pit);
    //   // int index = *pit;
    //   // std::cout << "index = " << index << " ";
    //   image_cloud->at(index % 640, index / 640).x = original_cloud->points[index].x;
    //   image_cloud->at(index % 640, index / 640).y = original_cloud->points[index].y;
    //   image_cloud->at(index % 640, index / 640).z = original_cloud->points[index].z;
    //   image_cloud->at(index % 640, index / 640).rgb = original_cloud->points[index].rgb;
    //   ex_count++;
    // }
    // boost::posix_time::ptime e2 = boost::posix_time::microsec_clock::local_time();
    // logTime(s2, e2, "Extracted Object Points");

    // cout << "Went through the for-loops" << endl;

    // pcl::PCDWriter writer;
    // std::stringstream extracted_path;
    // extracted_path << "img_object_" << i << ".pcd";

    // if(writer_pcd) writer.write(extracted_path.str(), *image_cloud, false); // eigentlich *

    i++;
  }

  if(writer_pcd) writer.write ("cluster_from_projection_clusters.pcd", *object_clusters, false);

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

  boost::posix_time::ptime s0 = boost::posix_time::microsec_clock::local_time();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      objects_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  boost::posix_time::ptime e0 = boost::posix_time::microsec_clock::local_time();
  logTime(s0, e0, "initial reading");

  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  // cloud_filtered = cloud;
  // Build a filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZRGB> pass(true);
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0, 1.4); // fine tuned! Warning // TODO Clustering for biggest plane
  pass.setFilterLimits (0, 1.8); // fine tuned! Warning // TODO Clustering for biggest plane
  pass.filter (*cloud_filtered);
  pass.setKeepOrganized(true);
  std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;

  std::vector<int> removed_indices_filtered;
  removed_indices_filtered = *pass.getIndices();
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logTime(s, e, "z-filter");


  //voxelizing cloud
  boost::posix_time::ptime s1 = boost::posix_time::microsec_clock::local_time();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::VoxelGrid <pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud_filtered);
  vg.setLeafSize(0.01f,0.01f,0.01f);
  vg.filter(*cloud_downsampled);
  cloud_filtered = cloud_downsampled;

  boost::posix_time::ptime e1 = boost::posix_time::microsec_clock::local_time();
  logTime(s1, e1, "voxeld()");
  // cloud_filtered = cloud_downsampled; // Use the downsampled cloud now


  boost::posix_time::ptime s2 = boost::posix_time::microsec_clock::local_time();
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  // seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  boost::posix_time::ptime e2 = boost::posix_time::microsec_clock::local_time();
  logTime(s2, e2, "ransac");


  boost::posix_time::ptime s3 = boost::posix_time::microsec_clock::local_time();
  //splitting the cloud in two: plane + other
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_p;
  extract_p.setInputCloud(cloud_filtered);
  extract_p.setIndices(inliers);
  extract_p.filter(*cloud_plane);
  if(writer_pcd) writer.write ("cloud_plane.pcd", *cloud_plane, false);

  boost::posix_time::ptime s23 = boost::posix_time::microsec_clock::local_time();
  // Use cluster extraction to get rid of the outliers of the segmented table
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeTable (new pcl::search::KdTree<pcl::PointXYZRGB>);
  treeTable->setInputCloud (cloud_plane);  
  std::vector<pcl::PointIndices> table_cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecTable;
  ecTable.setClusterTolerance (0.02); // 2cm
  ecTable.setMinClusterSize (10000); // TODO Parameterize
  ecTable.setMaxClusterSize (200000); // TODO Parameterize
  ecTable.setSearchMethod (treeTable);
  ecTable.setInputCloud (cloud_plane);
  ecTable.extract (table_cluster_indices);

  std::vector<int> cluster_get_indices;
  cluster_get_indices = *ecTable.getIndices();
  pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices);

  // Extract the biggest cluster (e.g. the table) in the plane cloud
  std::vector<pcl::PointIndices>::const_iterator it = table_cluster_indices.begin ();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
    plane_cluster->points.push_back (cloud_plane->points[*pit]); //*
    new_inliers->indices.push_back(cluster_get_indices.at(*pit)); // Map the indices
    // from the cluster extraction
    // to a proper PointIndicies
    // instance relative to our
    // original plane
  }
  plane_cluster->width = plane_cluster->points.size ();
  plane_cluster->height = 1;
  plane_cluster->is_dense = true;

  if(writer_pcd) writer.write ("plane_cluster.pcd", *plane_cluster, false);
  // std::cout << "Table point cloud " << plane_cluster->points.size () << " data points." << std::endl;

  boost::posix_time::ptime e23 = boost::posix_time::microsec_clock::local_time();
  logTime(s23, e23, "clustered table");
  // NOTE: We need to transform the inliers from table_cluster_indices to inliers
  inliers = new_inliers;
  
  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;
  if(writer_pcd) writer.write ("cloud_projected.pcd", *cloud_projected, false);

  // Create a convex Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud (cloud_projected);
  // chull.setAlpha (0.1); // Only in Concave Hulls
  // Note: Concave Hulls are much more detailed
  chull.reconstruct (*cloud_hull);

  boost::posix_time::ptime e3 = boost::posix_time::microsec_clock::local_time();
  logTime(s3, e3, "create hull from plane");

  std::cerr << "Convex hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  if(writer_pcd) writer.write ("cloud_hull.pcd", *cloud_hull, false);

  boost::posix_time::ptime s4 = boost::posix_time::microsec_clock::local_time();
  pcl::PointIndices::Ptr object_indices (new pcl::PointIndices); // The indices of the objects above the plane

  // Extract everything above the plane
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
  prism.setInputCloud (cloud_filtered);
  prism.setInputPlanarHull (cloud_hull); //??
  prism.setHeightLimits (0.01, 0.5);
  prism.segment (*object_indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers of the prism
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (object_indices);
  extract.setNegative (false);
  extract.filter (*object_clusters);
  if(writer_pcd) writer.write ("object_clusters.pcd", *object_clusters, false);

        // Project the model inliers
        pcl::ProjectInliers<pcl::PointXYZRGB> proj_objs;
        proj_objs.setModelType (pcl::SACMODEL_PLANE);
        proj_objs.setIndices (object_indices); // project the whole object cloud to the plane
        proj_objs.setInputCloud (cloud_filtered);
        proj_objs.setModelCoefficients (coefficients); // project to the plane model
        proj_objs.filter (*objects_cloud_projected);
        std::cerr << "Object PointCloud after projection has: "
                  << objects_cloud_projected->points.size () << " data points." << std::endl;
        if(writer_pcd) writer.write ("objects_cloud_projected.pcd", *objects_cloud_projected, false);

  boost::posix_time::ptime e4 = boost::posix_time::microsec_clock::local_time();
  logTime(s4, e4, "filter the objects above the plane");

  // clusterThreeDee(object_clusters);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects;
	std::vector<cv::Mat> extractedImages;
  clusterFromProjection(objects_cloud_projected, cloud_in, &removed_indices_filtered, extractedObjects, extractedImages);
	std::cerr << "extractedObjects Vector size" << extractedObjects.size() << std::endl;
	std::cerr << "extractedImages Vector size" << extractedImages.size() << std::endl;
	if(extractedImages.size() > 0){
		cv::imwrite("/tmp/first_cluster.jpg", extractedImages.at(0));
	}

    
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

	boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
	logTime(start, end, "TOTAL");
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
