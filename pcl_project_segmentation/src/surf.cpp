#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/ModelCoefficients.h>
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

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

bool writer_pcd = false;

using namespace std;
// debug timelog for profiling
//
void logTime(boost::posix_time::ptime s, boost::posix_time::ptime e, std::string text)
{
                boost::posix_time::time_duration d = e - s;
                float diff = (float)d.total_microseconds() / (float)1000;
                std::cout << "[perception_lib] Time for " << text << ": " << diff << " ms" << std::endl;
}
// void 
// extractImages(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
//                                std::vector<pcl::PointIndices> *extracted_indices,
//                                std::vector<int> *nan_indices, int file_index = 0)

void clusterTwoDee(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered)
{
  cout << "Cluster 2d started";
  pcl::PCDWriter writer;
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (object_clusters);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.03); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
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
    cout << "Read first cloud";
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
    // chull.setAlpha (0.1); // Only in Concave Hulls
    // Note: Concave Hulls are much more detailed
    chull.reconstruct (*cloud_hull);

    // std::cerr << "Convex hull has: " << cloud_hull->points.size ()
    //           << " data points." << std::endl;

    // writer.write ("cloud_hull.pcd", *cloud_hull, false);
    // std::ostringstream fn;
    // fn << "hull_" << i << ".pcd";

    // writer.write(fn.str(), *cloud_cluster, false);

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

    boost::posix_time::ptime e1 = boost::posix_time::microsec_clock::local_time();
    logTime(s1, e1, "Extracted Object Points");

    std::ostringstream cl_file;
    cl_file << "2d_Z_cluster_" << i << ".pcd";
    if(writer_pcd) writer.write(cl_file.str(), *object_points, false);
    cout << "2d_Z_cluster_" << i << " has " << object_points->size() << " points" << endl;

    boost::posix_time::ptime s2 = boost::posix_time::microsec_clock::local_time();
    // create black cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    image_cloud->resize(640*480);
    image_cloud->width = 640;
    image_cloud->height = 480;
    image_cloud->is_dense = false;
    for (int x = 0; x < 640; x++) {
      for (int y = 0; y < 480; y++) {
        image_cloud->at(x,y).rgb = 0;
      }
    }
    if(removed_indices_filtered == NULL)
    {
      cout << "indice mappings are NULL. Can't calculate image" << endl;
    }

    int ex_count = 0;
    // fill in color of extracted object points
    for (std::vector<int>::const_iterator pit = object_indices->indices.begin(); pit != object_indices->indices.end(); pit++)
    {
      int index = removed_indices_filtered->at(*pit);
      // int index = *pit;
      // std::cout << "index = " << index << " ";
      image_cloud->at(index % 640, index / 640).x = original_cloud->points[index].x;
      image_cloud->at(index % 640, index / 640).y = original_cloud->points[index].y;
      image_cloud->at(index % 640, index / 640).z = original_cloud->points[index].z;
      image_cloud->at(index % 640, index / 640).rgb = original_cloud->points[index].rgb;
      ex_count++;
    }
    boost::posix_time::ptime e2 = boost::posix_time::microsec_clock::local_time();
    logTime(s2, e2, "Extracted Object Points");

    cout << "Tried to map " << ex_count << " points to the image" << endl;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    // {
    //   int index = nan_indices->at(*pit);
    //   object_cloud->points.push_back (cloud_in->points[index]); //*
    // }
    cout << "Went through the for-loops" << endl;

    // object_cloud->width = object_cloud->points.size ();
    // object_cloud->height = 1;
    // object_cloud->is_dense = true;

    pcl::PCDWriter writer;
    std::stringstream extracted_path;
    extracted_path << "img_object_" << i << ".pcd";

    if(writer_pcd) writer.write(extracted_path.str(), *image_cloud, false); // eigentlich *

    // std::stringstream real_cloud_path;
    // real_cloud_path << "img_object" << i << "_real.pcd";
    // writer.write(real_cloud_path.str(), *object_cloud, false); // eigentlich *
    i++;
  }

  if(writer_pcd) writer.write ("TwoDee_object_clusters.pcd", *object_clusters, false);

}

// void clusterThreeDee(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters)
// {
//   pcl::PCDWriter writer;
//   // Creating the KdTree object for the search method of the extraction
//   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//   tree->setInputCloud (object_clusters);
// 
//   std::vector<pcl::PointIndices> cluster_indices;
//   pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
//   ec.setClusterTolerance (0.02); // 2cm
//   ec.setMinClusterSize (100);
//   ec.setMaxClusterSize (25000);
//   ec.setSearchMethod (tree);
//   ec.setInputCloud (object_clusters);
//   ec.extract(cluster_indices);
// 
//   int i=0;
//   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//   {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
//     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
//       cloud_cluster->points.push_back (object_clusters->points[*pit]); //*
//     cloud_cluster->width = cloud_cluster->points.size ();
//     cloud_cluster->height = 1;
//     cloud_cluster->is_dense = true;
//     std::ostringstream fn;
//     fn << "3dcluster_" << i << ".pcd";
//     writer.write(fn.str(), *cloud_cluster, false);
//     i++;
//   }
// 
//   writer.write ("ThreeDee_object_clusters.pcd", *object_clusters, false);
// }

int main( int argc, char** argv )
{
  boost::posix_time::ptime s0 = boost::posix_time::microsec_clock::local_time();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      objects_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  reader.read ("cup_tape_box.pcd", *cloud);
  boost::posix_time::ptime e0 = boost::posix_time::microsec_clock::local_time();
  logTime(s0, e0, "initial reading");

  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  // cloud_filtered = cloud;
  // Build a filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZRGB> pass(true);
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.4); // fine tuned! Warning // TODO Clustering for biggest plane
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
  // pcl::ExtractIndices<pcl::PointXYZRGB> extract_p;
  // extract_p.setInputCloud(cloud_filtered);
  // extract_p.setIndices(inliers);
  // extract_p.filter(*cloud_plane);
  // writer.write ("cloud_plane.pcd", *cloud_plane, false);

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
  clusterTwoDee(objects_cloud_projected, cloud, &removed_indices_filtered ); // TODO: Refactor to better interface. two dee and three dee clustering are very similar

  return (0);

}


// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
