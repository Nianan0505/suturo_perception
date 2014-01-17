#include <suturo_perception_match_cuboid/detected_plane.h>

float DetectedPlane::angleBetween(Eigen::Vector3f v)
{
  float dotproduct = getCoefficientsAsVector3f().dot(v);
  return  acos(dotproduct);
}

Eigen::Vector3f DetectedPlane::getCoefficientsAsVector3f()
{
  Eigen::Vector3f v(
      coefficients_ ->values.at(0),
      coefficients_ ->values.at(1),
      coefficients_ ->values.at(2)
      );
  return v;
}

DetectedPlane::DetectedPlane()
{
  // Allocate memory for the class attributes
  coefficients_ = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
  points_= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  inliers_ = pcl::PointIndices::Ptr(new pcl::PointIndices);

}

