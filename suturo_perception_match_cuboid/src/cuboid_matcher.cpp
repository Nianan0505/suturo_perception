#include <suturo_perception_match_cuboid/cuboid_matcher.h>

CuboidMatcher::CuboidMatcher()
{
    input_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}
std::vector<DetectedPlane> *CuboidMatcher::getDetectedPlanes()
{
  return &detected_planes_;
}

void CuboidMatcher::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*cloud, *input_cloud_);
}

void CuboidMatcher::setSaveIntermediateResults(bool b)
{
  save_intermediate_results_ = b;
}

Eigen::Vector3f reduceNormAngle(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
  float dotproduct = v1.dot(v2);
  if(acos(dotproduct)> M_PI/2)
  {
    std::cout << "NORM IS ABOVE 90 DEG! TURN IN THE OTHER DIRECTION" << std::endl;
    v2 = -v2;
  }
  return v2;
}
