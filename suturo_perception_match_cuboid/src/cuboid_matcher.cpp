#include <suturo_perception_match_cuboid/cuboid_matcher.h>

CuboidMatcher::CuboidMatcher()
{
    input_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    debug = true;
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

Eigen::Vector3f CuboidMatcher::reduceNormAngle(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
  float dotproduct = v1.dot(v2);
  if(acos(dotproduct)> M_PI/2)
  {
    std::cout << "NORM IS ABOVE 90 DEG! TURN IN THE OTHER DIRECTION" << std::endl;
    v2 = -v2;
  }
  return v2;
}

int CuboidMatcher::transformationCount()
{
  return transformations_.size();
}

void CuboidMatcher::segmentPlanes()
{
  // Running index for the plane vectors
  int planeIdx = 0;

  // For each Extraction
  for (int i = 0; i < 2; i++) // Hack, extract two planes
  {
    DetectedPlane dp;
    detected_planes_.push_back(dp); // TODO boost pointer

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE ); // TODO edit
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.005); // Tolerance is 0.5 cm

    seg.setInputCloud (input_cloud_);
    seg.segment (*detected_planes_.at(planeIdx).getInliers(),
        *detected_planes_.at(planeIdx).getCoefficients());

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Extract the inliers
    extract.setInputCloud (input_cloud_);
    extract.setIndices (detected_planes_.at(planeIdx).getInliers());
    extract.setNegative (false);
    extract.filter (*detected_planes_.at(planeIdx).getPoints());

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*input_cloud_);

    // Calculate the centroid of each object
    Eigen::Vector4f centroid;
    computeCentroid(detected_planes_.at(planeIdx).getPoints(), centroid);
    detected_planes_.at(planeIdx).setCentroid(centroid);

    planeIdx ++ ;
  }

  for (int i = 0; i < detected_planes_.size(); i++)
  {
    for (int j = 0; j < detected_planes_.size(); j++)
    {
      if(i==j) continue;

      if(debug)
        std::cout << "Angle between Normal " << i << " and Normal " << j;

      // TODO check angle between planes. The should be near 0°, 90°, 180° or 270°
      // for cuboids

      float angle = detected_planes_.at(i).angleBetween(
          detected_planes_.at(j).getCoefficientsAsVector3f());

      if(debug)
      {
        std::cout << ": " << angle << " RAD, " << ((angle * 180) / M_PI) << " DEG";
        std::cout << std::endl;
      }

    }
  }

}
void CuboidMatcher::computeCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, Eigen::Vector4f &centroid)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());

  pcl::ConvexHull<pcl::PointXYZRGB> hull;
  hull.setInputCloud(cloud_in);
  hull.reconstruct (*hull_points);

  // Centroid calulcation
  pcl::compute3DCentroid (*hull_points, centroid);  
}

// Rotate the Vector 'normal_to_rotate' into 'base_normal'
// Returns a rotation matrix which can be used for the transformation
// The matrix also includes an empty translation vector
// By passing store_transformation=true, the transformation
// will be saved in the class attribute 'transformations'
Eigen::Matrix< float, 4, 4 > CuboidMatcher::rotateAroundCrossProductOfNormals(
    Eigen::Vector3f base_normal,
    Eigen::Vector3f normal_to_rotate, bool store_transformation=true)
{
    // M
    // Eigen::Vector3f plane_normal(dest.x, dest.y, dest.z);

    // Compute the necessary rotation to align a face of the object with the camera's
    // imaginary image plane
    // N
    // Eigen::Vector3f camera_normal;
    // camera_normal(0)=0;
    // camera_normal(1)=0;
    // camera_normal(2)=1;
    // Eigen::Vector3f camera_normal_normalized = camera_normal.normalized();
    float costheta = normal_to_rotate.dot(base_normal) / (normal_to_rotate.norm() * base_normal.norm() );

    Eigen::Vector3f axis;
    Eigen::Vector3f firstAxis = normal_to_rotate.cross(base_normal);
    // axis = plane_normal.cross(camera_normal) / (plane_normal.cross(camera_normal)).normalize();
    firstAxis.normalize();
    axis=firstAxis;
    float c = costheta;
    std::cout << "rotate COSTHETA: " << acos(c) << " RAD, " << ((acos(c) * 180) / M_PI) << " DEG" << std::endl;
    float s = sqrt(1-c*c);
    float CO = 1-c;

    float x = axis(0);
    float y = axis(1);
    float z = axis(2);
    
    Eigen::Matrix< float, 4, 4 > rotationBox;
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

    transformations_.push_back(rotationBox);

    return rotationBox;
}

bool CuboidMatcher::execute(Cuboid &c)
{
  segmentPlanes();
  // rotate until two axis are aligned
  // calculate bb
  // transform bb back and calculate dimensions, center, orientation, etc.
}
