// detectShape(perceivedObject.get_pointCloud());
#include "suturo_perception_3d_capabilities/cuboid_matcher_annotator.h"


using namespace suturo_perception_3d_capabilities;
using namespace suturo_perception_lib;

CuboidMatcherAnnotator::CuboidMatcherAnnotator(PerceivedObject &obj) : Capability(obj), perceived_object_(obj)
{
  table_mode_ = false;
}
CuboidMatcherAnnotator::CuboidMatcherAnnotator(suturo_perception_lib::PerceivedObject &obj, pcl::ModelCoefficients::Ptr table_coefficients) : Capability(obj), perceived_object_(obj)
{
  table_mode_ = true;
  table_coefficients_ = table_coefficients;
}
void CuboidMatcherAnnotator::execute(bool debug)
{
    CuboidMatcher cm;
    cm.setInputCloud(perceived_object_.get_pointCloud() );
    if(table_mode_)
    {
      cm.setTableCoefficients(table_coefficients_);
      cm.setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
    }
    else
    {
      cm.setMode(CUBOID_MATCHER_MODE_WITHOUT_COEFFICIENTS);
    }
    cm.setDebug(debug);
    // cm.setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
    // cm.setSaveIntermediateResults(true);
    Cuboid cuboid;
    // Set a value pattern to see if the cuboid value has been changed
    // in the CuboidMatcher call. These values WILL be overwritten, if a 
    // cuboid has been estimated successfully
    cuboid.length1 = 76;
    cuboid.length2 = 79;
    cuboid.length3 = 76;

    cm.execute(cuboid);

    // Set cuboid, if the estimation was successful
    if(cm.estimationSuccessful())
      perceived_object_.set_c_cuboid(cuboid);

}
void CuboidMatcherAnnotator::execute()
{
  execute(false);
}
