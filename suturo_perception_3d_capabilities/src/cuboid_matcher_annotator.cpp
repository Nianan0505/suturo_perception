// detectShape(perceivedObject.get_pointCloud());
#include "suturo_perception_3d_capabilities/cuboid_matcher_annotator.h"


using namespace suturo_perception_3d_capabilities;
using namespace suturo_perception_lib;

CuboidMatcherAnnotator::CuboidMatcherAnnotator(PerceivedObject &obj) : Capability(obj), perceived_object_(obj)
{
}
void CuboidMatcherAnnotator::execute()
{
    CuboidMatcher cm;
    cm.setInputCloud(perceived_object_.get_pointCloud() );
    // TODO PASS TABLE COEFFICIENTS
    // cm.setDebug(true);
    // cm.setTableCoefficients(table_coefficients);
    // cm.setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
    cm.setMode(CUBOID_MATCHER_MODE_WITHOUT_COEFFICIENTS);
    // cm.setSaveIntermediateResults(true);
    Cuboid cuboid;
    cm.execute(cuboid);

    // TODO error handling after execute

    perceived_object_.set_c_cuboid(cuboid);
}
