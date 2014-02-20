#ifndef SUTURO_PERCEPTION_CUBOID_MATCHER_ANNOTATOR
#define SUTURO_PERCEPTION_CUBOID_MATCHER_ANNOTATOR

#include <boost/signals2/mutex.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "capability.h"
#include "perceived_object.h"
#include "suturo_perception_utils.h"
#include "suturo_perception_match_cuboid/cuboid_matcher.h"
#include <pcl/ModelCoefficients.h>


namespace suturo_perception_3d_capabilities
{
  /**
   * This class will estimate a Cuboid, that fit's a given pointcloud.
   * The core implementation is in the ROS package suturo_perception_match_cuboid.
   */
  class CuboidMatcherAnnotator : public suturo_perception_lib::Capability
  {
    public:
      // capability method
      CuboidMatcherAnnotator(suturo_perception_lib::PerceivedObject &obj);
      // Start the CuboidMatcher in table coefficient mode
      CuboidMatcherAnnotator(suturo_perception_lib::PerceivedObject &obj, pcl::ModelCoefficients::Ptr table_coefficients);
      void execute();
      void execute(bool debug);

    private:
      suturo_perception_utils::Logger logger_;
      suturo_perception_lib::PerceivedObject &perceived_object_;
      bool table_mode_;
      pcl::ModelCoefficients::Ptr table_coefficients_;
  };
}
#endif
