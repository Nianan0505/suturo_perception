#ifndef SUTURO_PERCEPTION_VFH_ESTIMATION
#define SUTURO_PERCEPTION_VFH_ESTIMATION

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/signals2/mutex.hpp>

#include "suturo_perception_utils.h"
#include "capability.h"
#include "perceived_object.h"

using namespace suturo_perception_lib;

namespace suturo_perception_vfh_estimation
{
  class VFHEstimation : public Capability
  {
    public:
      VFHEstimation(PerceivedObject &obj);

      pcl::VFHSignature308 estimateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

      // capability method
      void execute();

    private:
      suturo_perception_utils::Logger logger;
  };
}
#endif 
