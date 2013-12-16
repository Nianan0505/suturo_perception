#ifndef SUTURO_PERCEPTION_COLOR_ANALYSIS
#define SUTURO_PERCEPTION_COLOR_ANALYSIS

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "opencv2/core/core.hpp"
#include <boost/signals2/mutex.hpp>

#include "suturo_perception_utils.h"
#include "capability.h"
#include "perceived_object.h"

using namespace suturo_perception_lib;

namespace suturo_perception_color_analysis
{
  typedef struct HSVColor_ {
    uint32_t h;
    double s;
    double v;
  } HSVColor;

  class ColorAnalysis : public Capability
  {
    public:
      ColorAnalysis(PerceivedObject &obj, boost::signals2::mutex &m);

      uint32_t getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      HSVColor getAverageColorHSV(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      boost::shared_ptr<std::vector<uint32_t> > getHistogramHue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      HSVColor convertRGBToHSV(uint32_t rgb);
      uint32_t convertHSVToRGB(HSVColor hsv);
      cv::Mat histogramToImage(boost::shared_ptr<std::vector<uint32_t> > histogram);
      uint8_t getHistogramQuality();
      std::vector<cv::Mat> getPerceivedClusterHistograms();

      // capability method
      void execute();

    private:
      suturo_perception_utils::Logger logger;
      uint8_t histogram_quality;
  };
}
#endif 
