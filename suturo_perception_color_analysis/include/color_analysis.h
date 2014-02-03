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
      ColorAnalysis(PerceivedObject &obj);

      void allInOne(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      uint32_t getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      HSVColor getAverageColorHSV(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      HSVColor getAverageColorHSVQuality(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      std::vector<uint32_t> *getHistogramHue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      HSVColor convertRGBToHSV(uint32_t rgb);
      uint32_t convertHSVToRGB(HSVColor hsv);
      cv::Mat *histogramToImage(std::vector<uint32_t> *histogram);
      uint8_t getHistogramQuality();
      std::vector<cv::Mat> getPerceivedClusterHistograms();

      void setLowerSThreshold(double t) { s_lower_threshold = t; };
      void setUpperSThreshold(double t) { s_upper_threshold = t; };
      void setLowerVThreshold(double t) { v_lower_threshold = t; };
      void setUpperVThreshold(double t) { v_upper_threshold = t; };

      double getLowerSThreshold() { return s_lower_threshold; };
      double getUpperSThreshold() { return s_upper_threshold; };
      double getLowerVThreshold() { return v_lower_threshold; };
      double getUpperVThreshold() { return v_upper_threshold; };

      // capability method
      void execute();

    private:
      bool inHSVThreshold(HSVColor col);

      suturo_perception_utils::Logger logger;
      
      std::vector<uint32_t> *hueHistogram;
      uint8_t histogramQuality;
      uint32_t averageColor;
      HSVColor averageColorHSV;
      HSVColor averageColorHSVQuality;

      double s_lower_threshold;
      double s_upper_threshold;
      double v_lower_threshold;
      double v_upper_threshold;
  };
}
#endif 
