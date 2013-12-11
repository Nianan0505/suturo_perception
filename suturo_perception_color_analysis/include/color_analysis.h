#ifndef SUTURO_PERCEPTION_COLOR_ANALYSIS
#define SUTURO_PERCEPTION_COLOR_ANALYSIS

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "opencv2/core/core.hpp"
#include "suturo_perception_utils.h"

namespace suturo_perception_color_analysis
{
  class ColorAnalysis 
  {
    public:
      ColorAnalysis();

      uint32_t getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      uint32_t getAverageColorHSV(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      boost::shared_ptr<std::vector<int> > getHistogramHue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      uint32_t convertRGBToHSV(uint32_t rgb);
      cv::Mat histogramToImage(boost::shared_ptr<std::vector<int> > histogram);

    private:
      suturo_perception_utils::Logger logger;
  };
}
#endif 
