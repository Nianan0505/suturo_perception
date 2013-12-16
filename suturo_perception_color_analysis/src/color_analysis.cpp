#include "color_analysis.h"

using namespace suturo_perception_color_analysis;

ColorAnalysis::ColorAnalysis(PerceivedObject &obj, boost::signals2::mutex &m) : Capability(obj, m)
{
  logger = suturo_perception_utils::Logger("color_analysis");
}

uint32_t
ColorAnalysis::getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  if(cloud_in->points.size() == 0) return 0;

  double average_r = 0;
  double average_g = 0;
  double average_b = 0;
  for(int i = 0; i < cloud_in->points.size(); ++i)
  {
    uint32_t rgb = *reinterpret_cast<int*>(&cloud_in->points[i].rgb);
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8) & 0x0000ff;
    uint8_t b = (rgb) & 0x0000ff;
    average_r += (double)r / (double)cloud_in->points.size();
    average_g += (double)g / (double)cloud_in->points.size();
    average_b += (double)b / (double)cloud_in->points.size();
  }

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "getAverageColor()");

  return ((uint32_t)average_r << 16 | (uint32_t)average_g << 8 | (uint32_t)average_b);

}

HSVColor
ColorAnalysis::getAverageColorHSV(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  return convertRGBToHSV(getAverageColor(cloud_in));
}

std::vector<uint32_t> *
ColorAnalysis::getHistogramHue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  std::vector<uint32_t> *ret = new std::vector<uint32_t>(121); // 360 / 3 = 120
  uint32_t excluded_point_cnt = 0;

  if(cloud_in->points.size() == 0) return ret;

  for (int i = 0; i < (*ret).size(); i++)
  {
    ret->at(i) = 0;
  }

  for(int i = 0; i < cloud_in->points.size(); ++i)
  {
    uint32_t rgb = *reinterpret_cast<int*>(&cloud_in->points[i].rgb);
    HSVColor hsv = convertRGBToHSV(rgb);

    if (hsv.s < 0.3 || hsv.v < 0.3)
    {
      excluded_point_cnt++;
      continue;
    }

    if (hsv.h/3 > 120)
      logger.logWarn((boost::format("bad shit going to happen! %d") % (hsv.h / 3)).str());
    ret->at(hsv.h/3) ++;
  }

  histogram_quality = (uint8_t) (100.0 - ((100.0 / (double) cloud_in->points.size()) * (double) excluded_point_cnt));

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "getHistogramHue()");

  return ret;
}

/*
 * Convert rgb color to hsv color
 * colors are in packed value format. the msb is used as debug flag, everthing != 0
 * turns on debugging
 * based on: http://en.literateprograms.org/RGB_to_HSV_color_space_conversion_%28C%29
 */
HSVColor
ColorAnalysis::convertRGBToHSV(uint32_t rgb) 
{
  HSVColor hsv;
  double r = ((uint8_t) ((rgb & 0xff0000) >> 16)) / 255.0;
  double g = ((uint8_t) ((rgb & 0xff00) >> 8)) / 255.0;
  double b = ((uint8_t) (rgb & 0xff)) / 255.0;
  //logger.logWarn((boost::format("convertRGBToHSV input: %f, %f, %f") % r % g % b).str());
  double rgb_min, rgb_max;
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(1) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  hsv.v = rgb_max;
  if (hsv.v == 0) {
      hsv.h = hsv.s = 0;
      return hsv;
  }
  /* Normalize value to 1 */
  r /= hsv.v;
  g /= hsv.v;
  b /= hsv.v;
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(2) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  hsv.s = rgb_max - rgb_min;
  if (hsv.s == 0) {
      hsv.h = 0;
      return hsv;
  }
  /* Normalize saturation to 1 */
  r = (r - rgb_min)/(rgb_max - rgb_min);
  g = (g - rgb_min)/(rgb_max - rgb_min);
  b = (b - rgb_min)/(rgb_max - rgb_min);
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(3) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  /* Compute hue */
  if (rgb_max == r) {
      int h_tmp = 0.0 + 60.0*(g - b);
      //logger.logWarn((boost::format("convertRGBToHSV hue: %f, %f, %f") % hsv.h % g % b).str());
      if (h_tmp < 0.0) {
          h_tmp += 360.0;
      }
      hsv.h = h_tmp;
      //logger.logWarn((boost::format("convertRGBToHSV hue2: %f") % hsv.h).str());
  } else if (rgb_max == g) {
      hsv.h = 120.0 + 60.0*(b - r);
      //logger.logWarn((boost::format("convertRGBToHSV hue3: %f") % hsv.h).str());
  } else /* rgb_max == b */ {
      hsv.h = 240.0 + 60.0*(r - g);
      //logger.logWarn((boost::format("convertRGBToHSV hue4: %f") % hsv.h).str());
  }
  return hsv;
}

/*
 * convert hsv to rgb color
 * taken from: http://stackoverflow.com/a/6930407
 */
uint32_t
ColorAnalysis::convertHSVToRGB(HSVColor hsv)
{
  double hh, p, q, t, ff;
  long i;
  uint8_t r,g,b;

  uint32_t h = hsv.h;
  double s = hsv.s;
  double v = hsv.v;

  if(s <= 0.0)
  {
    r = (uint8_t) v * 255;
    g = (uint8_t) v * 255;
    b = (uint8_t) v * 255;
    return (r << 16) | (g << 8) | b;
  }
  hh = h;
  if(hh >= 360.0) 
    hh = 0.0;
  hh /= 60;
  i = (long)hh;
  ff = hh - i;
  p = v * (1 - s);
  q = v * (1 - (s * ff));
  t = v * (1 - (s * (1 - ff)));

  switch(i) 
  {
  case 0:
    r = (uint8_t) (v * 255);
    g = (uint8_t) (t * 255);
    b = (uint8_t) (p * 255);
    break;
  case 1:
    r = (uint8_t) (q * 255);
    g = (uint8_t) (v * 255);
    b = (uint8_t) (p * 255);
    break;
  case 2:
    r = (uint8_t) (p * 255);
    g = (uint8_t) (v * 255);
    b = (uint8_t) (t * 255);
    break;
  case 3:
    r = (uint8_t) (p * 255);
    g = (uint8_t) (q * 255);
    b = (uint8_t) (v * 255);
    break;
  case 4:
    r = (uint8_t) (t * 255);
    g = (uint8_t) (p * 255);
    b = (uint8_t) (v * 255);
    break;
  case 5:
  default:
    r = (uint8_t) (v * 255);
    g = (uint8_t) (p * 255);
    b = (uint8_t) (q * 255);
    break;
  }
  return (r << 16) | (g << 8) | b;
}

cv::Mat
ColorAnalysis::histogramToImage(boost::shared_ptr<std::vector<uint32_t> > histogram)
{
  uint32_t hw = 1024;
  uint32_t hh = 768;
  int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double fontScale = 0.75;
  int thickness = 3;
  cv::Scalar bg_color(255,255,255);
  cv::Scalar fg_color(0,0,0);

  cv::Mat hist(cv::Size(hw,hh), CV_8UC3, bg_color); 

  int baseline = 0;
  cv::Size txt_size_yaxis = cv::getTextSize(boost::lexical_cast<std::string>(9999), fontFace, fontScale, thickness, &baseline);
  baseline += thickness;

  cv::line(hist, cv::Point(10 + txt_size_yaxis.width, 10), cv::Point(10 + txt_size_yaxis.width, hh - 20), fg_color);
  cv::line(hist, cv::Point(10 + txt_size_yaxis.width, hh - 20), cv::Point(hw - 20, hh - 20), fg_color);
  
  uint32_t max_h = 0;
  for (int j = 0; j < histogram->size(); j++)
  {
    max_h = std::max(max_h, histogram->at(j));
  }
  int step = max_h / 10;
  int y_axis_txt = 0;
  for (int j = 0; j < 11; j++)
  {
    cv::Point text_org(5, hh - j*(hh / 10) - 20);
    cv::putText(hist, boost::lexical_cast<std::string>(y_axis_txt), text_org, fontFace, fontScale, fg_color, thickness, 8);
    y_axis_txt += step;
  }
  int x_axis_width = hw - 20 - txt_size_yaxis.width - 10;
  uint8_t hue = 0;
  for (int j = 0; j < x_axis_width; j++)
  {
    // draw xaxis color line
    hue = (uint8_t) ((255.0 / (double)x_axis_width) * j);

    HSVColor hsv_tmp;
    hsv_tmp.h = hue;
    hsv_tmp.s = 1.0;
    hsv_tmp.v = 1.0;
    uint32_t tmp_color = convertHSVToRGB(hsv_tmp);
    cv::Scalar x_color(
      tmp_color & 0xff,
      (tmp_color & 0xff00) >> 8,
      (tmp_color & 0xff0000) >> 16);

    cv::line(hist, cv::Point(10 + txt_size_yaxis.width + j, hh - 20), cv::Point(10 + txt_size_yaxis.width + j, hh), x_color);
  }
  //cv::Point from(txt_size_yaxis.width + 5, hh - 20 - (uint32_t) (((double)hh - 20) / (double) max_h) * histogram->at(0) );
  cv::Point from(
      txt_size_yaxis.width + 10, 
      hh - 20 - (uint32_t) ((((double)hh - 20) / (double) max_h)) * (double) histogram->at(0) );
  cv::Point to(0,0);
  //std::stringstream ykoors;
  for (int j = 1; j < histogram->size(); j++)
  {
    to.x = txt_size_yaxis.width + 10 + j * ( (hw - txt_size_yaxis.width - 20) / histogram->size() );
    to.y = hh - 20 - (uint32_t) ((((double)hh - 20) / (double) max_h)) * (double) histogram->at(j);

    //ykoors << "(" << to.x << "," << to.y << ") ";

    cv::line(hist, from, to, fg_color);

    from.x = to.x;
    from.y = to.y;
  }
  //logger.logInfo((boost::format("[histogram image] max_h = %s, a = %s, b = %s, c = %s") % max_h % (uint32_t) ((((double)hh - 20) / (double) max_h)) % (hh - 20 - (uint32_t) ((((double)hh - 20) / (double) max_h)) * (double) histogram->at(0)) % ykoors.str()).str());
  return hist;
}

uint8_t
ColorAnalysis::getHistogramQuality()
{
  return histogram_quality;
}


void
ColorAnalysis::execute()
{
  // Get average color of the object
  uint32_t averageColor = getAverageColor(perceivedObject.get_pointCloud());
  HSVColor averageColorHSV = convertRGBToHSV(averageColor);

  // Get hue histogram of the object
  std::vector<uint32_t> *histogram = getHistogramHue(perceivedObject.get_pointCloud());
  uint8_t histogram_quality = getHistogramQuality();

  // generate image of histogram
  //perceived_cluster_histograms_.push_back(histogramToImage(histogram));
  perceivedObject.set_c_color_average_r((averageColor >> 16) & 0x0000ff);
  perceivedObject.set_c_color_average_g((averageColor >> 8)  & 0x0000ff);
  perceivedObject.set_c_color_average_b((averageColor)       & 0x0000ff);
  perceivedObject.set_c_color_average_h(averageColorHSV.h);
  perceivedObject.set_c_color_average_s(averageColorHSV.s);
  perceivedObject.set_c_color_average_v(averageColorHSV.v);
  perceivedObject.set_c_hue_histogram(histogram);
  perceivedObject.set_c_hue_histogram_quality(histogram_quality);
}
