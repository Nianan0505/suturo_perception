#ifndef PERCEIVED_OBJECT_H
#define PERCEIVED_OBJECT_H

#include "point.h"
#include "roi.h"
#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include "opencv2/core/core.hpp"
#include <boost/thread.hpp>
#include <suturo_perception_match_cuboid/cuboid.h>

namespace suturo_perception_lib
{
  class PerceivedObject
  {
    public:
      PerceivedObject() : mutex(new boost::signals2::mutex()) {
        c_id = -1;
        c_centroid.x = -1;
        c_centroid.y = -1;
        c_centroid.z = -1;
        c_volume = -1.0;
        c_shape = 0;
        c_color_average_r = 0;
        c_color_average_g = 0;
        c_color_average_b = 0;
        c_color_average_h = 0;
        c_color_average_s = -1;
        c_color_average_v = -1;
        c_color_average_qh = 0;
        c_color_average_qs = -1;
        c_color_average_qv = -1;
        c_recognition_label_2d = "";
        c_hue_histogram_quality = 0;
        c_roi.origin.x = 0;
        c_roi.origin.y = 0;
        c_roi.width = 0;
        c_roi.height = 0;
        c_cuboid.length1 = -1;
        c_cuboid.length2 = -1;
        c_cuboid.length3 = -1;
        c_cuboid.volume = -1;

      };

      // Threadsafe getters      
      int get_c_id() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_id; 
      };
      
      Point get_c_centroid() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        return c_centroid;
      };
      
      double get_c_volume() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_volume;
      };
      
      int get_c_shape() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_shape;
      };
      
      uint8_t get_c_color_average_r() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_r;
      };

      uint8_t get_c_color_average_g() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_g; 
      };

      uint8_t get_c_color_average_b() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_b; 
      };

      uint32_t get_c_color_average_qh() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_qh; 
      };

      double get_c_color_average_qs() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_qs; 
      };

      double get_c_color_average_qv() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_qv; 
      };

      uint32_t get_c_color_average_h() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_h; 
      };

      double get_c_color_average_s() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_s; 
      };

      double get_c_color_average_v() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_average_v; 
      };

      std::string get_c_recognition_label_2d() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_recognition_label_2d; 
      };

      std::vector<uint32_t>* get_c_hue_histogram() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_hue_histogram; 
      };

      uint8_t get_c_hue_histogram_quality() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_hue_histogram_quality; 
      };

      cv::Mat *get_c_hue_histogram_image() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_hue_histogram_image;
      }

      ROI get_c_roi() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_roi; 
      };

      pcl::VFHSignature308 get_c_vfhs() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_vfhs; 
      };

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_pointCloud() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return pointCloud; 
      };

      Cuboid get_c_cuboid() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_cuboid; 
      };

      // Threadsafe setters
      void set_c_id(int value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_id = value;
      };
      void set_c_centroid(Point value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_centroid = value;
      };
      void set_c_volume(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_volume = value;
      };
      void set_c_shape(int value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_shape = value;
      };
      void set_c_color_average_r(uint8_t value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_r = value;
      };
      void set_c_color_average_g(uint8_t value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_g = value;
      };
      void set_c_color_average_b(uint8_t value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_b = value;
      };
      void set_c_color_average_qh(uint32_t value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_qh = value;
      };
      void set_c_color_average_qs(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_qs = value;
      };
      void set_c_color_average_qv(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_qv = value;
      };
      void set_c_color_average_h(uint32_t value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_h = value;
      };
      void set_c_color_average_s(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_s = value;
      };
      void set_c_color_average_v(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_average_v = value;
      };
      void set_c_recognition_label_2d(std::string value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_recognition_label_2d = value;
      };
      void set_c_hue_histogram(std::vector<uint32_t> *value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_hue_histogram = value;
      };
      void set_c_hue_histogram_quality(uint8_t value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_hue_histogram_quality = value;
      };
      void set_c_hue_histogram_image(cv::Mat *histImg)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_hue_histogram_image = histImg;
      }
      void set_c_roi(ROI value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_roi = value;
      };
      void set_c_vfhs(pcl::VFHSignature308 value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_vfhs = value;
      };
      void set_pointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        pointCloud = value;
      };
    
      void set_c_cuboid(Cuboid value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_cuboid = value;
      };
    
    private:
      int c_id;
      Point c_centroid;
      double c_volume;
      int c_shape;
      uint8_t c_color_average_r;
      uint8_t c_color_average_g;
      uint8_t c_color_average_b;
      uint32_t c_color_average_h;
      double c_color_average_s;
      double c_color_average_v;
      uint32_t c_color_average_qh;
      double c_color_average_qs;
      double c_color_average_qv;
      std::string c_recognition_label_2d;
      std::vector<uint32_t> *c_hue_histogram;
      uint8_t c_hue_histogram_quality;
      cv::Mat *c_hue_histogram_image;
      ROI c_roi;
      pcl::VFHSignature308 c_vfhs;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
      Cuboid c_cuboid;


      boost::shared_ptr<boost::signals2::mutex> mutex;
  };
}

#endif
