#ifndef PERCEIVED_OBJECT_H
#define PERCEIVED_OBJECT_H

#include "point.h"
#include "roi.h"
#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include "opencv2/core/core.hpp"

namespace suturo_perception_lib
{
  class PerceivedObject
  {
    public:
      PerceivedObject() : mutex(new boost::signals2::mutex()) {};
      //PerceivedObject(boost::signals2::mutex &m) : mutex(m) {};
      //void setCommonMutex(boost::signals2::mutex *m) {mutex = m;};

      // Threadsafe getters      
      int get_c_id() const
      {
        mutex->lock(); 
        int res = c_id;
        mutex->unlock();
        return res; 
      };
      
      Point get_c_centroid() const
      {
        mutex->lock();
        Point res = c_centroid;
        mutex->unlock();
        return res;
      };
      
      double get_c_volume() const
      {
        mutex->lock(); 
        double res = c_volume;
        mutex->unlock();
        return res;
      };
      
      int get_c_shape() const
      {
        mutex->lock(); 
        int res = c_shape;
        mutex->unlock();
        return res; 
      };
      
      uint8_t get_c_color_average_r() const
      {
        mutex->lock(); 
        uint8_t res = c_color_average_r;
        mutex->unlock();
        return res; 
      };

      uint8_t get_c_color_average_g() const
      {
        mutex->lock(); 
        uint8_t res = c_color_average_g; 
        mutex->unlock();
        return res;
      };

      uint8_t get_c_color_average_b() const
      {
        mutex->lock(); 
        uint8_t res = c_color_average_b; 
        mutex->unlock();
        return res;
      };

      uint32_t get_c_color_average_h() const
      {
        mutex->lock(); 
        uint32_t res = c_color_average_h; 
        mutex->unlock();
        return res;
      };

      double get_c_color_average_s() const
      {
        mutex->lock(); 
        double res = c_color_average_s; 
        mutex->unlock();
        return res;
      };

      double get_c_color_average_v() const
      {
        mutex->lock(); 
        double res = c_color_average_v; 
        mutex->unlock();
        return res;
      };

      std::string get_c_recognition_label_2d() const
      {
        mutex->lock(); 
        std::string res = c_recognition_label_2d; 
        mutex->unlock();
        return res;
      };

      std::vector<uint32_t>* get_c_hue_histogram() const
      {
        mutex->lock(); 
        std::vector<uint32_t> *res = c_hue_histogram; 
        mutex->unlock();
        return res;
      };

      uint8_t get_c_hue_histogram_quality() const
      {
        mutex->lock(); 
        uint8_t res = c_hue_histogram_quality; 
        mutex->unlock();
        return res;
      };

      cv::Mat *get_c_hue_histogram_image() const
      {
        mutex->lock();
        cv::Mat *res = c_hue_histogram_image;
        mutex->unlock();
        return res;
      }

      ROI get_c_roi() const
      {
        mutex->lock(); 
        ROI res = c_roi; 
        mutex->unlock();
        return res;
      };

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_pointCloud() const
      {
        mutex->lock(); 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr res = pointCloud; 
        mutex->unlock();
        return res;
      };

      // Threadsafe setters
      void set_c_id(int value)
      {
        mutex->lock();
        c_id = value;
        mutex->unlock();
      };
      void set_c_centroid(Point value)
      {
        mutex->lock();
        c_centroid = value;
        mutex->unlock();
      };
      void set_c_volume(double value)
      {
        mutex->lock();
        c_volume = value;
        mutex->unlock();
      };
      void set_c_shape(int value)
      {
        mutex->lock();
        c_shape = value;
        mutex->unlock();
      };
      void set_c_color_average_r(uint8_t value)
      {
        mutex->lock();
        c_color_average_r = value;
        mutex->unlock();
      };
      void set_c_color_average_g(uint8_t value)
      {
        mutex->lock();
        c_color_average_g = value;
        mutex->unlock();
      };
      void set_c_color_average_b(uint8_t value)
      {
        mutex->lock();
        c_color_average_b = value;
        mutex->unlock();
      };
      void set_c_color_average_h(uint32_t value)
      {
        mutex->lock();
        c_color_average_h = value;
        mutex->unlock();
      };
      void set_c_color_average_s(double value)
      {
        mutex->lock();
        c_color_average_s = value;
        mutex->unlock();
      };
      void set_c_color_average_v(double value)
      {
        mutex->lock();
        c_color_average_v = value;
        mutex->unlock();
      };
      void set_c_recognition_label_2d(std::string value)
      {
        mutex->lock();
        c_recognition_label_2d = value;
        mutex->unlock();
      };
      void set_c_hue_histogram(std::vector<uint32_t> *value)
      {
        mutex->lock();
        c_hue_histogram = value;
        mutex->unlock();
      };
      void set_c_hue_histogram_quality(uint8_t value)
      {
        mutex->lock();
        c_hue_histogram_quality = value;
        mutex->unlock();
      };
      void set_c_hue_histogram_image(cv::Mat *histImg)
      {
        mutex->lock();
        c_hue_histogram_image = histImg;
        mutex->unlock();
      }
      void set_c_roi(ROI value)
      {
        mutex->lock();
        c_roi = value;
        mutex->unlock();
      };
      void set_pointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr value)
      {
        mutex->lock();
        pointCloud = value;
        mutex->unlock();
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
      std::string c_recognition_label_2d;
      std::vector<uint32_t> *c_hue_histogram;
      uint8_t c_hue_histogram_quality;
      cv::Mat *c_hue_histogram_image;
      ROI c_roi;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;

      boost::shared_ptr<boost::signals2::mutex> mutex;
  };
}

#endif
