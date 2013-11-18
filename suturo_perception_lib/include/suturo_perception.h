#ifndef MY_CLASS_H
#define MY_CLASS_H

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include "PerceivedObject.h"

class SuturoPerception
{
  public:
	void process_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
  std::vector<PerceivedObject> getPerceivedObjects();
  
  private:
	// ID counter for the perceived objects
	int objectID;

	// Buffer for the last perceived objects
	std::vector<PerceivedObject> perceivedObjects;
	
	// Mutex for buffer locking
	boost::signals2::mutex mutex;
};

#endif
