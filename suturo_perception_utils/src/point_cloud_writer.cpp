#include "point_cloud_writer.h"

using namespace suturo_perception_utils;

void PointCloudWriter::writeCloudToDisk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, std::string filename="")
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ex_obj;
	ex_obj.push_back(point_cloud);
	PointCloudWriter::writeFirstCloudToDisk(ex_obj, filename);
}


// debug method
void PointCloudWriter::writeCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects)
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = extractedObjects.begin();
	pcl::PCDWriter writer;
	int cnt = 0;
	for(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = extractedObjects.begin(); 
			it != extractedObjects.end(); ++it)
	{
		std::stringstream ss;
		ss << "debug_pcd_" << cnt << ".pcd" << std::endl;
		writer.write(ss.str(), **it);
		++cnt;
	}
}

void PointCloudWriter::writeFirstCloudToDisk(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects, std::string filename)
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = extractedObjects.begin();
	pcl::PCDWriter writer;
	writer.write(filename, **it);
}
