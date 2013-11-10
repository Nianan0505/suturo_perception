#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

int
  main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

	if (argc < 3)
	{
		fprintf (stderr, "Usage: convert_pcd_to_rgb_image point_cloud.pcd filename_of_output_image.jpg\n");
		return 1;
	}

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file %s\n", argv[1]);
		return -1;
	}

	cout << "The PC has " << static_cast<int>(cloud.points.size() ) << " Points" << endl;
	cout << "The dimensions are " << cloud.width << "x"<<cloud.height << endl;

	// pcl::io::savePNGFile("foobar.png",cloud); // Not in PCL 1.6?

	// RGB Values for points
	int r,g,b;
	Mat img(Size(cloud.width,cloud.height),CV_8UC3);
	for(int row = 0; row < cloud.height; row++){
		for(int column = 0; column < cloud.width; column++){
			// (row,colum)
			img.at<Vec3b>(row,column)[0] = cloud.at(column,row).b;
			img.at<Vec3b>(row,column)[1] = cloud.at(column,row).g;
			img.at<Vec3b>(row,column)[2] = cloud.at(column,row).r;
		}
	}
  // imshow( "PC", img );
  // waitKey(0);
	imwrite(argv[2], img );

  return (0);
}
