// Based on
// http://siddhantahuja.wordpress.com/2011/07/20/working-with-ros-and-opencv-draft/ 

//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
 
#include <boost/program_options.hpp>
#include <algorithm>
#include <iterator>

#include "object_matcher.h"
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
namespace po = boost::program_options;

using namespace boost;
using namespace std;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

// The image processing class
ObjectMatcher om;

bool playback_paused = false;

static void onMouseClick( int event, int x, int y, int, void* )
{
	if( event != EVENT_LBUTTONDOWN )
    return;
	cout << "Clicked" << endl;
	playback_paused = !playback_paused;
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    // cout << "Received image " << endl;
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

 
		ObjectMatcher::ExecutionResult res = om.recognizeTrainedImages(cv_ptr->image, true);

    //Display the image using OpenCV
		if(res.match_image.data && !playback_paused)
		{

      if(res.object_recognized){
				if(!res.label.empty()){
					cout << "Recognized an object with the label " << res.label << endl;
				}
				else
				{
					cout << "Recognized an object with an empty label" << endl;
				}
			}else{
				cout << "No object recognized" << endl;
			}
			cv::imshow(WINDOW, res.match_image);
			cv::setMouseCallback(WINDOW, onMouseClick, 0 );
		}
     
 
    //Display the image using OpenCV
    // cv::imshow(WINDOW, cv_ptr->image);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
}

// class MouseClick
// {
// 	public:
// };
//
int main(int argc, char **argv)
{
	std::string database_file;
	std::string image_topic="";
  int min_good_matches;

	ros::init(argc, argv, "image_processor");

  // "HashMap" for program parameters
   po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("min-good-matches,m", po::value<int>(&min_good_matches)->default_value(0), "The minimum amount of good matches which must be present to perform object recognition")
      ("database-file,f", po::value<std::string>(&database_file)->required(), "Give a database file with training images and their keypoints/descriptors. By using a database, you don't need to specify your training images every time you call this node")
      ("topic,t", po::value<std::string>(&image_topic), "The ROS topic this node should listen to")
    ;

    po::positional_options_description p;
    po::store(po::command_line_parser(argc, argv).
    options(desc).positional(p).run(), vm); 

    if (vm.count("help")) {
      cout << "Usage: suturo_perception_live_2d -f database-file" << endl << endl;
      cout << desc << "\n";
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

  }
  catch(std::exception& e)
  {
		cout << "Usage: suturo_perception_live_2d -f database-file" << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

	// Configure Object Recognition Module
  om.setMinGoodMatches(min_good_matches);
	om.readTrainImagesFromDatabase(database_file);
  om.drawBoundingBoxWithCrossings(false);
	om.setVerboseLevel(0);

	ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
	image_transport::ImageTransport it(nh);
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

	cv::setMouseCallback(WINDOW, onMouseClick, 0 );
  // cout << "Given topic is " << image_topic << endl;
  // cout << "Given topic is " << image_topic.empty() << endl;
  image_transport::Subscriber sub;

  if(image_topic.empty())
      image_topic = "/camera/rgb/image_raw";

  sub = it.subscribe(image_topic, 1, imageCallback);
  // sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
  // if( ! image_topic.empty())
  // {
  //   cout << "Subscribed to topic " << image_topic << endl;
  //   sub = it.subscribe(image_topic, 1, imageCallback);
  // }
  // else
  // {
  //   sub= it.subscribe("camera/rgb/image_raw", 1, imageCallback);
  // }
	//OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW);

	pub = it.advertise("camera/rgb/image_processed", 1);
  cout << "Waiting for images" << endl;
	ros::spin();
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
