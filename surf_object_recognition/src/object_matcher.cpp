#include "object_matcher.h"
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include <boost/timer.hpp>      // For boost::timer class
#include <algorithm>
#include <iterator>

// Include concrete Matching Strategies
#include "simple_matcher.cpp"
#include "nndr_matcher.cpp"

using namespace boost;

// OpenCV 2.4 moved the SURF and SIFT libraries to a new nonfree/ directory
#if CV_MINOR_VERSION > 3
	#include <opencv2/nonfree/features2d.hpp>
#else
	#include <opencv2/features2d/features2d.hpp>
#endif

using namespace cv;
using namespace std;

void ObjectMatcher::setMatcher(MatchingStrategy* matching_strategy)
{
	this->matching_strategy_ = matching_strategy;
  min_good_matches_ = 0;
}

ObjectMatcher::ObjectMatcher()
{
  // Detect the keypoints using SURF Detector per default with a minHessian of 400
  int minHessian = 400;
  detector_ = new cv::SURF(minHessian);
  extractor_ = new SurfDescriptorExtractor(); // SurfDescriptorExtractor produces more positive matches then cv::SURF(400)?
	this->matching_strategy_ = new NNDRMatcher();
  min_good_matches_ = 0;
}

ObjectMatcher::ObjectMatcher(cv::Ptr<cv::FeatureDetector> detector, cv::Ptr<cv::DescriptorExtractor> extractor)
{
  this->detector_ = detector;
  this->extractor_ = extractor;
	this->matching_strategy_ = new NNDRMatcher();
}

bool ObjectMatcher::execute(std::string train_image, std::string test_image, bool headless)
{
  Mat img_object = imread( train_image, CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_scene = imread( test_image, CV_LOAD_IMAGE_GRAYSCALE );

  if( !img_object.data || !img_scene.data )
  { std::cout<< " --(!) Error reading images " << std::endl; exit(0); }
  cout << "Train image" << train_image << "; Test image: " << test_image << endl;

  // Start the execution timer
  boost::timer t;

  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector_->detect( img_object, keypoints_object );
  double keypointObjectElapsedTime = t.elapsed();
  detector_->detect( img_scene, keypoints_scene );
  double keypointSceneElapsedTime = t.elapsed();


  //-- Step 2: Calculate descriptors (feature vectors)
  Mat descriptors_object, descriptors_scene;

  extractor_->compute( img_object, keypoints_object, descriptors_object );
	double descriptorObjectElapsedTime = t.elapsed();
  extractor_->compute( img_scene, keypoints_scene, descriptors_scene );
	double descriptorSceneElapsedTime = t.elapsed();

  //-- Step 3: Matching descriptor vectors using the given MatchingStrategy
  std::vector< DMatch > good_matches;
	this->matching_strategy_->match(descriptors_object, descriptors_scene, good_matches);

	double matcherElapsedTime = t.elapsed();

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }
  double goodMatchesElapsedTime = t.elapsed();
  cout << "Training image keypoint count: " <<keypoints_object.size() << endl;
  cout << "Test image keypoint count: " << keypoints_scene.size() << endl;
  cout << "Good match count: " << good_matches.size() << std::endl;

  // if(good_matches.size() < min_good_matches_)
  //   return false;

  // Don't consider object recognition, if the minimum amount of good matches has not been reached
  // But, atleast 4 good matches must be found in order to draw the homography
  if(good_matches.size() >= 4 && good_matches.size() >= min_good_matches_)
	{
    std::vector<uchar> outlier_mask;  // Used for homography
    Mat H = findHomography( obj, scene, CV_RANSAC, 1.0, outlier_mask);
    double homographyElapsedTime = t.elapsed();
    int inliers=0, outliers=0;
    for(unsigned int k=0; k<obj.size();++k)
    {
      if(outlier_mask.at(k))
      {
        ++inliers;
      }
      else
      {
        ++outliers;
      }
    }

    // The homography must contain atleast 70% of the extracted keypoints in the homography
    // for a "good match"
    // If the inlier to outlier ratio is above this threshold, we will draw the bounding box
    // and count the object as recognized
    float inlierRatio = 0.4;
    float actualInlierRatio = static_cast<float>(inliers) / static_cast<float>(good_matches.size());
    std::cout << "InlierRatio: " << actualInlierRatio << std::endl;
    std::cout << "Recognized the object in the scene: ";
    if( good_matches.size() > 0 && actualInlierRatio >= inlierRatio){
      // Object recognized
      cout << "[X]" << endl;

      // Only calculate bounding box if running with GUI
      if(!headless){
        //-- Get the corners from the image_1 ( the object to be "detected" on the left)
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
        obj_corners[3] = cvPoint( 0, img_object.rows );
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform( obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
      }
    }
    else
    {
      cout << "[ ]"<<endl;
    }

    std::cout << "Homography Inlier count: " << inliers << endl;
    std::cout << "Homography Outlier count: " << outliers << endl;
  }else{
    std::cout << "Didn't try to compute homography. Either good_matches is not greater than 4 or min_good_matches_ is not reached" << endl;
  }
  double totalElapsedTime = t.elapsed();
  // std::cout << std::endl << "#### Time measurements #### " << std::endl;
  // std::cout << "The method ran in " << totalElapsedTime << "seconds" << std::endl;
  // std::cout << "Keypoints calculated at " << keypointObjectElapsedTime << "seconds for the object and at "<< keypointSceneElapsedTime << " seconds for the scene " << std::endl;
  // std::cout << "Descriptors calculated at " << descriptorObjectElapsedTime << "seconds for the object and at "<< descriptorSceneElapsedTime << " seconds for the scene " << std::endl;


  //-- Show detected matches
  if(!headless)
    imshow( "Good Matches & Object detection", img_matches );

  waitKey(0);
}

void ObjectMatcher::setMinGoodMatches(int min){
 min_good_matches_ = min;
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
