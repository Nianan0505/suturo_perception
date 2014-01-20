// TODO
// If the homography contains crossings, discard the result

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
#include "math2d.h"

// Include concrete Matching Strategies
#include "nndr_matcher.h"

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
}

void ObjectMatcher::drawBoundingBoxWithCrossings(bool draw)
{
	this->draw_bounding_box_with_crossings_ = draw;
}

ObjectMatcher::ObjectMatcher()
{
  // Detect the keypoints using SURF Detector per default with a minHessian of 400
  int minHessian = 400;
  detector_ = new cv::SURF(minHessian);
  extractor_ = new SurfDescriptorExtractor(); // SurfDescriptorExtractor produces more positive matches then cv::SURF(400)?
	this->matching_strategy_ = new NNDRMatcher();
  min_good_matches_ = 0;
  draw_bounding_box_with_crossings_ = true;
  verbose_level = VERBOSE_EXTRA;
}

ObjectMatcher::ObjectMatcher(cv::Ptr<cv::FeatureDetector> detector, cv::Ptr<cv::DescriptorExtractor> extractor)
{
  this->detector_ = detector;
  this->extractor_ = extractor;
	this->matching_strategy_ = new NNDRMatcher();
  min_good_matches_ = 0;
  draw_bounding_box_with_crossings_ = true;
  verbose_level = VERBOSE_EXTRA;
}

void ObjectMatcher::computeKeyPointsAndDescriptors(Mat &img, std::vector<cv::KeyPoint> &keypoints, Mat &descriptors)
{
  detector_->detect( img, keypoints);
  extractor_->compute( img, keypoints, descriptors );
}

void ObjectMatcher::calculateHomography(std::vector<cv::KeyPoint> &keypoints_object, std::vector<cv::KeyPoint> &keypoints_scene, std::vector<DMatch> &good_matches, Mat &H, std::vector<uchar> &outlier_mask ){
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
      //-- Get the keypoints from the good matches
      obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
      scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    H = findHomography( obj, scene, CV_RANSAC, 1.0, outlier_mask);
}

// Decide, if the object has been found
// @param H The computed homography during the object recognition. Will only be filled when the object 
// @return true, if the good matches and the keypoints meet the criteria for "Object recognized". false otherwise
bool ObjectMatcher::objectRecognized(std::vector< DMatch > &good_matches, std::vector<cv::KeyPoint> &keypoints_object, std::vector<cv::KeyPoint> &keypoints_scene, Mat &H){

  // Don't consider object recognition, if the minimum amount of good matches has not been reached
  // But, atleast 4 good matches must be found in order to draw the homography
  if(good_matches.size() >= 4 && good_matches.size() >= min_good_matches_)
	{

    std::vector<uchar> outlier_mask;  // Used for homography
    calculateHomography(keypoints_object, keypoints_scene, good_matches, H, outlier_mask );
    

    // Check the ratio of inliers against the size of good matches
    if( isInlierRatioHighEnough( outlier_mask, good_matches.size()) )
      return true; // Object recognized
    return false;

  }else{
    if(verbose_level>=VERBOSE_NORMAL)
      std::cout << "Didn't try to compute homography. Either good_matches is not greater than 4 or min_good_matches_ is not reached. Good matches: " << good_matches.size() << " min_good_matches_ : " << min_good_matches_ << endl;
  }
  return false;
}

// TODO refactor core functions of the following two functions

void ObjectMatcher::trainImages(vector<string> file_names)
{
  std::cout << "Train given training images" << std::endl;

  for(int i = 0; i < file_names.size(); i++)
  {
    TrainingImageData ti;
    ti.img = imread( file_names.at(i), CV_LOAD_IMAGE_GRAYSCALE );
    if( !(ti.img.data))
    { std::cout<< " --(!) Error reading images during training" << std::endl; exit(0); }

    computeKeyPointsAndDescriptors( ti.img, ti.keypoints, ti.descriptors);
    training_images_.push_back(ti);
    std::cout << "Image: " << file_names.at(i) << " stored" << std::endl;
    std::cout << "Keypoints: " << ti.keypoints.size() << std::endl;
  }
}
void ObjectMatcher::trainImages(vector<string> file_names, vector<string> labels)
{
  if(file_names.size() != labels.size() )
    return;

  std::cout << "Train given training images" << std::endl;

  for(int i = 0; i < file_names.size(); i++)
  {
    TrainingImageData ti;
    ti.img = imread( file_names.at(i), CV_LOAD_IMAGE_GRAYSCALE );
    if( !(ti.img.data))
    { std::cout<< " --(!) Error reading images during training" << std::endl; exit(0); }

    computeKeyPointsAndDescriptors( ti.img, ti.keypoints, ti.descriptors);
    ti.label = labels.at(i);
    training_images_.push_back(ti);
    std::cout << "Image: " << file_names.at(i) << " stored" << std::endl;
    std::cout << "Keypoints: " << ti.keypoints.size() << std::endl;
  }
}

void ObjectMatcher::trainImagesToDatabase(string database_filename, vector<string> training_img_file, vector<string> training_img_label)
{
  // vector of training images should be equally sized
  // to the vector of labels for it
  if(training_img_file.size() != training_img_label.size())
    return;

  cv::FileStorage fs(database_filename, cv::FileStorage::WRITE);
 
  // Save everything in a mapping "training_data", which contains
  // a list (e.g. sequence in YAML terms) of
  // keypoints,descriptors and the training image itself
  // fs << "uber_keypoints " << 
  fs << "training_data" << "[";
  for(int i = 0; i < training_img_file.size(); i++)
  {
    // Read in training images
    Mat img, descriptors;
    img = imread( training_img_file.at(i), CV_LOAD_IMAGE_GRAYSCALE );
    vector<KeyPoint> keypoints;

    computeKeyPointsAndDescriptors(img, keypoints, descriptors);
    cout << "Trained " << keypoints.size() << " keypoints" << endl;

    // fs << "{:" << "keypoints" << keypoints << "descriptors" << descriptors;
    fs << "{" << "label" << training_img_label.at(i) << "keypoints" << keypoints ;
    fs << "img" << img;
    fs << "descriptors" << descriptors;
    fs << "}";
  }
  fs << "]";
  fs.release();

}

void ObjectMatcher::readTrainImagesFromDatabase(string database_filename)
{
  cout << "Reading training data from " << database_filename << endl;
  cv::FileStorage fs(database_filename, FileStorage::READ);
  FileNode training_data = fs["training_data"];

  int idx=0;
  FileNodeIterator it = training_data.begin(), it_end = training_data.end();
  for( ; it != it_end; ++it, idx++ )
  {
    TrainingImageData ti;

    (*it)["label"] >> ti.label;
    // (*it)["keypoints"] >> kp; // doesn't work ?
    read( (*it)["keypoints"] , ti.keypoints); // works
    (*it)["img"] >> ti.img;
    (*it)["descriptors"] >> ti.descriptors;

    cout << "feature #" << idx << ": " << ti.label << " - Keypoints: " << ti.keypoints.size() << endl;
    training_images_.push_back(ti);
  }
  fs.release();
}

ObjectMatcher::ExecutionResult ObjectMatcher::recognizeTrainedImages(cv::Mat &test_image, bool headless)
{
  Mat &img_scene = test_image;
  if( !img_scene.data )
  { std::cout<< " --(!) Error reading test_image in recognizeTrainedImages " << std::endl; exit(0); }
  // cout << "Train image" << train_image << "; Test image: " << test_image << endl;
  
  // Calculate keypoints and descriptors
  std::vector<KeyPoint> keypoints_scene;
  Mat descriptors_scene;

  computeKeyPointsAndDescriptors( img_scene, keypoints_scene, descriptors_scene);
  Mat img_matches;

  // At least 1 keypoint has to be present to continue
  if(keypoints_scene.size() == 0){
    ExecutionResult result;
    result.object_recognized = false;
    result.match_image = img_matches;
    result.label = "";
    cerr << "No Keypoints found in scene!" << endl;
    return result;
  }

  /*
   * To recognize an object, proceed with the following steps:
   *  - Match all training images against the scene
   *  - Store the Vector with the Correspondences between a training
   *    image and the scene in a list
   *  - Evaluate afterwards, which training images has the most correspondences with the scene
   *  - Return this object as the best match
   */

  // true, if objectRecognized() was true on the training_image at index i
  std::vector<bool> object_recognized;
  std::vector<std::vector< DMatch > > good_matches_for_train_image;

  // Check these descriptors against all stored training images
  for(int i = 0; i < training_images_.size(); i++)
  {
    TrainingImageData &ti = training_images_.at(i);

    // Matching descriptor vectors using the given MatchingStrategy
    std::vector< DMatch > good_matches;
    this->matching_strategy_->match(ti.descriptors, descriptors_scene, good_matches);

    drawMatches( ti.img, ti.keypoints, img_scene, keypoints_scene,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


    bool return_value = false;

    // Mat for the computed homography, if the object has been found
    Mat Hom;

    if(objectRecognized(good_matches, ti.keypoints, keypoints_scene, Hom))
    {
      object_recognized.push_back(true);
    }
    else
    {
      object_recognized.push_back(false);
    }
    good_matches_for_train_image.push_back(good_matches);

    Mat H;
    if(objectRecognized(good_matches, ti.keypoints, keypoints_scene, H))
    {
        // Only calculate bounding box if running with GUI
        bool crossings = drawBoundingBoxFromHomography(H, ti.img, img_matches);
        // if(!headless){
        if(crossings){
          if(verbose_level>=VERBOSE_NORMAL){
            cout << "Bounding Box lines contain crossings. Discard the result... " << endl;
            // Skip this iteration if crossings occured
            cout << "Object recognized: [ ]" << endl;
          }
          continue;
          // return_value=false;
        }
        else
        {
          return_value=true;
        }
        //-- Show detected matches
        if(!headless)
        {
          imshow( "Good Matches & Object detection", img_matches );
          waitKey(0);
        }

        // Exit on the first non-crossing match (for now)
        ExecutionResult result;
        result.object_recognized = return_value;
        result.match_image = img_matches;
        if(!ti.label.empty() && return_value ){
          result.label = ti.label; // Do we have to copy the string here?
        }
        if(verbose_level>=VERBOSE_NORMAL)
          cout << "Object recognized: [x]" << endl;
        return result;
    }else{
        if(verbose_level>=VERBOSE_NORMAL)
          cout << "Object recognized:[ ]"<<endl;
    }
  }

  int max_match_id = 0;
  int max_match_count = 0;
  bool one_image_matched = false;

  // TODO get the best result
  for (int i = 0; i < training_images_.size(); i++) {
      // Could the object at index i be recognized?
      if( object_recognized.at(i) )
      {
        if( good_matches_for_train_image.at(i).size() >= max_match_count )
        {
          max_match_count = good_matches_for_train_image.at(i).size();
          max_match_id = i;
          one_image_matched = true;
        }
      }
  }

  // Return the results if atleast one image has matched the criteria
  // Return the one with the most matches
  /*
  if(one_image_matched)
  {

  }
  */


  // No match has been found? return false with an empty image
  // and show the scene without any matches
  if(!headless)
  {
    putText(img_scene, "No Match found", cvPoint(20,20), 
    FONT_HERSHEY_PLAIN, 1.5, cvScalar(0,0,0), 2, CV_AA);

    imshow( "Good Matches & Object detection", img_scene );
    waitKey(0);
  }

  Mat empty;
  ExecutionResult result;
  result.object_recognized = false;
  result.match_image = img_matches; 
  return result;
}
ObjectMatcher::ExecutionResult ObjectMatcher::recognizeTrainedImages(std::string test_image, bool headless)
{
  Mat img_scene = imread( test_image, CV_LOAD_IMAGE_GRAYSCALE );

  return recognizeTrainedImages(img_scene, headless);
}


ObjectMatcher::ExecutionResult ObjectMatcher::execute(std::string train_image, std::string test_image, bool headless)
{
  Mat img_object = imread( train_image, CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_scene = imread( test_image, CV_LOAD_IMAGE_GRAYSCALE );

  if( !img_object.data || !img_scene.data )
  { std::cout<< " --(!) Error reading images " << std::endl; exit(0); }
  cout << "Train image" << train_image << "; Test image: " << test_image << endl;
  // Calculate keypoints and descriptors
  std::vector<KeyPoint> keypoints_object, keypoints_scene;
  Mat descriptors_object, descriptors_scene;

  computeKeyPointsAndDescriptors( img_object, keypoints_object, descriptors_object);
  computeKeyPointsAndDescriptors( img_scene, keypoints_scene, descriptors_scene);

  // Matching descriptor vectors using the given MatchingStrategy
  std::vector< DMatch > good_matches;
	this->matching_strategy_->match(descriptors_object, descriptors_scene, good_matches);

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


  cout << "Training image keypoint count: " <<keypoints_object.size() << endl;
  cout << "Test image keypoint count: " << keypoints_scene.size() << endl;
  cout << "Good match count: " << good_matches.size() << std::endl;
  cout << "Minimum good matches: " << min_good_matches_ << std::endl;

  bool return_value = false;

  // Mat for the computed homography, if the object has been found
  Mat H;
  if(objectRecognized(good_matches, keypoints_object, keypoints_scene, H))
  {
      cout << "Object recognized: [X]" << endl;
      // Only calculate bounding box if running with GUI
      if(!headless){
        bool crossings = drawBoundingBoxFromHomography(H, img_object, img_matches);
        cout << "Does the bounding box have intersections? " << crossings << endl;
      }
      return_value=true;
  }else{
      cout << "Object recognized:[ ]"<<endl;
  }

  //-- Show detected matches
  if(!headless)
  {
    imshow( "Good Matches & Object detection", img_matches );
    waitKey(0);
  }
  ExecutionResult result;
  result.object_recognized = return_value;
  result.match_image = img_matches;
  return result;
}

// Take the outlier_mask output of findHomography and calculate how many
// of the good matches are inliers of the homography
// @return true, if the amount of inliers divided by good_match_count is higher
// then ObjectMatcher::inlier_ratio_
bool ObjectMatcher::isInlierRatioHighEnough(std::vector<uchar> &outlier_mask, int good_match_count){
    int inliers=0, outliers=0;
    for(unsigned int k = 0; k < good_match_count; ++k)
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

  // The homography must contain atleast 40% of the extracted keypoints in the homography
  // for a "good match"
  // If the inlier to outlier ratio is above this threshold, we will draw the bounding box
  // and count the object as recognized
  float actualInlierRatio = static_cast<float>(inliers) / static_cast<float>(good_match_count);

  if(verbose_level>=VERBOSE_NORMAL)
  {
    std::cout << "Homography Inlier count: " << inliers << endl;
    std::cout << "Homography Outlier count: " << outliers << endl;
    std::cout << "InlierRatio: " << actualInlierRatio << std::endl;
  }

  if( actualInlierRatio >= inlier_ratio_)
    return true;
  return false;
}

void ObjectMatcher::setMinGoodMatches(int min)
{
 min_good_matches_ = min;
}

void ObjectMatcher::setVerboseLevel(int level)
{
 verbose_level = level;
}

// return true, if bounding box contains crossings
bool ObjectMatcher::drawBoundingBoxFromHomography(Mat &H, Mat &img_object, Mat &img_matches)
{
  bool lines_intersect = false;
  //-- Get the corners from the image_1 ( the object to be "detected" on the left)
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0);
  obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
  obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  cv::Point2f intersection;
  if(intersectLineSegments(scene_corners[0], scene_corners[1], scene_corners[2], scene_corners[3], intersection)
      || intersectLineSegments(scene_corners[0], scene_corners[3], scene_corners[1], scene_corners[2], intersection)
      )
  {
    lines_intersect = true;
  }

  if(!lines_intersect || (lines_intersect && draw_bounding_box_with_crossings_ ) )
  {
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 3 );
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 255), 3 );
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 255, 0, 0), 3);
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 255, 255, 0), 3 );
  }
  return lines_intersect;
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
