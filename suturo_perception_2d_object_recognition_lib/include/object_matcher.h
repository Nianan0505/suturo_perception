#ifndef OBJECT_MATCHER_H
#define OBJECT_MATCHER_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>

// OpenCV 2.4 moved the SURF and SIFT libraries to a new nonfree/ directory
#if CV_MINOR_VERSION > 3
	#include <opencv2/nonfree/features2d.hpp>
#else
	#include <opencv2/features2d/features2d.hpp>
#endif

#include "matching_strategy.h"

using namespace cv;

#define VERBOSE_MINIMAL 0
#define VERBOSE_NORMAL 1
#define VERBOSE_EXTRA 2

class ObjectMatcher
{
private:
  // The minimum amount of good matches. If the amount of good matches is lower
  // this threshold, the execute method in this class will immediately return false
  int min_good_matches_;
  
  // The minimum inlier ratio for homographies in order
  // to recognize an object
  const static float inlier_ratio_ = 0.4;
  
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
  MatchingStrategy* matching_strategy_;

  int verbose_level;

  // Should we draw the bounding box if it contains crossings?
  // Crossing indicates that a object has not been correctly matched
  bool draw_bounding_box_with_crossings_;
  // result will be put into good_matches by reference
  // void match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches);
	
public:
    struct TrainingImageData
    {
    	std::string label;
    	std::vector<KeyPoint> keypoints;
      Mat descriptors;
    	Mat img;
    };
    
    struct ExecutionResult
    {
    	bool object_recognized; // true if the object has been recognized
      int good_match_count; // The amount of "good" matching keypoints
    	Mat match_image; // an Image produced by cv::drawMatches for debugging purposes
    	string label; // if an object has been recognized, this variable will
                    // hold the label of it, if it has been given in the training step
    };
    
    // Vector of training images
    std::vector<TrainingImageData> training_images_;
    
    ObjectMatcher();
    ObjectMatcher(cv::Ptr<cv::FeatureDetector> detector, cv::Ptr<cv::DescriptorExtractor> extractor);
    
    /**
     * Try to recognize the Object given in train_image in the test_image
     * @param headless If this is set to true, no Windows will be opened to display the matches.
     * @returns ExecutionResult with result=True, if Object has been recognized
     */
    ObjectMatcher::ExecutionResult execute(std::string train_image, std::string test_image, bool headless);
    /**
     * Try to recognize the trained images (trained via ObjectMatcher::trainImages) in a given
     * test image
     *
     * @param headless If this is set to true, no Windows will be opened to display the matches.
     * @returns ExecutionResult with result=True, if Object has been recognized.
     *          ExecutionResult with result=false otherwise (note: This includes scenes, where
     *                          no keypoints couldn't be extracted).
     *          
     */
    ObjectMatcher::ExecutionResult recognizeTrainedImages(std::string test_image, bool headless);
    ObjectMatcher::ExecutionResult recognizeTrainedImages(cv::Mat &test_image, bool headless);
    
    void setMatcher(MatchingStrategy* matching_strategy);
    void setMinGoodMatches(int min);
    void setVerboseLevel(int level);
    void drawBoundingBoxWithCrossings(bool draw);
    
    // Draw the bounding box of a given object in cv::drawMatches image
    // according to a given Homography h
    bool drawBoundingBoxFromHomography(Mat &H, Mat &img_object, Mat &img_matches);
    bool isInlierRatioHighEnough(std::vector<uchar> &outlier_mask, int good_match_count);
    void computeKeyPointsAndDescriptors(Mat &img, std::vector<cv::KeyPoint> &keypoints, Mat &descriptors);
    void calculateHomography(std::vector<cv::KeyPoint> &keypoints_object, std::vector<cv::KeyPoint> &keypoints_scene, std::vector<DMatch> &good_matches, Mat &H, std::vector<uchar> &outlier_mask );
    bool objectRecognized(std::vector< DMatch > &good_matches, std::vector<cv::KeyPoint> &keypoints_object, std::vector<cv::KeyPoint> &keypoints_scene, Mat &H);
    // Train images without a label
    void trainImages(vector<string> file_names);
    // Train images with a label. Both vector sizes must be equal. Otherwise 
    // this method will do nothing.
    void trainImages(vector<string> file_names, vector<string> labels);
    void trainImagesToDatabase(string database_filename, vector<string> training_img_file, vector<string> training_img_label);
    void readTrainImagesFromDatabase(string database_filename);
};

#endif

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
