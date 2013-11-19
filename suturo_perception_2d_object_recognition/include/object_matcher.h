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
	// result will be put into good_matches by reference
	// void match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches);
	
public:
		struct ExecutionResult
		{
			bool object_recognized; // true if the object has been recognized
			Mat match_image; // an Image produced by cv::drawMatches for debugging purposes
		};

		ObjectMatcher();
		ObjectMatcher(cv::Ptr<cv::FeatureDetector> detector, cv::Ptr<cv::DescriptorExtractor> extractor);
		/**
		 * Try to recognize the Object given in train_image in the test_image
		 * @param headless If this is set to true, no Windows will be opened to display the matches.
		 * @returns True, if Object has been recognized
		 */
		ObjectMatcher::ExecutionResult execute(std::string train_image, std::string test_image, bool headless);
		void setMatcher(MatchingStrategy* matching_strategy);
		void setMinGoodMatches(int min);

		// Draw the bounding box of a given object in cv::drawMatches image
		// according to a given Homography h
		void drawBoundingBoxFromHomography(Mat &H, Mat &img_object, Mat &img_matches);
		bool isInlierRatioHighEnough(std::vector<uchar> &outlier_mask, int good_match_count);
};

#endif

