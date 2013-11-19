#ifndef MATCHING_STRATEGY_H
#define MATCHING_STRATEGY_H

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
using namespace cv;

class MatchingStrategy
{
	public:
		virtual void match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches) = 0;
};

#endif
