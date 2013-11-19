#ifndef SYMMETRY_NNDR_MATCHER_H
#define SYMMETRY_NNDR_MATCHER_H


#include "matching_strategy.h"
#include "nndr_matcher.h"
#include <opencv2/legacy/legacy.hpp>  // Use legacy BruteForce Matcher for now ... 
                                      // TODO: Change to BFMatcher if it works well
/**
 * This Matching Strategy is based on the NNDRMatcher, but uses
 * symmetric matching. It matches the descriptors from the
 * object against the descripts of the scene, and vice versa
 * Only symmetric matches will be kept
 *
 * This matcher is also based on:
 *   Laganière, Robert. OpenCV 2 computer vision application programming cookbook. Packt Publishing, 2011. 
 */
class SymmetryNNDRMatcher : public NNDRMatcher{
public:

  // Insert symmetrical matches in symMatches vector
  void symmetryTest(
      const std::vector<std::vector<cv::DMatch> >& matches1,
      const std::vector<std::vector<cv::DMatch> >& matches2,
      std::vector<cv::DMatch>& symMatches);


	void match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches);
};

#endif

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
