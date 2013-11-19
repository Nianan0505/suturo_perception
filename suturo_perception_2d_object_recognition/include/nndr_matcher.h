#ifndef NNDR_MATCHER_H
#define NNDR_MATCHER_H

#include "matching_strategy.h"
#include <opencv2/legacy/legacy.hpp>  // Use legacy BruteForce Matcher for now ... 
                                      // TODO: Change to BFMatcher if it works well
/**
 * This Matching Strategy tries to match keypoints with the following strategy:
 * * For every keypoint in the train image:
 *   * Look up the 2 nearest-Neighbours in the test image
 *   * If distance(neighbour1)/distance(neighbour2) > allowed_ratio : DISCARD Match
 *     This will discard 2 "matches" against feature descriptors, that are relatively equal
 *     Since we can't decide which one is better, we discard both matches
 * This Matching Strategy is mentioned on multiple sources, but the main source
 * for my work was:
 *   Laganière, Robert. OpenCV 2 computer vision application programming cookbook. Packt Publishing, 2011. 
 */
class NNDRMatcher : public MatchingStrategy{
public:
  float nndr_ratio_;
  NNDRMatcher()
  {
    nndr_ratio_ = 0.65f; // TODO make parameter
  }

  int ratioTest(std::vector<std::vector<cv::DMatch> > &matches);

	void match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches);

};
#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
