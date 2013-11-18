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

  int ratioTest(std::vector<std::vector<cv::DMatch> > &matches)
  {
    int removed=0;
    // for all matches
    for (std::vector<std::vector<cv::DMatch> >::iterator matchIterator= matches.begin();
        matchIterator!= matches.end();
        ++matchIterator)
    {
      // if 2 NN has been identified
      if (matchIterator->size() > 1)
      {
        // check distance ratio
        if ((*matchIterator)[0].distance/(*matchIterator)[1].distance > nndr_ratio_)
        {
          matchIterator->clear(); // remove match
          removed++;
        }
      } else {
        // does not have 2 neighbours
        matchIterator->clear(); // remove match
        removed++;
      }
    }
    return removed;

  }

	void match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches)
	{
    cv::BruteForceMatcher<cv::L2<float> > matcher;	
    std::vector<std::vector<cv::DMatch> > matches;

    matcher.knnMatch(descriptors_object, descriptors_scene,
        matches, // Vector of matches
        2); // return 2 nearest neighbours
    int removedMatches;
    removedMatches = ratioTest(matches);
    for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator= matches.begin();
                                      matchIterator!= matches.end(); ++matchIterator)
    {
      // ignore deleted matches
      if(matchIterator->size() < 2)
        continue;
      // Push good match into good_matches reference
      good_matches.push_back(cv::DMatch(  (*matchIterator)[0].queryIdx,
                                          (*matchIterator)[0].trainIdx,
                                          (*matchIterator)[0].distance));

    }
	}
};

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
