#include "matching_strategy.h"
#include "nndr_matcher.h"
#include <opencv2/legacy/legacy.hpp>  // Use legacy BruteForce Matcher for now ... 
int NNDRMatcher::ratioTest(std::vector<std::vector<cv::DMatch> > &matches)
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

void NNDRMatcher::match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches)
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

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
