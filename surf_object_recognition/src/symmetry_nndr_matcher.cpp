#ifndef SYMMETRY_NNDR_MATCHER_CPP
#define SYMMETRY_NNDR_MATCHER_CPP


#include "matching_strategy.h"
#include "nndr_matcher.cpp"
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
      std::vector<cv::DMatch>& symMatches)
  {
    // for all matches image 1 -> image 2
    for (std::vector<std::vector<cv::DMatch> >::
        const_iterator matchIterator1= matches1.begin();
        matchIterator1!= matches1.end(); ++matchIterator1)
    {
      // ignore deleted matches
      if (matchIterator1->size() < 2)
        continue;

      // for all matches image 2 -> image 1
      for (std::vector<std::vector<cv::DMatch> >::
          const_iterator matchIterator2= matches2.begin();
          matchIterator2!= matches2.end();
          ++matchIterator2)
      {
        // ignore deleted matches
        if (matchIterator2->size() < 2)
          continue;

        // Match symmetry test
        if ((*matchIterator1)[0].queryIdx ==
            (*matchIterator2)[0].trainIdx &&
            (*matchIterator2)[0].queryIdx ==
            (*matchIterator1)[0].trainIdx)
        {
          // add symmetrical match
          symMatches.push_back(
              cv::DMatch((*matchIterator1)[0].queryIdx,
                (*matchIterator1)[0].trainIdx,
                (*matchIterator1)[0].distance));
          break; // next match in image 1 -> image 2
        }
      }
    }
  }


	void match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches)
	{
    cv::BruteForceMatcher<cv::L2<float> > matcher;	

    // Match all keypoints from the object to the scene
    std::vector<std::vector<cv::DMatch> > matches1;
    matcher.knnMatch(descriptors_object, descriptors_scene,
        matches1, // Vector of matches
        2); // return 2 nearest neighbours

    // Match all keypoints from the scene with the object
    std::vector<std::vector<cv::DMatch> > matches2;
    matcher.knnMatch(descriptors_scene, descriptors_object,
        matches2, // Vector of matches
        2); // return 2 nearest neighbours


    // Check NNDR for both match vectors
    ratioTest(matches1);
    ratioTest(matches2);

    // Remove non-symmetrical matches
    // std::vector<cv::DMatch> symMatches;
    symmetryTest(matches1,matches2,good_matches);

	}
};

#endif

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
