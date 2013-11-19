#include "matching_strategy.h"
#include "simple_matcher.h"
#include <stdio.h>
#include <iostream>

void SimpleMatcher::match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches)
{
  std::cout << "USING SIMPLE MATCHER" << std::endl;
  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );
  // double matcherElapsedTime = t.elapsed();

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  printf("Max dist between matches: %f \n", max_dist );
  printf("Min dist between matches: %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
  // std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
    { good_matches.push_back( matches[i]); }
  }
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
