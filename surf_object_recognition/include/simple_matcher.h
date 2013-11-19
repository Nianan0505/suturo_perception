#ifndef SIMPLE_MATCHER_H
#define SIMPLE_MATCHER_H

#include "matching_strategy.h"
#include <stdio.h>
#include <iostream>

/*
 * Wrapper class for basic matching from the OpenCV website
 * e.g. Take the global minimum distance between the feature descriptors
 * and accept everything with global_minimum_distance*x as good match
 */
class SimpleMatcher : public MatchingStrategy
{
public:
	void match(Mat &descriptors_object, Mat &descriptors_scene, std::vector<DMatch> &good_matches);
};

#endif

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
