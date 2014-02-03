#!/bin/sh

find . -type f -name '*.pcd' | xargs rosrun vfh_prototype estimate_vfh_features
