cmake_minimum_required(VERSION 2.8)


# Name of the executable and the related source files
add_executable (surf_keypoints surf.cpp)

find_package( OpenCV 2.4 REQUIRED )
find_package( Boost REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( surf_keypoints ${OpenCV_LIBS} )
