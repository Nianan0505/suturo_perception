cmake_minimum_required(VERSION 2.8)


# Name of the executable and the related source files
add_executable (surf_keypoints surf.cpp object_matcher.cpp simple_matcher.cpp)

find_package( OpenCV 2.4 REQUIRED )
# include boost
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package( Boost COMPONENTS program_options thread REQUIRED )
# find_package(Boost 1.48.0 COMPONENTS thread) 

include_directories ("${PROJECT_SOURCE_DIR}/include")
include_directories(${Boost_INCLUDE_DIRS}) 
link_directories(${Boost_LIBRARY_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( surf_keypoints ${OpenCV_LIBS} ${Boost_LIBRARIES} )
set(CMAKE_BUILD_TYPE Debug)
