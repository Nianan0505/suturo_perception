cmake_minimum_required(VERSION 2.8)

configure_file( pointcloud_path.h.in ${CMAKE_BINARY_DIR}/generated/pointcloud_path.h )
include_directories( ${CMAKE_BINARY_DIR}/generated/ )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/generated/
  ../lib
)

# Define all source files for executable
set(
    test.cpp
)

# Thanks to 
# http://www.thebigblob.com/getting-started-with-google-test-on-ubuntu/
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Use the PCL packages for the test
find_package(PCL 1.6 REQUIRED COMPONENTS)

# Include PCL directories after discovery of PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Name of the executable and the related source files
add_executable (test_bin test.cpp)

# Link the test program against the lib, the gtest framework and pthread
target_link_libraries(test_bin suturo_perception_lib ${GTEST_LIBRARIES} ${PCL_LIBRARIES} pthread)
