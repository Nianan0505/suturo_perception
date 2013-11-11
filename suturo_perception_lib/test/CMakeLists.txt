cmake_minimum_required(VERSION 2.8)

# Include header files from lib directory
include_directories(
  ${catkin_INCLUDE_DIRS}
  ../lib
)
message("Hallo Welt...")

# Define all source files for executable
set(
    test.cpp
)

# Thanks to 
# http://www.thebigblob.com/getting-started-with-google-test-on-ubuntu/
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Name of the executable and the related source files
add_executable (test_bin test.cpp)

# Link the test program against the lib, the gtest framework and pthread
target_link_libraries(test_bin suturo_perception_lib ${GTEST_LIBRARIES} pthread)
