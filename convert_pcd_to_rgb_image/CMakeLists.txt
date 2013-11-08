project(convert_pcd_to_rgb_image) # project name
cmake_minimum_required(VERSION 2.8)

# Required packages
find_package(Boost REQUIRED)

# Search the following folders for additional CMakeLists.txt
add_subdirectory(src)
