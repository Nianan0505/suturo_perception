cmake_minimum_required(VERSION 2.8)

# Define all source files for executable
set(src_files
    convert_pcd_to_rgb.cpp
)

# Use the PCL packages for the lib
# TO FIX: Use older PCL if we don't use PCL 1.7
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED )

# Include OpenCV Library
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Include PCL directories after discovery of PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Name of the executable and the related source files
add_executable (convert_pcd_to_rgb ${src_files})

# Link program against PCL and OpenCV
target_link_libraries(convert_pcd_to_rgb ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS})
