# Compile my_class as library
add_library(suturo_perception_lib suturo_perception.cpp point_cloud_operations.cpp)

# Use the PCL packages for the lib
find_package(PCL 1.6 REQUIRED COMPONENTS geometry_msgs)

# Include PCL directories after discovery of PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(OpenCV REQUIRED)

# include boost
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.48.0 COMPONENTS thread) 

include_directories(${Boost_INCLUDE_DIRS}) 
link_directories(${Boost_LIBRARY_DIRS})
# Link library against PCL and boost
target_link_libraries(suturo_perception_lib suturo_perception_utils ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

