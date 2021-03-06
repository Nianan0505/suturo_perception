cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_ros_utils)

# WORKAROUND - import pcl here as well 
find_package(PCL 1.6 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl
  pcl_ros
  rosbag
  cv_bridge
  image_transport
  suturo_perception_utils
)
# WORKAROUND - import pcl here as well 
find_package(PCL 1.6 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system program_options thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_ros_utils
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs suturo_perception_utils
  DEPENDS opencv
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(suturo_perception_ros_utils
  src/publisher_helper.cpp
)

add_library(suturo_perception_ros_utils_basic_io_pcd
  src/basic_io_pcd.cpp
)

## Declare a cpp executable
add_executable(remove_nans 
  src/remove_nans.cpp
  src/basic_io_pcd.cpp)

add_executable(smooth_surface 
  src/smooth_surface.cpp
  src/basic_io_pcd.cpp)

add_executable(compute_bounding_box 
  src/compute_bounding_box.cpp
  src/basic_io_pcd.cpp)

add_executable(compute_convex_hull 
  src/compute_convex_hull.cpp
  src/basic_io_pcd.cpp)

add_executable(get_min_max_3d 
  src/get_min_max_3d.cpp
  src/basic_io_pcd.cpp)

add_executable(downsample_cloud
  src/downsample_cloud.cpp
  src/basic_io_pcd.cpp)

add_executable(segment_plane_and_show
  src/segment_plane_and_show.cpp
  src/basic_io_pcd.cpp)

add_executable(bag_extraction
  src/bag_extraction.cpp
)

add_executable(ply_to_pcd
  src/ply_to_pcd.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(suturo_perception_utils_node suturo_perception_utils_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(suturo_perception_ros_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(suturo_perception_ros_utils_basic_io_pcd
  ${catkin_LIBRARIES}
)

target_link_libraries(remove_nans
  ${catkin_LIBRARIES}
)

target_link_libraries(smooth_surface
  ${catkin_LIBRARIES}
)

target_link_libraries(compute_bounding_box
  ${catkin_LIBRARIES}
)

target_link_libraries(compute_convex_hull
  ${catkin_LIBRARIES}
)

target_link_libraries(get_min_max_3d
  ${catkin_LIBRARIES}
)

target_link_libraries(downsample_cloud
  ${catkin_LIBRARIES}
)

target_link_libraries(segment_plane_and_show
  ${catkin_LIBRARIES}
)

target_link_libraries(ply_to_pcd
  ${catkin_LIBRARIES}
)

target_link_libraries(bag_extraction
  ${catkin_LIBRARIES}
  ${BOOST_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS suturo_perception_utils suturo_perception_utils_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_suturo_perception_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

catkin_add_gtest(${PROJECT_NAME}-test 
  test/test.cpp
)
if(TARGET ${PROJECT_NAME}-test)
	# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} suturo_perception_shape_detection)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
	# find_package(PCL 1.6 REQUIRED COMPONENTS)
	# include_directories(${PCL_INCLUDE_DIRS})
	# link_directories(${PCL_LIBRARY_DIRS})
	# add_definitions(${PCL_DEFINITIONS})
	# Link against catkin libs for ROS etc.

	# In contrast to the tests in suturo_perception_lib, i need this declaration ..
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()
