cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_rosnode)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
suturo_perception_lib
suturo_perception_msgs
visualization_msgs
roscpp
dynamic_reconfigure
message_generation
cv_bridge
image_transport
roscpp
std_msgs
suturo_perception_2d_capabilities
suturo_perception_3d_capabilities
suturo_perception_2d_object_recognition_lib
suturo_perception_color_analysis
suturo_perception_shape_detection
suturo_perception_ros_utils 
suturo_perception_utils 
suturo_perception_vfh_estimation
suturo_perception_svm_classification
suturo_perception_match_cuboid
pcl
pcl_ros
)

find_package(OpenCV REQUIRED)

#find_package(catkin REQUIRED dynamic_reconfigure)
# generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/SuturoPerception.cfg
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system program_options thread)

# WORKAROUND - import pcl here as well 
find_package(PCL 1.6 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


link_directories(${suturo_perception_lib_LIBRARY_DIRS})
link_directories(${suturo_perception_ros_utils_LIBRARY_DIRS})
link_directories(${suturo_perception_utils_LIBRARY_DIRS})

# add_dependencies(suturo_perception_rosnode ${PROJECT_NAME}_gencfg)

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
#   std_msgs  # Or other packages containing msgs
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

#catkin_package(DEPENDS roscpp CATKIN_DEPENDS suturo_perception_lib)
#catkin_package( ## at 25.11.2013 -- 11:00
#  INCLUDE_DIRS include
#  LIBRARIES suturo_perception_rosnode
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
  DEPENDS opencv
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
message(========= INCLUDE DIRS "${catkin_INCLUDE_DIRS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)
# workaround: catkin_include doesn't work
#include_directories(${suturo_perception_lib_INCLUDE_DIRS})
#include_directories(${suturo_perception_msgs_INCLUDE_DIRS})


## Declare a cpp library
# add_library(suturo_perception_rosnode
#   src/${PROJECT_NAME}/suturo_perception_rosnode.cpp
# )

## Declare a cpp executable
add_executable(segment_objects src/segment_objects.cpp)
add_executable(pose_estimator src/pose_estimator.cpp)
add_executable(suturo_perception_rosnode src/main.cpp src/suturo_perception_rosnode.cpp src/visualization_publisher.cpp)
add_executable(suturo_perception_dummynode src/dummy_node.cpp)
add_executable(suturo_perception_rosclient src/client.cpp)
add_executable(knowledge_gen src/knowledge_gen.cpp)
add_executable(calc_pc_from_img_and_depth src/calc_pc_from_img_and_depth.cpp)
add_executable(knowledge_gen src/knowledge_gen.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(suturo_perception_rosnode suturo_perception_lib suturo_perception_msgs ${PROJECT_NAME}_gencfg)
add_dependencies(suturo_perception_dummynode suturo_perception_lib suturo_perception_msgs ${PROJECT_NAME}_gencfg)
add_dependencies(suturo_perception_rosclient suturo_perception_lib suturo_perception_msgs)
add_dependencies(knowledge_gen suturo_perception_lib suturo_perception_msgs)

## Specify libraries to link a library or executable target against
target_link_libraries(suturo_perception_rosnode
 ${OpenCV_LIBS} ${catkin_LIBRARIES}
)

target_link_libraries(suturo_perception_dummynode
  ${catkin_LIBRARIES}
)

target_link_libraries(suturo_perception_rosclient
  ${catkin_LIBRARIES}
)

target_link_libraries(knowledge_gen
  ${catkin_LIBRARIES}
)

target_link_libraries(segment_objects
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

target_link_libraries(segment_objects
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

target_link_libraries(calc_pc_from_img_and_depth
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)
target_link_libraries(pose_estimator
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

target_link_libraries(calc_pc_from_img_and_depth
  ${catkin_LIBRARIES}
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
# install(TARGETS suturo_perception_rosnode suturo_perception_rosnode_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_suturo_perception_rosnode.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
