cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_cloud_fitter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  suturo_perception_utils
  suturo_perception_lib
)
link_directories(${suturo_perception_utils_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_cloud_fitter
  CATKIN_DEPENDS pcl roscpp
  #DEPENDS opencv
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
add_library(suturo_perception_cloud_fitter
 src/mesh_loader.cpp
 )

## Declare a cpp executable
# add_executable(perception_node src/perception_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(perception_node perception_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(perception_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS perception perception_node
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
#catkin_add_gtest(${PROJECT_NAME}-test 
#  test/test.cpp
#  WORKING_DIRECTORY ${POINTCLOUD_PATH}
#)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  
  # other stuff neded for the test
  #configure_file( test/pointcloud_path.h.in ${CMAKE_BINARY_DIR}/generated/pointcloud_path.h )
  #include_directories( ${CMAKE_BINARY_DIR}/generated/ )
  
  # test needs pcl
  find_package(PCL 1.6 REQUIRED COMPONENTS)

  find_package( OpenCV REQUIRED )

  # Include PCL directories after discovery of PCL
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBS})
  add_definitions(${PCL_DEFINITIONS})
  

  # used to execute the sync_pointclouds.sh script
  #add_dependencies(${PROJECT_NAME}-test suturo_pcd_files)
endif()

