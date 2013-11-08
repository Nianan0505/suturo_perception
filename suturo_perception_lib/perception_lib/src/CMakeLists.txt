cmake_minimum_required(VERSION 2.8)

# Include header files from lib directory
include_directories ("${PROJECT_SOURCE_DIR}/lib")

# Define all source files for executable
set(src_files
    main.cpp
)

# Name of the executable and the related source files
add_executable (main_bin ${src_files})

# Use the PCL packages for the lib
find_package(PCL 1.6 REQUIRED COMPONENTS common io)

# Include PCL directories after discovery of PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Link the main program against the lib
target_link_libraries(main_bin MyClass ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
