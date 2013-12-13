cmake_minimum_required(VERSION 2.8)
project(boost_threadpool_test)

add_executable (boost_threadpool_test threadpool.cpp)

#find_package(Boost REQUIRED)

# include boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package( Boost COMPONENTS thread system REQUIRED )
# find_package(Boost 1.48.0 COMPONENTS thread) 

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries( boost_threadpool_test ${Boost_LIBRARIES} )
