cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(uniform_sampling)

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(uniform_sampling uniform_sampling.cpp)

target_link_libraries (uniform_sampling ${PCL_LIBRARIES} ${Boost_LIBRARIES})