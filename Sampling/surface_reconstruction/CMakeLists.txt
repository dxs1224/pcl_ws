cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(surface_reconstruction)

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(surface_reconstruction surface_reconstruction.cpp)

target_link_libraries (surface_reconstruction ${PCL_LIBRARIES} ${Boost_LIBRARIES})