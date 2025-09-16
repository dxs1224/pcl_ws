cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(cloud_viewer)

# 查找必要的包
find_package(PCL 1.2 REQUIRED COMPONENTS common io visualization filters sample_consensus)
find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(random_sample_consensus random_sample_consensus.cpp)
target_link_libraries(random_sample_consensus 
    ${PCL_LIBRARIES} 
    ${Boost_THREAD_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY}
    pthread
)