cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)
project(readLAS2PCD)

find_package(PCL 1.7 REQUIRED)
find_package(libLAS REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (readLAS2PCD main.cpp convertlas.h)
target_link_libraries (readLAS2PCD ${PCL_LIBRARIES} ${libLAS_LIBRARIES})
