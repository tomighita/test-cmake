cmake_minimum_required(VERSION 3.26)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(TSP)

project(online_assesment)
#
## Find SCIP
#find_package(SCIP REQUIRED)
#
## Include the SCIP headers
#include_directories(${SCIP_INCLUDE_DIRS})

add_executable(online_assesment main.cpp)

target_link_libraries(online_assesment sciptsp)
#
## Link your project with the SCIP library
#target_link_libraries(online_assesment ${SCIP_LIBRARIES})