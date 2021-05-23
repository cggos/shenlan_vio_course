cmake_minimum_required(VERSION 3.10)
project(Triangulate)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(estimate_depth triangulate.cpp)
#target_link_libraries(estimate_depth  ${Sophus_LIBRARIES})
