cmake_minimum_required( VERSION 2.8 )
project( eigen_rotation )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

include_directories( "/usr/include/eigen3" )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( eigen_rotation eigen_rotation.cpp)
target_link_libraries( eigen_rotation ${Sophus_LIBRARIES} )
