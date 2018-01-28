cmake_minimum_required(VERSION 2.8)
project( RobotController )

#set( CMAKE_CXX_COMPILER "g++" )
#set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

set(CMAKE_CXX_FLAGS "-std=c++11")

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )


############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
