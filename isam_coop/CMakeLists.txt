cmake_minimum_required(VERSION 2.6.0)
# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME isam_coop)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
set(CMAKE_MODULE_PATH /usr/local/lib/pkgconfig)
include(cmake/lcmtypes.cmake)
lcmtypes_build()

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

find_package(PkgConfig REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


pkg_check_modules(VIS visualization)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_subdirectory(src)
