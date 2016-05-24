cmake_minimum_required(VERSION 2.6.0)
set(MAKE_BUILD_TYPE DEBUG)
add_definitions("-std=c++11")
add_definitions("-g")
# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME MOOSApps)
include(cmake/pods.cmake)


SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})


find_package(MOOSIvP REQUIRED)
find_package(Goby REQUIRED)
include_directories(${GOBY_INCLUDE_DIRS})
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
message("protobuf include dirs: " ${PROTOBUF_INCLUDE_DIRS})



include_directories("${PROJECT_SOURCE_DIR}/include") 
include_directories("${PROJECT_SOURCE_DIR}/pod-build") # only way I have found to get the protobufs to include properly

file(GLOB h_files include/*.h)  # I have a few useful header files shared by all apps 
pods_install_headers(${h_files} DESTINATION ${POD_NAME})

add_subdirectory(protobufs)
add_subdirectory(src/pSensorProc)
add_subdirectory(src/pSimFeatureDetector)
add_subdirectory(src/iSimCompass)
add_subdirectory(src/iSimGPS)
add_subdirectory(src/iSimDVL)
add_subdirectory(src/iSimDepth)
add_subdirectory(src/pAcommsManager)
#add_subdirectory(src/comms_test)
add_subdirectory(src/pAcommsScheduler)