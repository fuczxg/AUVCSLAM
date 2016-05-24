#--------------------------------------------------------
# The CMakeLists.txt for:                       pSimFeatureDetector
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  SimFeatureDetector.cpp
  SimFeatureDetector_Info.cpp
  main.cpp
)

file(GLOB h_files *.h)

ADD_EXECUTABLE(pSimFeatureDetector ${SRC})

TARGET_LINK_LIBRARIES(pSimFeatureDetector
   ${MOOS_LIBRARIES}
   mbutil
   m
   lcm
   pthread)

pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(pSimFeatureDetector)