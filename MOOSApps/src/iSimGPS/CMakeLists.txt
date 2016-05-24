#--------------------------------------------------------
# The CMakeLists.txt for:                       iSimGPS
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  SimGPS.cpp
  SimGPS_Info.cpp
  main.cpp
)

file(GLOB h_files *.h)

ADD_EXECUTABLE(iSimGPS ${SRC})

TARGET_LINK_LIBRARIES(iSimGPS
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(iSimGPS)