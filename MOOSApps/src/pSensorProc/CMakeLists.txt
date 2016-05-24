#--------------------------------------------------------
# The CMakeLists.txt for:                       pSensorProc
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  SensorProc.cpp
  SensorProc_Info.cpp
  main.cpp
)

file(GLOB h_files *.h)

ADD_EXECUTABLE(pSensorProc ${SRC})

TARGET_LINK_LIBRARIES(pSensorProc
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread
   lcm)


pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(pSensorProc)