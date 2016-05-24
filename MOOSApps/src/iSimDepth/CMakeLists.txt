#--------------------------------------------------------
# The CMakeLists.txt for:                       iSimDepth
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  SimDepth.cpp
  SimDepth_Info.cpp
  main.cpp
)

file(GLOB h_files *.h)

ADD_EXECUTABLE(iSimDepth ${SRC})

TARGET_LINK_LIBRARIES(iSimDepth
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(iSimDepth)