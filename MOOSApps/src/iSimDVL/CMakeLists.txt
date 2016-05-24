#--------------------------------------------------------
# The CMakeLists.txt for:                       iSimDVL
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  SimDVL.cpp
  SimDVL_Info.cpp
  main.cpp
)

file(GLOB h_files *.h)

ADD_EXECUTABLE(iSimDVL ${SRC})

TARGET_LINK_LIBRARIES(iSimDVL
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(iSimDVL)