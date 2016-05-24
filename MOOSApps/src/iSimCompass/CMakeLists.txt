#--------------------------------------------------------
# The CMakeLists.txt for:                       iSimCompass
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  SimCompass.cpp
  SimCompass_Info.cpp
  main.cpp
)

file(GLOB h_files *.h)



ADD_EXECUTABLE(iSimCompass ${SRC})

TARGET_LINK_LIBRARIES(iSimCompass
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(iSimCompass)