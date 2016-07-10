#--------------------------------------------------------
# The CMakeLists.txt for:                       pVRP
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  VRP.cpp
  VRP_Info.cpp
  main.cpp
)


file (GLOB h_files *.h)

ADD_EXECUTABLE(pVRP ${SRC})

TARGET_LINK_LIBRARIES(pVRP
   geometry
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(pVRP)