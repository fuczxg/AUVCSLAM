#--------------------------------------------------------
# The CMakeLists.txt for:                       pAcommsScheduler
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  AcommsScheduler.cpp
  AcommsScheduler_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pAcommsScheduler ${SRC})

TARGET_LINK_LIBRARIES(pAcommsScheduler
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

pods_install_executables(pAcommsScheduler)