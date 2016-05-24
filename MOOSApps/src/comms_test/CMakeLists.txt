#--------------------------------------------------------
# The CMakeLists.txt for:                       pAcommsManager
# Author(s):                              
#--------------------------------------------------------

#=============================================================================                        
# Find the hovergroup trunk                                                                           
#=============================================================================                        



SET(SRC
  comms_test.cpp
)


ADD_EXECUTABLE(comms_test ${SRC})


target_link_libraries(comms_test
   messages
   dccl
   protobuf
   )

pods_install_executables(comms_test)