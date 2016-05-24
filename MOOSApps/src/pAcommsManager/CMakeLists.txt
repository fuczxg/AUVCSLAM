#--------------------------------------------------------
# The CMakeLists.txt for:                       pAcommsManager
# Author(s):                              
#--------------------------------------------------------

#=============================================================================                        
# Find the hovergroup trunk                                                                           
#=============================================================================                        

find_path( HOVERGROUP_TRUNK_BASE
                        NAMES libraries
                        PATHS "~/hover/code"
                        NO_DEFAULT_PATH
)

FILE(GLOB HOVERGROUP_LIB_DIRECTORIES
    ${HOVERGROUP_TRUNK_BASE}/libraries/src/lib_*
    )

set ( HOVERGROUP_INCLUDE_DIRECTORIES
                ${HOVERGROUP_LIB_DIRECTORIES}
     )

set ( HOVERGROUP_LIBRARY_PATH
                ${HOVERGROUP_TRUNK_BASE}/libraries/lib )



LINK_DIRECTORIES(${HOVERGROUP_LIBRARY_PATH})
INCLUDE_DIRECTORIES(${HOVERGROUP_INCLUDE_DIRECTORIES})

message("${HOVERGROUP_TRUNK_BASE}")

SET(SRC
  AcommsManager.cpp
  AcommsManager_Info.cpp
  main.cpp
)


ADD_EXECUTABLE(pAcommsManager ${SRC})


target_link_libraries(pAcommsManager
   ${MOOS_LIBRARIES}
   geometry
   mbutil
   m
   pthread
   messages
   dccl
   lcm
   boost_system
   HoverAcomms
   )

pods_install_executables(pAcommsManager)