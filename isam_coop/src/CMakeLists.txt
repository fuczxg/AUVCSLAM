set(MAKE_BUILD_TYPE DEBUG)
add_definitions("-std=c++11")
add_definitions("-g")

FILE(GLOB SRC 
*.cpp)

file(GLOB h_files *.h)

include_directories(.)

add_executable(${POD_NAME} ${SRC})
target_link_libraries(${POD_NAME}
isam
cholmod
boost_system
)

pods_use_pkg_config_packages(${POD_NAME} lcm lcmtypes_${POD_NAME} visualization)

# make exeutables public
pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(${POD_NAME})