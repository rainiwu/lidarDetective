set(TARGET lidarDetective)
add_executable(${TARGET} main.cpp dep/lipkg.cpp dep/tofbf.cpp dep/cmd_interface_linux.cpp iface/Lidar.cpp)
target_link_libraries(${TARGET} pthread udev)
