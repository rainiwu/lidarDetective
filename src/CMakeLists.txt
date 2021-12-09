set(TARGET lidarDetective)
add_executable(${TARGET} main.cpp dep/lipkg.cpp dep/tofbf.cpp dep/cmd_interface_linux.cpp dep/JHPWMPCA9685.cpp iface/Lidar.cpp
  iface/Drive.cpp rc/Robot.cpp rc/Control.cpp rc/Control.cu)
target_link_libraries(${TARGET} pthread udev i2c)
