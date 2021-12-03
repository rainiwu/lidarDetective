#include "dep/cmd_interface_linux.h"
#include "dep/lipkg.h"
#include "dep/tofbf.h"
#include <iostream>
#include <stdio.h>
#include <string>

int main(int argc, char **argv) {

  LiPkg *lidar = new LiPkg;

  CmdInterfaceLinux cmd_port;
  std::vector<std::pair<std::string, std::string>> device_list;
  std::string port_name;
  cmd_port.GetCmdDevices(device_list);
  for (auto n : device_list) {
    std::cout << n.first << "    " << n.second << std::endl;
    if (strstr(n.second.c_str(), "CP2102")) {
      port_name = n.first;
    }
  }

  if (port_name.empty()) {
    std::cout << "Can't find LiDAR LD06" << std::endl;
    return 1;
  }

  std::cout << "FOUND LiDAR_LD06" << std::endl;
  cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
    if (lidar->Parse((uint8_t *)byte, len)) {
      // std::cout << lidar->GetPkgData()[0].angle << std::endl;
    }
  });

  if (cmd_port.Open(port_name))
    std::cout << "LiDAR_LD06 started successfully " << std::endl;

  while (true) {
    if (lidar->IsPkgReady()) {
      if (lidar->GetPkgData()[0].angle <= 1.0) {
        std::cout << lidar->GetPkgData()[0].distance
                  << "                      \t\r" << std::flush;
      }
      lidar->ResetFrameReady();
    }
  }

  return 0;
}
