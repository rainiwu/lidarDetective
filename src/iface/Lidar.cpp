#include "iface/Lidar.hpp"

namespace LiDet {

Lidar::Lidar() : myCmdPort() {
  myGrid = std::make_unique<std::vector<bool>>(256);
  myLidar = std::make_unique<LiPkg>();
  initLidar();
}

Lidar::~Lidar() {}

bool Lidar::initLidar() {

  std::vector<std::pair<std::string, std::string>> device_list;
  std::string port_name;
  myCmdPort.GetCmdDevices(device_list);
  for (auto n : device_list) {
    std::cout << n.first << "    " << n.second << std::endl;
    if (strstr(n.second.c_str(), "CP2102")) {
      port_name = n.first;
    }
  }

  if (port_name.empty()) {
    std::cout << "Can't find LiDAR LD06" << std::endl;
  }

  std::cout << "FOUND LiDAR_LD06" << std::endl;
  myCmdPort.SetReadCallback([this](const char *byte, size_t len) {
    if (myLidar->Parse((uint8_t *)byte, len)) {
      myLidar->AssemblePacket();
    }
  });

  if (myCmdPort.Open(port_name))
    std::cout << "LiDAR_LD06 started successfully " << std::endl;

  return 1;
}

std::vector<bool> &Lidar::getGrid() { return *myGrid; }

} // namespace LiDet
