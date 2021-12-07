#include "iface/Lidar.hpp"

namespace LiDet {

Lidar::Lidar() : myCmdPort() {
  myLidar = std::make_unique<LiPkg>();
  initLidar();
  lidarLoop = std::thread(&Lidar::updateData, this);
  lidarLoop.detach();
}

Lidar::~Lidar() {
  endThread = true;
  lidarLoop.join();
  endThread = false;
}

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

  return EXIT_SUCCESS;
}

void Lidar::graph(std::ostream &aStream) {

  for (int i = 10000; i > 0; i -= 250) {
    for (int j = 0; j < 360; j += 3) {
      if (myData[j] > i)
        aStream << "1";
      else
        aStream << " ";
    }
    aStream << '\n';
  }
}

void Lidar::updateData() {
  while (!endThread) {
    if (myLidar->IsPkgReady()) {
      for (auto lidarVal : myLidar->GetPkgData()) {
        if (lidarVal.confidence >= CONF_THRESH)
          myData[(uint16_t)lidarVal.angle] = lidarVal.distance;
      }
      myLidar->ResetFrameReady();
    }
  }
}

} // namespace LiDet
