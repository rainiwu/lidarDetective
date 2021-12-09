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

const std::array<uint16_t, LIDAR_VALS> &Lidar::getData() { return myData; }

void Lidar::graph(std::ostream &aStream) {
  for (int i = GRAPH_DIST; i > 0; i -= GRAPH_DIST / GRAPH_ROWS) {
    for (int j = 0; j < LIDAR_VALS; j += LIDAR_VALS / GRAPH_COLS) {
      if (myData[j] > i)
        aStream << GRAPH_CHAR;
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
