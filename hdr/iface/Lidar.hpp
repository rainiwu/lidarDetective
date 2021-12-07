#ifndef Lidar_h
#define Lidar_h

#include "Common.h"
#include "dep/cmd_interface_linux.h"
#include "dep/lipkg.h"
#include <array>
#include <cstdint>
#include <memory>
#include <thread>

namespace LiDet {

class Lidar {
public:
  Lidar();
  Lidar(const Lidar &aCopy) = delete;
  Lidar &operator=(const Lidar &aCopy) = delete;
  ~Lidar();

  void graph(std::ostream &aStream);

protected:
  std::unique_ptr<LiPkg> myLidar;
  std::array<uint16_t, LIDAR_VALS> myData;
  std::thread lidarLoop;
  CmdInterfaceLinux myCmdPort;

  bool endThread = false;
  bool initLidar();
  void updateData();
};
} // namespace LiDet

#endif
