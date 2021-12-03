#ifndef Lidar_h
#define Lidar_h

#include "Common.h"
#include "dep/cmd_interface_linux.h"
#include "dep/lipkg.h"
#include <array>
#include <memory>

namespace LiDet {
class Lidar {
public:
  Lidar();
  Lidar(const Lidar &aCopy) = delete;
  Lidar &operator=(const Lidar &aCopy) = delete;
  ~Lidar();

  std::array<bool, GRID_SIZE> &getGrid();

private:
  std::unique_ptr<std::array<bool, GRID_SIZE>> myGrid;
  std::unique_ptr<LiPkg> myLidar;
  CmdInterfaceLinux myCmdPort;

  bool initLidar();
};
} // namespace LiDet

#endif
