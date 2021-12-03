#ifndef Lidar_h
#define Lidar_h

#include "dep/cmd_interface_linux.h"
#include "dep/lipkg.h"
#include <memory>
#include <vector>

namespace LiDet {
class Lidar {
public:
  Lidar();
  Lidar(const Lidar &aCopy) = delete;
  Lidar &operator=(const Lidar &aCopy) = delete;
  ~Lidar();

  std::vector<bool> &getGrid();

private:
  std::unique_ptr<std::vector<bool>> myGrid;
  std::unique_ptr<LiPkg> myLidar;
  CmdInterfaceLinux myCmdPort;

  bool initLidar();
};
} // namespace LiDet

#endif
