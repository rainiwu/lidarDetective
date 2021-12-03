#ifndef Lidar_h
#define Lidar_h

#include "Common.h"
#include "dep/cmd_interface_linux.h"
#include "dep/lipkg.h"
#include <array>
#include <memory>

namespace LiDet {

struct LiGrid : public std::array<bool, GRID_SIZE> {
  const bool &getValue(const size_t &x, const size_t &y) {
    return (*this)[x + y * GRID_SIZE];
  }
  inline void setValue(const bool &aValue, const size_t &x, const size_t &y) {
    (*this)[x + y * GRID_SIZE] = aValue;
  }
};

class Lidar {
public:
  Lidar();
  Lidar(const Lidar &aCopy) = delete;
  Lidar &operator=(const Lidar &aCopy) = delete;
  ~Lidar();

  const LiGrid &getGrid();

private:
  std::unique_ptr<LiGrid> myGrid;
  std::unique_ptr<LiPkg> myLidar;
  CmdInterfaceLinux myCmdPort;

  bool initLidar();
};
} // namespace LiDet

#endif
