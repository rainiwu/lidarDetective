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

/** Lidar provides access to onboard LiDAR values
 * Spawns a child thread which continually updates the myData array
 */
class Lidar {
public:
  Lidar();
  Lidar(const Lidar &aCopy) = delete;
  Lidar &operator=(const Lidar &aCopy) = delete;
  ~Lidar();

  const std::array<uint16_t, LIDAR_VALS> &getData();
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
