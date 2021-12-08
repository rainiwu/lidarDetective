#ifndef Robot_hpp
#define Robot_hpp

#include "Common.h"
#include "iface/Drive.hpp"
#include <cstdint>

namespace LiDet {
/** Robot drives the car
 *
 */
class Robot {
public:
  Robot();
  Robot(const Robot &aCopy) = delete;
  Robot &operator=(const Robot &aCopy) = delete;
  ~Robot();

  // perform action
  void operator()(uint8_t action);

protected:
  Drive myDrive;
  float delta_throt = 0.03;
  float delta_steer = 0.03;

  void throtUp();
  void throtDn();
  void steerLf();
  void steerRi();
};
} // namespace LiDet

#endif
