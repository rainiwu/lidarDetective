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
  Robot(const Robot &aCopy);
  Robot &operator=(const Robot &aCopy);
  ~Robot();

  // perform action
  void operator()(uint8_t action);

protected:
  Drive myDrive();
  uint8_t delta_throt = 10;
  uint8_t delta_steer = 10;

  void throtUp();
  void throtDn();
  void steerLf();
  void steerRi();
};
} // namespace LiDet

#endif
