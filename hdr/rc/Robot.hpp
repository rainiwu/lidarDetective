#ifndef Robot_hpp
#define Robot_hpp

#include "Common.h"
#include "iface/Drive.hpp"
#include <cstdint>
#include <thread>

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
  float delta_throt = THR_DELT;
  float delta_steer = STR_DELT;

  void throtUp();
  void throtDn();
  void steerLf();
  void steerRi();
  void throtStop();
  void steerStraight();
};
} // namespace LiDet

#endif
