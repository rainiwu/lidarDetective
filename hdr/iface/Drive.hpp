#ifndef Drive_h
#define Drive_h

#include "Common.h"
#include "dep/JHPWMPCA9685.h"
#include <iostream>

namespace LiDet {
class Drive {
public:
  Drive();
  Drive(const Drive &aCopy);
  Drive &operator=(const Drive &aCopy);
  ~Drive();

  void setThrottle(const float &aValue);
  void setSteering(const float &aValue);

  const float &getThrottle();
  const float &getSteering();

protected:
  float throttle = THROTTLE_NEUTRAL;
  float steering = STEERING_NEUTRAL;
  PCA9685 *pca9685;
};
} // namespace LiDet

#endif
