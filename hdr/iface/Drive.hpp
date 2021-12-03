#ifndef Drive_h
#define Drive_h

#include "Common.h"

namespace LiDet {
class Drive {
public:
  Drive();
  Drive(const Drive &aCopy);
  Drive &operator=(const Drive &aCopy);
  ~Drive();

  void setThrottle();
  void setSteering();

protected:
  float throttle = DEFAULT_THROTTLE;
  float steering = DEFAULT_STEERING;
};
} // namespace LiDet

#endif
