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

  void setThrottle(const float &aValue);
  void setSteering(const float &aValue);

  const float &getThrottle();
  const float &getSteering();

protected:
  float throttle = DEFAULT_THROTTLE;
  float steering = DEFAULT_STEERING;
};
} // namespace LiDet

#endif
