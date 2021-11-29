#ifndef Drive_h
#define Drive_h

namespace LiDet {
class Drive {
public:
  Drive();
  Drive(const Drive &aCopy);
  Drive &operator=(const Drive &aCopy);
  ~Drive();
};
} // namespace LiDet

#endif
