#ifndef Detector_h
#define Detector_h

namespace LiDet {
class Detector {
public:
  Detector();
  Detector(const Detector &aCopy);
  Detector &operator=(const Detector &aCopy);
  ~Detector();

private:
};
} // namespace LiDet

#endif
