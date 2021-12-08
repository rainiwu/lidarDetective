#ifndef Control_hpp
#define Control_hpp

#include "Common.h"
#include "iface/Lidar.hpp"
#include "rc/Robot.hpp"
#include <curand.h>
#include <curand_kernel.h>

namespace LiDet {

/** Control runs a reinforcement learning algorithm for Robot navigation
 *
 */
class Control {
public:
  Control();
  Control(const Control &aCopy);
  Control &operator=(const Control &aCopy);
  ~Control();

  void start();

protected:
  // I/O classes
  Robot myRobot;
  Lidar myLidar;

  // state weighting table on device
  float *dQtable = nullptr;
  // cuda random state
  curandState *dRandState = nullptr;

  void agentUpdate();
  void agentAct();

  void loadQtable();
  void saveQtable();
};
} // namespace LiDet
#endif
