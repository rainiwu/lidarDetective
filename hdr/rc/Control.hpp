#ifndef Control_hpp
#define Control_hpp

#include "Common.h"
#include "iface/Lidar.hpp"
#include "rc/Control.cuh"
#include "rc/Robot.hpp"
#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>
#include <fstream>

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

  // start/stop controlling
  void start();
  void stop();

protected:
  // I/O classes
  Robot myRobot;
  Lidar myLidar;

  // asynchronous handle
  std::thread myLoop;
  bool stopFlag = false;

  // state weighting table on device
  float *dQtable = nullptr;
  // action on device
  uint8_t *dAction = nullptr;
  // cuda random state
  curandState *dRandState = nullptr;
  // store current state
  uint8_t *dCState = nullptr;
  // store previous state
  uint8_t *dNState = nullptr;
  // device lidar data
  uint16_t *dLaserDat = nullptr;
  // device reward
  float *dReward = nullptr;

  // host action
  uint8_t *hAction = nullptr;
  // host lidar data
  uint16_t *hLaserDat = nullptr;

  // main control loop
  void control();

  // qtable utilities
  void loadQtable();
  void saveQtable();
};
} // namespace LiDet
#endif
