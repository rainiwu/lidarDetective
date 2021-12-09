#include "rc/Control.hpp"
namespace LiDet {

Control::Control() {
  size_t stateSize = (NUM_STATES ^ NUM_REGIONS);
  size_t qtabSize = stateSize * NUM_ACTION;

  cudaMalloc((void **)&dQtable, sizeof(float) * qtabSize);
  cudaMalloc((void **)&dAction, sizeof(uint8_t));
  cudaMalloc((void **)&dRandState, sizeof(curandState));
  cudaMalloc((void **)&dReward, sizeof(float));

  cudaMalloc((void **)&dCState, sizeof(uint8_t) * stateSize);
  cudaMalloc((void **)&dNState, sizeof(uint8_t) * stateSize);

  cudaMalloc((void **)&dLaserDat, sizeof(uint16_t) * LIDAR_VALS);

  hAction = new uint8_t;
  hLaserDat = (uint16_t *)malloc(sizeof(uint16_t) * LIDAR_VALS);

  if (QTAB_LOAD)
    loadQtable();
  // else
  // clearQtable();
}

Control::~Control() {
  if (QTAB_SAVE)
    saveQtable();

  cudaFree(dQtable);
  cudaFree(dAction);
  cudaFree(dRandState);
  cudaFree(dCState);
  cudaFree(dNState);
  cudaFree(dLaserDat);
  cudaFree(dReward);

  delete hAction;
  free(hLaserDat);
}

void Control::start() {
  stopFlag = false;
  myLoop = std::thread(&Control::control, this);
  myLoop.detach();
}

void Control::stop() {
  stopFlag = true;
  myLoop.join();
}

void Control::control() {
  while (false == stopFlag) {
    // get data from myLidar
    cudaMemcpy(dLaserDat, &myLidar.getData()[0], sizeof(uint16_t) * LIDAR_VALS,
               cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    // transform myLidar data into state
    calcState(dLaserDat, dCState);
    // determine action based on QTable, cstate, and policy
    agentAction(dQtable, dCState, dAction);
    cudaDeviceSynchronize();
    cudaMemcpy(hAction, dAction, sizeof(uint8_t), cudaMemcpyDeviceToHost);
    // execute action
    myRobot(*hAction);

    // determine next state
    cudaMemcpy(dLaserDat, &myLidar.getData()[0], sizeof(uint16_t) * LIDAR_VALS,
               cudaMemcpyHostToDevice);
    calcState(dLaserDat, dNState);

    // determine reward
    agentReward(dCState, dNState, dReward);
    // update QTable using state
    agentUpdate(dQtable, dCState, dNState, dReward, dAction);
  }
}

} // namespace LiDet
