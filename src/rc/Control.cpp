#include "rc/Control.hpp"
#include <cuda_runtime.h>
#include <exception>

namespace LiDet {

Control::Control() {
  // initvals(dQtable);
  size_t stateSize = pow(NUM_STATES, NUM_REGIONS);
  size_t qtabSize = stateSize * NUM_ACTION;

  cudaMalloc((void **)&dQtable, sizeof(float) * qtabSize);
  cudaMalloc((void **)&dAction, sizeof(uint8_t));
  cudaMalloc((void **)&dRandState, sizeof(curandState));
  cudaMalloc((void **)&dReward, sizeof(float) * NUM_REGIONS);

  cudaMalloc((void **)&dCState, sizeof(uint8_t) * stateSize);
  cudaMalloc((void **)&dNState, sizeof(uint8_t) * stateSize);

  cudaMalloc((void **)&dLaserDat, sizeof(uint16_t) * LIDAR_VALS);

  hAction = new uint8_t;
  hLaserDat = (uint16_t *)malloc(sizeof(uint16_t) * LIDAR_VALS);

  initrand(dRandState);

  if (QTAB_LOAD)
    loadQtable();
  else
    initvals(dQtable);
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
  try {
    myLoop.join();
  } catch (std::exception ex) {
    std::cout << "issue with loop join\n";
  }
  cudaDeviceSynchronize();
}

void Control::control() {
  while (false == stopFlag) {
    // get data from myLidar
    myLidar.graph(std::cout);
    cudaMemcpy(dLaserDat, &myLidar.getData()[LIDAR_CENT],
               sizeof(uint16_t) * (LIDAR_VALS / LIDAR_DIV),
               cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    // transform myLidar data into state
    calcState(dLaserDat, dCState);
    // determine action based on QTable, cstate, and policy
    agentAction(dQtable, dCState, dAction, dRandState);
    cudaDeviceSynchronize();
    cudaMemcpy(hAction, dAction, sizeof(uint8_t), cudaMemcpyDeviceToHost);
    // execute action
    // if (*hAction != ROBOT_STRR || *hAction != ROBOT_STRL)
      // myRobot(ROBOT_STRAIGHT);
    myRobot(*hAction);

    // determine next state
    cudaMemcpy(dLaserDat, &myLidar.getData()[LIDAR_CENT],
               sizeof(uint16_t) * (LIDAR_VALS / LIDAR_DIV),
               cudaMemcpyHostToDevice);
    calcState(dLaserDat, dNState);

    // determine reward
    agentReward(dCState, dNState, dReward);
    // update QTable using state
    agentUpdate(dQtable, dCState, dNState, dReward, dAction);
  }
}

void Control::saveQtable() {
  auto dataSize = (size_t)pow(NUM_STATES, NUM_REGIONS) * NUM_ACTION;
  auto data = std::vector<float>(dataSize);
  cudaMemcpy(&data[0], dQtable, sizeof(float) * dataSize,
             cudaMemcpyDeviceToHost);
  // write data to file defined by QTAB_FILE
  std::ofstream outfile;
  outfile.open(QTAB_FILE, std::ofstream::binary);
  outfile.write((const char *)&data[0], dataSize * sizeof(float));
  outfile.close();
}

void Control::loadQtable() {
  auto dataSize = (size_t)pow(NUM_STATES, NUM_REGIONS) * NUM_ACTION;
  auto data = std::vector<float>(dataSize);
  // load data from file defined by QTAB_FILE
  std::ifstream infile;
  infile.open(QTAB_FILE, std::ifstream::binary);
  infile.read((char *)&data[0], dataSize * sizeof(float));

  cudaMemcpy(dQtable, &data[0], sizeof(float) * dataSize,
             cudaMemcpyDeviceToHost);
}

} // namespace LiDet
