#include "rc/Control.cuh"
#include <cstdlib>

__global__ void findState(uint16_t *laserDat, uint8_t *states) {
  // parallelized by region - tid is region num
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  int sum = 0;
  int offset = 0;
  int avg, tmp;
  for (size_t i = 0; i < LIDAR_VALS / NUM_REGIONS; i++) {
    // iterate through region
    tmp = laserDat[tid * NUM_REGIONS + i];
    // if bad value, discard
    if (0 == tmp || LIDAR_MAX_V < tmp) {
      tmp = 0;
      offset++;
    }
    sum += tmp;
  }
  // avg of region is sum / numvals
  avg = sum / ((LIDAR_VALS / NUM_REGIONS) - offset);
  int stateSize = LIDAR_MAX_V / NUM_STATES;
  // assumes target is exactly half of max v
  for (int i = 0; i < NUM_STATES; i++) {
    if (avg < stateSize * i) {
      states[tid] = i + 1;
      return;
    }
  }
}

void calcState(uint16_t *laserDat, uint8_t *states) {
  findState<<<1, NUM_REGIONS>>>(laserDat, states);
}

void agentUpdate(float *qtable, uint8_t *cstate, uint8_t *nstate, float *reward,
                 uint8_t *action) {}

void agentAction(float *qtable, uint8_t *cstate, uint8_t *action) {}

void agentReward(uint8_t *cstate, uint8_t *nstate, float *reward) {
    for (int i = 0; i < NUM_REGIONS; i++) {
        if (abs(nstate[i] - CTR_STATE) != abs(cstate[i] - CTR_STATE)) { //got either closer or farther to center state
            reward[i] = 1.0 + log(abs(nstate[i] - cstate[i])); //minimum of 1 reward, diminishing returns after
            if (abs(nstate[i] - CTR_STATE) > abs(cstate[i] - CTR_STATE)) { //nextstate got farther from center
                reward[i] *= -1.0;
            }
        } else { //state maintained
            reward = 0.0;
        }
    }
}

__global__ void initQtable(float *qtable) {
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  qtable[tid] = 0;
}
void initvals(float *qtable) {
  size_t qtabSize = (NUM_STATES ^ NUM_REGIONS) * NUM_ACTION;
  initQtable<<<qtabSize / 64, 64>>>();
}
