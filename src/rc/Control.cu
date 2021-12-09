#include "rc/Control.cuh"

__device__ int qtableAccessor(uint8_t *state) {
  int qtableIndex = 0;
  for (int i = 0; i < NUM_REGIONS; i++)
    qtableIndex += state[i] * (NUM_STATES ^ i);
  return qtableIndex * 4;
}

__device__ int myMax(int a, int b) { return (a > b) ? a : b; }

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

__global__ void deviceUpdate(float *qtable, uint8_t *cstate, uint8_t *nstate,
                             float *reward, uint8_t *action) {
  short maxA = 0;
  float nMaxVal = qtable[qtableAccessor(nstate) + maxA];
  for (int i = 0; i < NUM_ACTION; i++) {
    if (qtable[qtableAccessor(nstate) + i] > nMaxVal) {
      maxA = i;
      nMaxVal = qtable[qtableAccessor(nstate) + i];
    }
  }
  // qtable CState index
  int qtCIdx = qtableAccessor(cstate) + *action;
  // qtable maxA NState index
  int qtNIdx = qtableAccessor(nstate) + maxA;

  if (0 == *reward)
    qtable[qtCIdx] += ((*reward + DISC_FACT * qtable[qtNIdx] - qtable[qtCIdx]) /
                       LEARN_RATE_DIV);
  else
    qtable[qtCIdx] += (*reward - qtable[qtCIdx]) / LEARN_RATE_DIV;
}

void agentUpdate(float *qtable, uint8_t *cstate, uint8_t *nstate, float *reward,
                 uint8_t *action) {
  deviceUpdate<<<1, 1>>>(qtable, cstate, nstate, reward, action);
}

__global__ void deviceAction(float *qtable, uint8_t *cstate, uint8_t *action) {
  int qtableIndex = qtableAccessor(cstate);
  float currMax = qtable[qtableIndex];
  int newMax = 0;
  uint8_t currGuess = 0;
  for (int i = 0; i < 4; i++) {
    newMax = myMax(currMax, qtable[qtableIndex + i]);
    if (newMax > currMax) {
      currMax = newMax;
      currGuess = (uint8_t)i;
    }
  }
  *action = currGuess;
}

void agentAction(float *qtable, uint8_t *cstate, uint8_t *action) {
  deviceAction<<<1, 1>>>(qtable, cstate, action);
}

__global__ void getReward(uint8_t *cstate, uint8_t *nstate, float *reward) {
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  if (abs(nstate[tid] - CTR_STATE) != abs(cstate[tid] - CTR_STATE)) {
    // got either closer or farther to center state
    reward[tid] = 1.0 + abs(nstate[tid] - cstate[tid]) / NUM_STATES;
    // minimum of 1 reward, diminishing returns after
    if (abs(nstate[tid] - CTR_STATE) > abs(cstate[tid] - CTR_STATE)) {
      // nextstate got farther from center
      reward[tid] *= -1.0;
    }
  } else { // state maintained
    reward[tid] = 0.0;
  }
}

// avg out reward, store into first value of reward
__global__ void avgReward(float *reward) {
  float sum = 0;
  for (int i = 0; i < NUM_REGIONS; i++) {
    sum += reward[i];
  }
  reward[0] = sum / NUM_REGIONS;
}

void agentReward(uint8_t *cstate, uint8_t *nstate, float *reward) {
  // TODO make sure this is not bad;
  getReward<<<1, NUM_REGIONS>>>(cstate, nstate, reward);
  avgReward<<<1, 1>>>(reward);
}

__global__ void initQtable(float *qtable) {
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  qtable[tid] = 0;
}
void initvals(float *qtable) {
  size_t qtabSize = (NUM_STATES ^ NUM_REGIONS) * NUM_ACTION;
  initQtable<<<qtabSize / 64, 64>>>(qtable);
}
