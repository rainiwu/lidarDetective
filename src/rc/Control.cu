#include "rc/Control.cuh"
#include <cstdio>
#include <curand.h>
#include <curand_kernel.h>

// region number of values
#define REG_NV (LIDAR_VALS / LIDAR_DIV) / NUM_REGIONS

__device__ int qtableAccessor(uint8_t *state) {
  int qtableIndex = 0;
  for (int i = 0; i < NUM_REGIONS; i++)
    qtableIndex += state[i] * pow(NUM_STATES, i);
  return qtableIndex * 4;
}

__device__ void qtableDeacc(int index, uint8_t *outarray) {
  for (int i = NUM_REGIONS - 1; i >= 0; i--) {
    for (int j = NUM_STATES - 1; j >= 0; j--) {
      if (index > j * pow(NUM_STATES, i)) {
        index = index - (j * pow(NUM_STATES, i));
        outarray[i] = j;
      }
    }
  }
}

__global__ void init_randstate(curandState *state) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  curand_init(clock() + tid, tid, 0, &state[tid]);
}

void initrand(curandState *state) { init_randstate<<<1, 1>>>(state); }

__device__ float myMax(float a, float b) { return (a > b) ? a : b; }

__global__ void findState(uint16_t *laserDat, uint8_t *states) {
  // parallelized by region - tid is region num
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  int sum = 0;
  int offset = 0;
  int avg, tmp;
  for (size_t i = 0; i < REG_NV; i++) {
    // iterate through region
    tmp = laserDat[tid * REG_NV + i];
    // if bad value, discard
    if (0 == tmp) {
      tmp = 0;
      offset++;
    } else if (LIDAR_MAX_V < tmp) {
      tmp = LIDAR_MAX_V;
    }
    sum += tmp;
  }
  // avg of region is sum / numvals
  avg = sum / (REG_NV - offset);
  int stateSize = LIDAR_MAX_V / NUM_STATES;
  // assumes target is exactly half of max v
  for (int i = 1; i <= NUM_STATES; i++) {
    if (avg <= stateSize * i) {
      states[tid] = i;
      printf(
          "current state for region %d is %d\nstateSize is %d and avg is %d\n",
          tid, i, stateSize * i, avg);
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
    qtable[qtCIdx] += ((*reward + DISC_FACT * qtable[qtNIdx] - qtable[qtCIdx]) *
                       LEARN_RATE_DIV);
  else
    qtable[qtCIdx] += (*reward - qtable[qtCIdx]) * LEARN_RATE_DIV;
}

void agentUpdate(float *qtable, uint8_t *cstate, uint8_t *nstate, float *reward,
                 uint8_t *action) {
  deviceUpdate<<<1, 1>>>(qtable, cstate, nstate, reward, action);
}

__global__ void deviceAction(float *qtable, uint8_t *cstate, uint8_t *action,
                             curandState *aState) {

  // TODO: parallelize
  float rand = curand_uniform(aState);
  if (rand < EPSILON) {
    rand = curand_uniform(aState);
    *action = (short)(rand * 3.99);
    return;
  }

  int qtableIndex = qtableAccessor(cstate);
  float currMax = qtable[qtableIndex];
  float newMax = currMax;
  uint8_t currGuess = 0;
  for (int i = 0; i < 4; i++) {
    newMax = myMax(currMax, qtable[qtableIndex + i]);
    if (newMax > currMax) {
      currMax = newMax;
      currGuess = (uint8_t)i;
    }
  }
  printf("CurrAction: %d\ncurrMax: %f\n", *action, currMax);
  *action = currGuess;
  if (currGuess == 0 && currMax == 0) {
    printf("rolling random\n");
    *action = (short)(rand * 3.99);
  }
}

void agentAction(float *qtable, uint8_t *cstate, uint8_t *action,
                 curandState *astate) {
  deviceAction<<<1, 1>>>(qtable, cstate, action, astate);
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
  int stateindex = tid / 4;
  int action = tid % 4;
  uint8_t state[NUM_REGIONS];
  qtableDeacc(stateindex, (uint8_t *)&state);
  if (state[NUM_REGIONS / 2] < CTR_STATE) {
    switch (action) {
    case ROBOT_THUP:
      qtable[tid] = -10;
      return;
    case ROBOT_THDN:
      qtable[tid] = 10;
      return;
    default:
      qtable[tid] = 0;
      return;
    }
  } else if (state[NUM_REGIONS / 2] > CTR_STATE) {
    switch (action) {
    case ROBOT_THUP:
      qtable[tid] = 10;
      return;
    case ROBOT_THDN:
      qtable[tid] = -10;
      return;
    default:
      qtable[tid] = 0;
      return;
    }
  }
  qtable[tid] = 0;
}

__global__ void printQTable(float *qtable) {
  for (int i = 0; i < pow(NUM_STATES, NUM_REGIONS) * NUM_ACTION; i++) {
    printf("%d: %f\n", i, qtable[i]);
  }
}
void initvals(float *qtable) {
  size_t qtabSize = pow(NUM_STATES, NUM_REGIONS) * NUM_ACTION;
  printf("qtabsize %d\n", (int)qtabSize);
  initQtable<<<qtabSize / 64, 64>>>(qtable);
  cudaDeviceSynchronize();
  printQTable<<<1, 1>>>(qtable);
}
