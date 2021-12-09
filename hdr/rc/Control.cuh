#ifndef Control_cuh
#define Control_cuh

#include "Common.h"
#include <cstdint>
#include <cuda_runtime.h>

// get states from Lidar data
void calcState(uint16_t *laserDat, uint8_t *states);

int qtableAccessor(uint8_t *state);
int max(int a, int b);

// update qtable
void agentUpdate(float *qtable, uint8_t *cstate, uint8_t *nstate, float *reward,
                 uint8_t *action);

// take action based on state, qtable
void agentAction(float *qtable, uint8_t *cstate, uint8_t *action);

// determine appropriate reward
void agentReward(uint8_t *cstate, uint8_t *nstate, float *reward);

// initialize values
void initvals(float *qtable);

#endif
