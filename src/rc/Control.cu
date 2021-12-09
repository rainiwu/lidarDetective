#include "rc/Control.cuh"

void calcState(uint16_t *laserDat, uint8_t *states) {}

void agentUpdate(float *qtable, uint8_t *cstate, uint8_t *nstate, float *reward,
                 uint8_t *action) {}

void agentAction(float *qtable, uint8_t *cstate, uint8_t *action) {}

void agentReward(uint8_t *cstate, uint8_t *nstate, float *reward) {}

void initvals(float *qtable) {}
