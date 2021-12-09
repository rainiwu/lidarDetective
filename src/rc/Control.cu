#include "rc/Control.cuh"

void calcState(uint16_t *laserDat, uint8_t *states) {}

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

void initvals(float *qtable) {}
