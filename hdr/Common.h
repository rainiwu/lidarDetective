#ifndef Common_h
#define Common_h

#define CONSTANT 30
#define CONF_THRESH 100

// pwm values
#define THROTTLE_NEUTRAL 350
#define THROTTLE_FORWARD 370 //was 390
#define THROTTLE_REVERSE 330 //was 310
#define STEERING_NEUTRAL 410
#define STEERING_LEFT 300
#define STEERING_RIGHT 520
#define THROTTLE_PIN 2
#define STEERING_PIN 1

#define BIAS_MULT 10

// rl algo actions
#define NUM_ACTION 4 // only first four actions used
#define ROBOT_THUP 0 // throttle up
#define ROBOT_THDN 1 // throttle down
#define ROBOT_STRL 2 // steer left
#define ROBOT_STRR 3 // steer right
#define ROBOT_STRAIGHT 4
#define ROBOT_STOP 5
#define DELTA 0       // whether or not to use delta
#define THR_DELT 0.002 // throttle delta values
#define STR_DELT 0.3 // steering delta
#define PWM_DELAY 10 // delay in milliseconds between each call

// rl configuration
#define NUM_REGIONS 9 // keep at odd number!
#define NUM_STATES 9  // keep at odd number!
// number of states is not max state
// e.g. NUM_STATES = 3, means states can be 0,1,2
// middle is 1
#define CTR_STATE NUM_STATES / 2 // middle point of num states

#define DISC_FACT 0.9    // discount factor
#define LEARN_RATE_DIV 0.01 // learning rate divisor
#define EPSILON 0

// maximum lidar value
#define LIDAR_MAX_V 4000
// target lidar value
#define LIDAR_TGT_V 2000
#define LIDAR_VALS 1080
// divisor describing number of divs
#define LIDAR_DIV 4
// location of follow target
#define LIDAR_CENT 135

// qtable options
#define QTAB_SAVE 1 // save qtable on exit?
#define QTAB_LOAD 0 // load qtable on start?
#define QTAB_FILE "qtable.bin.v1"

// lidar graph values
#define GRAPH_DIST 4000
#define GRAPH_CHAR "1"
#define GRAPH_COLS 120
#define GRAPH_ROWS 40

#endif
