#ifndef Common_h
#define Common_h

#define CONSTANT 30
#define CONF_THRESH 10

// pwm values
#define THROTTLE_NEUTRAL 350
#define THROTTLE_FORWARD 390
#define THROTTLE_REVERSE 310
#define STEERING_NEUTRAL 400
#define STEERING_LEFT 290
#define STEERING_RIGHT 510
#define THROTTLE_PIN 2
#define STEERING_PIN 1

// rl algo actions
#define NUM_ACTION 4 // only first four actions used
#define ROBOT_THUP 0 // throttle up
#define ROBOT_THDN 1 // throttle down
#define ROBOT_STRL 2 // steer left
#define ROBOT_STRR 3 // steer right
#define ROBOT_STRAIGHT 4
#define ROBOT_STOP 5
#define PWM_DELAY 250

// rl configuration
#define NUM_REGIONS 3
#define NUM_STATES 5
#define CTR_STATE 3 // middle point of num states
#define MAX_WING 2
#define DISC_FACT 0.9     // discount factor
#define LEARN_RATE_DIV 10 // learning rate divisor
#define EPSILON 0.9

// maximum lidar value
#define LIDAR_MAX_V 2000
// target lidar value
#define LIDAR_TGT_V 1000
#define LIDAR_VALS 720
// divisor describing number of divs
#define LIDAR_DIV 5
// location of follow target
#define LIDAR_CENT 0

// qtable options
#define QTAB_SAVE 1 // save qtable on exit?
#define QTAB_LOAD 0 // load qtable on start?
#define QTAB_FILE "qtable.bin.v1"

// lidar graph values
#define GRAPH_DIST 1000
#define GRAPH_CHAR "1"
#define GRAPH_COLS 120
#define GRAPH_ROWS 40

#endif
