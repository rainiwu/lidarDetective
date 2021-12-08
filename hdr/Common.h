#ifndef Common_h
#define Common_h

#define CONSTANT 30
#define LIDAR_VALS 360
#define CONF_THRESH 100

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
#define ROBOT_THUP 0 // throttle up
#define ROBOT_THDN 1 // throttle down
#define ROBOT_STRL 2 // steer left
#define ROBOT_STRR 3 // steer right

#define SAVE_QTAB 1 // save qtable on exit?

// lidar graph values
#define GRAPH_DIST 1000
#define GRAPH_CHAR "1"
#define GRAPH_COLS 120
#define GRAPH_ROWS 40

#endif
