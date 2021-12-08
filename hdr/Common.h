#ifndef Common_h
#define Common_h

#define CONSTANT 30
#define LIDAR_VALS 360
#define CONF_THRESH 100
#define DEFAULT_THROTTLE 1100
#define DEFAULT_STEERING 1000

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
