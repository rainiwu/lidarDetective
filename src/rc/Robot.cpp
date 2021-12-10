#include "rc/Robot.hpp"

namespace LiDet {

Robot::Robot() {
  throtStop();
  steerStraight();
}

Robot::~Robot() {
  std::cout << "Should be stopping\n";
  throtStop();
  steerStraight();
}

void Robot::operator()(uint8_t action) {
  switch (action) {
  case ROBOT_THUP:
    throtUp();
    break;
  case ROBOT_THDN:
    throtDn();
    break;
  case ROBOT_STRL:
    steerLf();
    break;
  case ROBOT_STRR:
    steerRi();
    break;
  case ROBOT_STRAIGHT:
    steerStraight();
    break;
  case ROBOT_STOP:
    throtStop();
    break;
  default:
    break;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds{PWM_DELAY});
}

void Robot::throtUp() {
  if (DELTA) {
    float newThrot = myDrive.getThrottle() + delta_throt;
    if (newThrot > -0.6 && newThrot < 0.6) {
      newThrot = 0.6;
    }
    myDrive.setThrottle(newThrot);
  } else {
    myDrive.setThrottle(0.6);
  }
}
void Robot::throtDn() {
  if (DELTA) {
    float newThrot = myDrive.getThrottle() - delta_throt;
    if (newThrot < 0.6 && newThrot > -0.6) {
      newThrot = -0.6;
    }
    myDrive.setThrottle(newThrot);
  } else {
    myDrive.setThrottle(-0.6);
  }
}
void Robot::steerLf() {
  // std::cout << "Turning left\n";
  if (DELTA)
    myDrive.setSteering(myDrive.getSteering() + delta_steer);
  else
    myDrive.setSteering(-0.4);
}
void Robot::steerRi() {
  // std::cout << "Turning right\n";
  if (DELTA)
    myDrive.setSteering(myDrive.getSteering() - delta_steer);
  else
    myDrive.setSteering(0.3);
}
void Robot::throtStop() { myDrive.setThrottle(0); }
void Robot::steerStraight() { myDrive.setSteering(0); }

} // namespace LiDet
