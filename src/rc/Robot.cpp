#include "rc/Robot.hpp"

namespace LiDet {

Robot::Robot() {
    throtStop();
    steerStraight();
}

Robot::~Robot() {
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
    // myDrive.setThrottle(myDrive.getThrottle() + delta_throt);
    myDrive.setThrottle(0.2);
}
void Robot::throtDn() {
    // myDrive.setThrottle(myDrive.getThrottle() - delta_throt);
    myDrive.setThrottle(-0.2);
}
void Robot::steerLf() {
    myDrive.setSteering(myDrive.getSteering() + delta_steer);
}
void Robot::steerRi() {
    myDrive.setSteering(myDrive.getSteering() - delta_steer);
}
void Robot::throtStop() {
    myDrive.setThrottle(0);
}
void Robot::steerStraight() {
    myDrive.setSteering(0);
}

}