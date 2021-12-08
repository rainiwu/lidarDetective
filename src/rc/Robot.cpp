#include "rc/Robot.hpp"

namespace LiDet {

Robot::Robot() {
    myDrive = Drive();
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
        default:
            break;
    }
}

void Robot::throtUp() {
    myDrive.setThrottle(myDrive.getThrottle() + delta_throt);
}
void Robot::throtDn() {
    myDrive.setThrottle(myDrive.getThrottle() - delta_throt);
}
void Robot::steerLf() {
    myDrive.setSteering(myDrive.getSteering() + delta_throt);
}
void Robot::steerRi() {
    myDrive.setSteering(myDrive.getSteering() - delta_throt);
}
}