#include "iface/Drive.hpp"

namespace LiDet {
Drive::Drive() {
  pca9685 = new PCA9685();
  int err = pca9685->openPCA9685();
  if (err < 0) {
    printf("Error: %d", pca9685->error);
  }
}

Drive &Drive::operator=(const Drive &aCopy) {
  pca9685 = aCopy.pca9685;
  return *this;
}

Drive::~Drive() {
  pca9685->setPWM(THROTTLE_PIN, 0, THROTTLE_NEUTRAL);
  pca9685->setPWM(STEERING_PIN, 0, STEERING_NEUTRAL);
  pca9685->closePCA9685();
}

void Drive::setThrottle(const float &aValue) {
  if (aValue < -1.0 || aValue > 1.0) {
    std::cout << "Throttle value out of bounds" << std::endl;
    return;
  }
  throttle = aValue;

  if (aValue > 0) { // forward
    int pwmValue = THROTTLE_NEUTRAL +
                   (int)(aValue * (float)(THROTTLE_FORWARD - THROTTLE_NEUTRAL));
    std::cout << pwmValue << "\n";
    pca9685->setPWM(THROTTLE_PIN, 0, pwmValue);
  } else if (aValue < 0) { // reverse
    int pwmValue = THROTTLE_NEUTRAL +
                   (int)(aValue * (float)(THROTTLE_FORWARD - THROTTLE_NEUTRAL));
    std::cout << pwmValue << "\n";
    pca9685->setPWM(THROTTLE_PIN, 0, pwmValue);
    pca9685->setPWM(THROTTLE_PIN, 0, THROTTLE_NEUTRAL);
    pca9685->setPWM(THROTTLE_PIN, 0, pwmValue);

  } else { // stopped
    pca9685->setPWM(THROTTLE_PIN, 0, THROTTLE_NEUTRAL);
  }
}

void Drive::setSteering(const float &aValue) {
  if (aValue < -1.0 || aValue > 1.0) {
    std::cout << "Steering value out of bounds" << std::endl;
    return;
  }
  steering = aValue;

  if (aValue > 0) { // right
    int pwmValue = STEERING_NEUTRAL +
                   (int)(aValue * (float)(STEERING_RIGHT - STEERING_NEUTRAL));
    std::cout << pwmValue << "\n";
    pca9685->setPWM(STEERING_PIN, 0, pwmValue);
  } else if (aValue < 0) { // left
    int pwmValue = STEERING_NEUTRAL +
                   (int)(aValue * (float)(STEERING_RIGHT - STEERING_NEUTRAL));
    std::cout << pwmValue << "\n";
    pca9685->setPWM(STEERING_PIN, 0, pwmValue);

  } else { // straight
    pca9685->setPWM(STEERING_PIN, 0, STEERING_NEUTRAL);
  }
}

const float &Drive::getThrottle() { return throttle; }
const float &Drive::getSteering() { return steering; }
} // namespace LiDet
