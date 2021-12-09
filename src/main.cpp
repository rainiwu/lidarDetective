#include "iface/Lidar.hpp"
#include "iface/Drive.hpp"
#include "rc/Robot.hpp"
#include <chrono>

int main() {
  auto test = LiDet::Lidar();
  auto robot = LiDet::Robot();
  for (int i = 0; i < 10; i++) {
    robot(ROBOT_THUP);
    std::this_thread::sleep_for(std::chrono::seconds{1});
  }
  for (int i = 0; i < 10; i++) {
    robot(ROBOT_STRR);
    std::this_thread::sleep_for(std::chrono::seconds{1});
  }
  robot(ROBOT_STOP);
  robot(ROBOT_STRAIGHT);

  while (true) {
    test.graph(std::cout);
    std::this_thread::sleep_for(std::chrono::seconds{1});
    for (int i = 0; i < 50; i++)
      std::cout << '\n';
  }

  return 0;
}
