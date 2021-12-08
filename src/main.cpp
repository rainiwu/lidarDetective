#include "iface/Lidar.hpp"
#include "iface/Drive.hpp"
#include <chrono>

int main() {
  auto test = LiDet::Lidar();
  auto robotControl = LiDet::Drive();
  robotControl.setThrottle(0);
  robotControl.setSteering(0);
  // robotControl.setThrottle(0);
  // robotControl.setThrottle(-0.2);

  while (true) {
    test.graph(std::cout);
    std::this_thread::sleep_for(std::chrono::seconds{1});
    for (int i = 0; i < 50; i++)
      std::cout << '\n';
  }

  return 0;
}
