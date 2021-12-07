#include "iface/Lidar.hpp"
#include <chrono>

int main() {
  auto test = LiDet::Lidar();
  while (true) {
    test.graph(std::cout);
    std::this_thread::sleep_for(std::chrono::seconds{1});
    for (int i = 0; i < 50; i++)
      std::cout << '\n';
  }

  return 0;
}
