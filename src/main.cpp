#include "rc/Control.hpp"
#include <chrono>

using namespace LiDet;

int main() {
  auto top = Control();
  top.start();
  auto buff = std::string("");
  while (true) {
    std::cout << "do anything to stop" << std::endl;
    std::cin >> buff;
    if (buff != "")
      break;
  }
  top.stop();

  return 0;
}
