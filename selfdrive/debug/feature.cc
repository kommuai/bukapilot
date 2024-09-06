#include "selfdrive/common/features.h"
#include "selfdrive/common/params.h"

#include <iostream>

int main(int argc, char *argv[]) {
  // ./feature_driver <clear? 1:0> <package? 1:0> [package] features...
  Features f;

  bool clear = argv[1][0] == '1';
  bool set_package = argv[2][0] =='1';

  if (argc < 2 + (int) set_package) {
    std::cout << "no enuf arg" << std::endl;
    return -1;
  }

  if (clear)
    f.reset();

  int s = 3;
  if (set_package) {
    f.set_package(argv[s]);
    s++;
  }

  for (; s < argc; s++)
    std::cout << ((f.has(argv[s])) ? "f has " : "f hasn't ") << argv[s] << std::endl;
}
