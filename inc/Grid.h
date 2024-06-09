#pragma once

#include "common.h"

namespace domain {
class Grid {
public:
  Grid(string fname);

private:
  int rows;
  int cols;
  std::vector<int> map;
  string map_name;
};
} // namespace domain
