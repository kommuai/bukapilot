#pragma once

#include <map>
#include <string>
#include <vector>
#include <set>

#include "selfdrive/common/params.h"

class Features {

public:
  Features();
  void clear(const std::string& feature);
  bool has(const std::string& feature);
  void reset();
  int set_package(const std::string& featuresInput);

  Params params;
  std::map<std::string, uint64_t> features;
  std::map<std::string, std::vector<std::string>> packages;
};

