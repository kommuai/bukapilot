#pragma once

#include <map>
#include <string>
#include <vector>

#include "selfdrive/common/params.h"

class Features {

public:
  Features();
  void clear(const std::string& feature);
  bool has(const std::string& feature);
  void reset();
  void set(const std::string& feature);
  void set_package(const std::string& package);

  Params params;
  std::map<std::string, uint64_t> features;
  std::map<std::string, std::vector<std::string>> packages;
};

