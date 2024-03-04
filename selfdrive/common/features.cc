#include "selfdrive/common/features.h"

#include "json11.hpp"

Features::Features() {
  std::string err;

  auto dict = json11::Json::parse(Params().get("FeaturesDict"), err);

  static_assert(sizeof(double) >= 8, "double should be big");

  for (auto const& [f, v] : dict["features"].object_items())
    features[f] = (uint64_t) v.number_value();

  for (auto const& [p, l] : dict["packages"].object_items()) {
    packages[p]; // ensure empty entry is inserted
    for (auto const& f : l.array_items())
      packages[p].push_back(f.string_value());
  }
}

void Features::clear(const std::string& feature) {
  uint64_t v = *((uint64_t *) params.get("FeaturesValue").c_str());
  v &= ~(features[feature]);
  params.put("FeaturesValue", (const char *) &v, 8);
}

bool Features::has(const std::string& feature) {
  auto v = *((uint64_t *) params.get("FeaturesValue").c_str());
  return (bool) (v & features[feature]);
}

void Features::reset() {
  uint64_t v = 0;
  params.put("FeaturesValue", (const char *) &v, 8);
}

void Features::set(const std::string& feature) {
  uint64_t v = *((uint64_t *) params.get("FeaturesValue").c_str());
  v |= features[feature];
  params.put("FeaturesValue", (const char *) &v, 8);
}

void Features::set_package(const std::string& package) {
  reset();
  if (packages.find(package) == packages.end())
    return;
  for (auto const& feature : packages[package])
    set(feature);
  params.put("FeaturesPackage", package);
}
