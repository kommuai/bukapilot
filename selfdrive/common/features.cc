#include "selfdrive/common/features.h"
#include "json11.hpp"
#include <sstream>
#include <set>

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

int Features::set_package(const std::string& featuresInput) {
  reset();
  std::string cleaned_featuresInput = featuresInput;

  cleaned_featuresInput.erase(remove_if(cleaned_featuresInput.begin(), cleaned_featuresInput.end(), [](char c) {
    return ::isspace(c) || c == '.';
  }), cleaned_featuresInput.end());

  uint64_t total_value = 0;  // To accumulate the sum of valid features
  std::stringstream formatted_features;
  bool first = true;
  bool has_invalid_feature = false; // Flag to track if any feature is invalid
  std::set<std::string> added_features; // To track already added feature package names

  std::stringstream ss(cleaned_featuresInput);
  std::string pkg_feature;

  while (std::getline(ss, pkg_feature, ',')) {
    if (packages.find(pkg_feature) != packages.end()) {
      // Check if pkg_feature is already added
      if (added_features.find(pkg_feature) == added_features.end()) {
        added_features.insert(pkg_feature);  // Mark pkg_feature as added
        const auto& features_list = packages[pkg_feature];
        for (const auto& feature : features_list) {
          uint64_t feature_value = features[feature];
          total_value |= feature_value;
        }
        if (pkg_feature != "default") {
          if (!first) {
            formatted_features << ", ";
          }
          formatted_features << pkg_feature;
          first = false;
        }
      }
    } else {
      has_invalid_feature = true;
    }
  }

  if (formatted_features.str().empty()) {
    formatted_features.str("default");
  }

  params.put("FeaturesValue", (const char*)&total_value, sizeof(total_value)); // Set the accumulated value to FeaturesValue
  params.put("FeaturesPackageNames", formatted_features.str()); // Set the final formatted string of valid features to Params
  return has_invalid_feature ? -1 : static_cast<int>(total_value); // Return -1 if any feature was invalid; otherwise return the total as int
}
