from contextlib import contextmanager
import json

from common.params import Params

class Features:
  def __init__(self):
    self.p = Params()
    j = json.loads(self.p.get("FeaturesDict", encoding="utf-8"))
    self.dict = j["features"]
    self.packages = j["packages"]

  def clear(self, feature):
    v = int.from_bytes(self.p.get("FeaturesValue"), byteorder="little")
    v = v & (~self.dict[feature])
    self.p.put("FeaturesValue", int.to_bytes(v, byteorder="little", length=8))

  def has(self, feature) -> bool:
    v = int.from_bytes(self.p.get("FeaturesValue"), byteorder="little")
    return (v & self.dict[feature]) != 0

  def reset(self):
    self.p.put("FeaturesValue", b'\x00' * 8)

  def set_package(self, package):
    self.reset()
    cleaned_package = package.replace(" ", "").replace(".", "")

    total_value = 0  # To accumulate the sum of valid features
    formatted_features = []
    has_invalid_feature = False  # Flag to track if any feature is invalid
    added_features = set()  # To track added features and avoid duplicates

    # Split the cleaned input string by commas
    packages_list = cleaned_package.split(',')

    for pkg_feature in packages_list:
      if pkg_feature in self.packages:
        features_list = self.packages[pkg_feature]
        for feature in features_list:
          if feature != "default" and feature not in added_features:  # Skip adding "default" and duplicates
            feature_value = self.dict[feature]
            total_value |= feature_value  # Accumulate feature values
            formatted_features.append(feature)
            added_features.add(feature)  # Mark the feature as added
      else:
        has_invalid_feature = True  # Mark that there was an invalid feature

    # Set formatted features or default if none
    if formatted_features:
      self.p.put("FeaturesPackageNames", ', '.join(formatted_features))
    else:
      self.p.put("FeaturesPackageNames", "default")

    # Set the accumulated value to FeaturesValue
    self.p.put("FeaturesValue", int.to_bytes(total_value, byteorder="little", length=8))
    return -1 if has_invalid_feature else total_value
