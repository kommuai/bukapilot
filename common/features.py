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

  def set(self, feature):
    v = int.from_bytes(self.p.get("FeaturesValue"), byteorder="little")
    v = v | self.dict[feature]
    self.p.put("FeaturesValue", int.to_bytes(v, byteorder="little", length=8))

  def set_package(self, package):
    if package not in self.packages:
      return
    self.reset()
    for feature in self.packages[package]:
      self.set(feature)
    self.p.put("FeaturesPackage", package)

