# Re-export for opendbc proton/byd/dnga that import from openpilot.selfdrive.car.docs_definitions
from opendbc.car.docs_definitions import *  # noqa: F401, F403
from opendbc.car.docs_definitions import CarDocs

# Alias used by proton/byd/dnga values
CarInfo = CarDocs
