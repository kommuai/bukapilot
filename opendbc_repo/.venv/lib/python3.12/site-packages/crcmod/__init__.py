"""crcmod is a Python module for generating objects that compute the Cyclic
Redundancy Check.  Any 8, 16, 24, 32, or 64 bit polynomial can be used.

The following are the public components of this module.

Crc -- a class that creates instances providing the same interface as the
algorithms in the hashlib module in the Python standard library.  These
instances also provide a method for generating a C/C++ function to compute
the CRC.

mkCrcFun -- create a Python function to compute the CRC using the specified
polynomial and initial value.  This provides a much simpler interface if
all you need is a function for CRC calculation.
"""

from .crcmod import Crc, mkCrcFun
from ._version import __version__
from . import predefined  # Auto-import for drop-in compatibility with original crcmod

__all__ = ("Crc", "mkCrcFun", "__version__", "predefined")
