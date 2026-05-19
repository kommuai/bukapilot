import errno
import os

import xattr

_cached_attributes: dict[tuple, bytes | None] = {}

# Maximum cache size to prevent unbounded growth
MAX_CACHE_SIZE = 10000

def _trim_cache_if_needed() -> None:
  """Trim cache if it exceeds maximum size (LRU eviction)."""
  if len(_cached_attributes) > MAX_CACHE_SIZE:
    # Remove oldest 25% of entries (simple approach - dict iteration order is insertion order in Python 3.7+)
    to_remove = len(_cached_attributes) - int(MAX_CACHE_SIZE * 0.75)
    keys_to_remove = list(_cached_attributes.keys())[:to_remove]
    for key in keys_to_remove:
      _cached_attributes.pop(key, None)

def getxattr(path: str, attr_name: str) -> bytes | None:
  key = (path, attr_name)
  if key not in _cached_attributes:
    try:
      response = xattr.getxattr(path, attr_name)
    except OSError as e:
      # ENODATA (Linux) or ENOATTR (macOS) means attribute hasn't been set
      if e.errno == errno.ENODATA or (hasattr(errno, 'ENOATTR') and e.errno == errno.ENOATTR):
        response = None
      else:
        raise
    _cached_attributes[key] = response
    _trim_cache_if_needed()
  return _cached_attributes[key]

def setxattr(path: str, attr_name: str, attr_value: bytes) -> None:
  _cached_attributes.pop((path, attr_name), None)
  os.setxattr(path, attr_name, attr_value)

def clear_cache() -> int:
  """Clear the entire cache. Returns number of entries cleared."""
  cleared = len(_cached_attributes)
  _cached_attributes.clear()
  return cleared
