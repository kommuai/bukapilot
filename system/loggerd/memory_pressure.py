#!/usr/bin/env python3
"""
Memory pressure handling utilities for openpilot.
Provides functions to check memory usage and clear caches when memory is low.
"""
import psutil
from openpilot.common.swaglog import cloudlog

# Memory thresholds
MEMORY_WARNING_THRESHOLD = 85  # Start taking action at 85%
MEMORY_CRITICAL_THRESHOLD = 90  # Critical threshold - same as controlsd
MEMORY_EMERGENCY_THRESHOLD = 95  # Emergency - clear everything possible

def get_memory_usage_percent() -> int:
  """Get current memory usage percentage."""
  return int(round(psutil.virtual_memory().percent))

def is_memory_pressure_low() -> bool:
  """Check if memory pressure is low (below warning threshold)."""
  return get_memory_usage_percent() < MEMORY_WARNING_THRESHOLD

def is_memory_pressure_high() -> bool:
  """Check if memory pressure is high (above warning threshold)."""
  return get_memory_usage_percent() >= MEMORY_WARNING_THRESHOLD

def is_memory_pressure_critical() -> bool:
  """Check if memory pressure is critical (above critical threshold)."""
  return get_memory_usage_percent() >= MEMORY_CRITICAL_THRESHOLD

def is_memory_pressure_emergency() -> bool:
  """Check if memory pressure is in emergency state (above emergency threshold)."""
  return get_memory_usage_percent() >= MEMORY_EMERGENCY_THRESHOLD

def clear_xattr_cache() -> int:
  """
  Clear the extended attributes cache.
  Returns number of entries cleared.
  """
  try:
    from openpilot.system.loggerd.xattr_cache import _cached_attributes
    cleared = len(_cached_attributes)
    _cached_attributes.clear()
    if cleared > 0:
      cloudlog.info(f"Cleared {cleared} xattr cache entries due to memory pressure")
    return cleared
  except Exception as e:
    cloudlog.error(f"Failed to clear xattr cache: {e}")
    return 0

def clear_filesystem_cache() -> None:
  """
  Attempt to clear filesystem page cache by dropping caches.
  Requires root privileges. This is a best-effort operation.
  """
  try:
    import subprocess
    # Write 1 to drop page cache (see /proc/sys/vm/drop_caches)
    # This requires root, so we try but don't fail if it doesn't work
    subprocess.run(
      ["sudo", "sh", "-c", "echo 1 > /proc/sys/vm/drop_caches"],
      check=False,
      capture_output=True,
      timeout=5
    )
    cloudlog.info("Attempted to clear filesystem page cache")
  except Exception as e:
    # Silently fail - this is optional
    pass

def handle_memory_pressure(clear_caches: bool = True, clear_fs_cache: bool = False) -> dict:
  """
  Handle memory pressure by clearing caches and returning status.
  
  Args:
    clear_caches: If True, clear application caches (xattr cache)
    clear_fs_cache: If True, attempt to clear filesystem page cache (requires root)
  
  Returns:
    dict with memory usage info and actions taken
  """
  mem_percent = get_memory_usage_percent()
  mem_info = psutil.virtual_memory()
  
  result = {
    "memory_percent": mem_percent,
    "memory_available_mb": mem_info.available // (1024 * 1024),
    "memory_total_mb": mem_info.total // (1024 * 1024),
    "xattr_cache_cleared": 0,
    "fs_cache_cleared": False,
  }
  
  if is_memory_pressure_high():
    if clear_caches:
      result["xattr_cache_cleared"] = clear_xattr_cache()
    
    if clear_fs_cache and is_memory_pressure_critical():
      clear_filesystem_cache()
      result["fs_cache_cleared"] = True
    
    cloudlog.warning(
      f"Memory pressure detected: {mem_percent}% used, "
      f"{result['memory_available_mb']}MB available. "
      f"Cleared {result['xattr_cache_cleared']} cache entries."
    )
  
  return result

def should_skip_filesystem_operation() -> bool:
  """
  Determine if filesystem operations should be skipped due to memory pressure.
  Returns True if memory is critical and operations should be skipped.
  """
  return is_memory_pressure_critical()
