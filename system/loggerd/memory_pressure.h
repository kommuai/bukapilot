#pragma once

#include <cstdint>

namespace MemoryPressure {
  // Memory thresholds (matching Python implementation)
  constexpr int MEMORY_WARNING_THRESHOLD = 75;   // Start taking action at 75%
  constexpr int MEMORY_CRITICAL_THRESHOLD = 80;  // Critical threshold - same as controlsd
  constexpr int MEMORY_EMERGENCY_THRESHOLD = 85; // Emergency - clear everything possible

  // Get current memory usage percentage
  // Returns -1 on error
  int get_memory_usage_percent();

  // Check if memory pressure is low (below warning threshold)
  bool is_memory_pressure_low();

  // Check if memory pressure is high (above warning threshold)
  bool is_memory_pressure_high();

  // Check if memory pressure is critical (above critical threshold)
  bool is_memory_pressure_critical();

  // Check if memory pressure is in emergency state (above emergency threshold)
  bool is_memory_pressure_emergency();

  // Should skip filesystem operations due to memory pressure?
  bool should_skip_filesystem_operation();
}
