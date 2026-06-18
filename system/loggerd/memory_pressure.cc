#include "system/loggerd/memory_pressure.h"

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

#include "common/swaglog.h"
#include "common/timing.h"

namespace MemoryPressure {

static int parse_meminfo() {
  std::ifstream stream("/proc/meminfo");
  if (!stream.is_open()) {
    return -1;
  }

  std::unordered_map<std::string, uint64_t> mem_info;
  std::string line, key;
  
  while (std::getline(stream, line)) {
    uint64_t val = 0;
    std::istringstream iss(line);
    if (iss >> key >> val) {
      // Values in /proc/meminfo are in KB, convert to bytes
      mem_info[key] = val * 1024;
    }
  }

  if (mem_info.find("MemTotal:") == mem_info.end() || 
      mem_info.find("MemAvailable:") == mem_info.end()) {
    return -1;
  }

  uint64_t mem_total = mem_info["MemTotal:"];
  uint64_t mem_available = mem_info["MemAvailable:"];
  
  if (mem_total == 0) {
    return -1;
  }

  uint64_t mem_used = mem_total - mem_available;
  int percent = (mem_used * 100) / mem_total;
  
  return percent;
}

int get_memory_usage_percent() {
  static int cached_percent = -1;
  static double last_check_ms = 0;
  
  // Cache for 1 second to avoid reading /proc/meminfo too frequently
  double now_ms = millis_since_boot();
  if (cached_percent >= 0 && (now_ms - last_check_ms) < 1000) {
    return cached_percent;
  }

  int percent = parse_meminfo();
  if (percent >= 0) {
    cached_percent = percent;
    last_check_ms = now_ms;
  }
  
  return percent;
}

bool is_memory_pressure_low() {
  int percent = get_memory_usage_percent();
  return percent >= 0 && percent < MEMORY_WARNING_THRESHOLD;
}

bool is_memory_pressure_high() {
  int percent = get_memory_usage_percent();
  return percent >= MEMORY_WARNING_THRESHOLD;
}

bool is_memory_pressure_critical() {
  int percent = get_memory_usage_percent();
  return percent >= MEMORY_CRITICAL_THRESHOLD;
}

bool is_memory_pressure_emergency() {
  int percent = get_memory_usage_percent();
  return percent >= MEMORY_EMERGENCY_THRESHOLD;
}

bool should_skip_filesystem_operation() {
  return is_memory_pressure_critical();
}

}  // namespace MemoryPressure
