#include "system/camerad/camera/common.h"

#include <chrono>
#include <vector>

#include "common/swaglog.h"
#include "common/util.h"

namespace {

bool cpu_online(int cpu) {
  const std::string path = util::string_format("/sys/devices/system/cpu/cpu%d/online", cpu);
  if (!util::file_exists(path)) {
    return true;
  }
  return util::strip(util::read_file(path)) == "1";
}

void wait_for_cpus_online(const std::vector<int> &cores, int timeout_ms = 10000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    bool ready = true;
    for (int core : cores) {
      if (!cpu_online(core)) {
        ready = false;
        break;
      }
    }
    if (ready) {
      return;
    }
    util::sleep_for(50);
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  const std::vector<int> cores = {6};
  wait_for_cpus_online(cores);
  int ret = util::set_core_affinity(cores);
  if (ret != 0) {
    LOGW("failed to set camerad core affinity");
  }

  camerad_thread();
  return 0;
}
