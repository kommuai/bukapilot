#pragma once

#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

#include "common/params.h"
#include "common/util.h"
#include "system/hardware/base.h"

class HardwareKa2 : public HardwareNone {
public:
  static constexpr float MAX_VOLUME = 0.9;
  static constexpr float MIN_VOLUME = 0.1;
  static bool TICI() { return false; }
  static bool AGNOS() { return false; }
  static bool KA2() { return true; }
  static std::string get_os_version() {
    return "RK-AGNOS " + util::read_file("/VERSION");
  }

  static std::string get_name() {
    return "KommuAssist2";
  }

  static cereal::InitData::DeviceType get_device_type() {
    return cereal::InitData::DeviceType::KA2;
  }

  static std::string get_serial() {
    static std::string serial("");
    if (serial.empty()) {
      std::ifstream stream("/proc/cmdline");
      std::string cmdline;
      std::getline(stream, cmdline);

      auto start = cmdline.find("serialno=");
      if (start == std::string::npos) {
        serial = "cccccc";
      } else {
        auto end = cmdline.find(" ", start + 9);
        serial = cmdline.substr(start + 9, end - start - 9);
      }
    }
    return serial;
  }

  static void reboot() { std::system("sudo reboot"); }
  static void poweroff() { std::system("sudo poweroff"); }
  static bool get_ssh_enabled() { return Params().getBool("SshEnabled"); }
  static void set_ssh_enabled(bool enabled) { Params().putBool("SshEnabled", enabled); }

  static void config_cpu_rendering(bool offscreen) {
    if (offscreen) {
      setenv("QT_QPA_PLATFORM", "offscreen", 1); // offscreen doesn't work with EGL/GLES
    }
    setenv("__GLX_VENDOR_LIBRARY_NAME", "mesa", 1);
    setenv("LP_NUM_THREADS", "0", 1); // disable threading so we stay on our assigned CPU
  }
};
