#pragma once

#include <cassert>
#include <memory>
#include <string>

#include "cereal/messaging/messaging.h"
#include "common/util.h"
#include "common/timing.h"
#include "system/hardware/hw.h"
#include "system/loggerd/memory_pressure.h"

class RawFile {
 public:
  RawFile(const std::string &path) {
    file = util::safe_fopen(path.c_str(), "wb");
    assert(file != nullptr);
    write_count = 0;
    last_flush_ms = millis_since_boot();
  }
  ~RawFile() {
    util::safe_fflush(file);
    int err = fclose(file);
    assert(err == 0);
  }
  inline void write(void* data, size_t size) {
    int written = util::safe_fwrite(data, 1, size, file);
    assert(written == size);
    write_count++;
  }
  inline void write(kj::ArrayPtr<capnp::byte> array) { write(array.begin(), array.size()); }
  
  // Flush if needed based on write count or time
  void flush_if_needed(bool force = false) {
    double now_ms = millis_since_boot();
    bool should_flush = force || 
                       (write_count > 0 && write_count % 100 == 0) ||  // Every 100 writes
                       (now_ms - last_flush_ms > 5000);  // Every 5 seconds
    
    if (should_flush) {
      util::safe_fflush(file);
      last_flush_ms = now_ms;
    }
  }

 private:
  FILE* file = nullptr;
  uint64_t write_count;
  double last_flush_ms;
};

typedef cereal::Sentinel::SentinelType SentinelType;


class LoggerState {
public:
  LoggerState(const std::string& log_root = Path::log_root());
  ~LoggerState();
  bool next();
  void write(uint8_t* data, size_t size, bool in_qlog);
  inline int segment() const { return part; }
  inline const std::string& segmentPath() const { return segment_path; }
  inline const std::string& routeName() const { return route_name; }
  inline void write(kj::ArrayPtr<kj::byte> bytes, bool in_qlog) { write(bytes.begin(), bytes.size(), in_qlog); }
  inline void setExitSignal(int signal) { exit_signal = signal; }

protected:
  int part = -1, exit_signal = 0;
  std::string route_path, route_name, segment_path, lock_file;
  kj::Array<capnp::word> init_data;
  std::unique_ptr<RawFile> rlog, qlog;
};

kj::Array<capnp::word> logger_build_init_data();
std::string logger_get_route_name();
std::string logger_get_identifier(std::string key);
