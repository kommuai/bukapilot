#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include "msgq/visionipc/visionipc_client.h"
#include "msgq/visionipc/visionipc.h"

static int measure_pair(VisionIpcClient &wide, VisionIpcClient &road, int samples) {
  std::vector<int64_t> ts_delta_ns;
  std::vector<int64_t> fid_delta;
  ts_delta_ns.reserve(samples);
  fid_delta.reserve(samples);

  for (int i = 0; i < samples; i++) {
    VisionIpcBufExtra wextra = {}, rextra = {};
    VisionBuf *wb = wide.recv(&wextra, 2000);
    VisionBuf *rb = road.recv(&rextra, 2000);
    if (!wb || !rb || !wextra.valid || !rextra.valid) {
      fprintf(stderr, "recv failed at sample %d\n", i);
      return 1;
    }
    int64_t dts = (int64_t)wextra.timestamp_sof - (int64_t)rextra.timestamp_sof;
    int64_t dfid = (int64_t)wextra.frame_id - (int64_t)rextra.frame_id;
    ts_delta_ns.push_back(dts);
    fid_delta.push_back(dfid);
  }

  auto stats = [](const std::vector<int64_t> &v, int64_t &mn, int64_t &mx, double &mean, double &stdev) {
    mn = mx = v[0];
    long double sum = 0;
    for (auto x : v) {
      mn = std::min(mn, x);
      mx = std::max(mx, x);
      sum += x;
    }
    mean = (double)(sum / v.size());
    long double var = 0;
    for (auto x : v) {
      long double d = x - mean;
      var += d * d;
    }
    stdev = (double)std::sqrt(var / v.size());
  };

  int64_t mn, mx;
  double mean, stdev;
  stats(ts_delta_ns, mn, mx, mean, stdev);
  printf("wide-road ts_delta_ns: min=%" PRId64 " max=%" PRId64 " mean=%.0f stdev=%.0f\n", mn, mx, mean, stdev);
  const bool ts_jitter_ok = stdev < 500000.0;
  stats(fid_delta, mn, mx, mean, stdev);
  printf("wide-road frame_id_delta: min=%" PRId64 " max=%" PRId64 " mean=%.2f stdev=%.2f\n", mn, mx, mean, stdev);
  const bool fid_stable = stdev < 0.5;
  return (ts_jitter_ok && fid_stable) ? 0 : 2;
}

int main(int argc, char **argv) {
  const int samples = (argc > 1) ? atoi(argv[1]) : 30;
  VisionIpcClient wide("camerad", VISION_STREAM_WIDE_ROAD, true);
  VisionIpcClient road("camerad", VISION_STREAM_ROAD, true);
  if (!wide.connect(true) || !road.connect(true)) {
    fprintf(stderr, "connect failed\n");
    return 1;
  }
  return measure_pair(wide, road, samples);
}
