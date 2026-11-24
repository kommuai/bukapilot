#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "msgq/visionipc/visionipc_client.h"
#include "msgq/visionipc/visionipc.h"

static VisionStreamType stream_for_cam(int cam) {
  switch (cam) {
    case 0: return VISION_STREAM_WIDE_ROAD;
    case 1: return VISION_STREAM_ROAD;
    case 2: return VISION_STREAM_DRIVER;
    default: return VISION_STREAM_ROAD;
  }
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s <cam> <out.nv12> [timeout_ms]\n", argv[0]);
    return 2;
  }
  const int cam = atoi(argv[1]);
  const char *out = argv[2];
  const int timeout_ms = (argc > 3) ? atoi(argv[3]) : 5000;

  VisionIpcClient client("camerad", stream_for_cam(cam), true);
  if (!client.connect(true)) {
    fprintf(stderr, "connect failed\n");
    return 1;
  }
  VisionBuf *buf = nullptr;
  const int tries = timeout_ms / 50;
  for (int i = 0; i < tries; i++) {
    buf = client.recv();
    if (buf) break;
  }
  if (!buf) {
    fprintf(stderr, "no frame\n");
    return 1;
  }
  const int w = buf->width, h = buf->height, s = buf->stride;
  const size_t sz = (size_t)s * h + (size_t)s * h / 2;
  FILE *f = fopen(out, "wb");
  if (!f) return 1;
  fwrite(buf->addr, 1, sz, f);
  fclose(f);
  std::string meta = std::string(out) + ".meta";
  FILE *mf = fopen(meta.c_str(), "w");
  fprintf(mf, "%d %d %d\n", w, h, s);
  fclose(mf);
  printf("cam%d %dx%d stride=%d -> %s\n", cam, w, h, s, out);
  return 0;
}
