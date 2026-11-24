#include "system/loggerd/encoder/mpp_encoder_input.h"

#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

static void require(bool ok, const char *condition, int line) {
  if (!ok) {
    std::fprintf(stderr, "check failed at line %d: %s\n", line, condition);
    std::exit(1);
  }
}

#define CHECK(condition) require((condition), #condition, __LINE__)

static void test_rejects_ordinary_files() {
  const int fd = open("/dev/null", O_RDONLY | O_CLOEXEC);
  CHECK(fd >= 0);
  CHECK(!mpp_encoder_internal::is_dma_buf_fd(fd));
  CHECK(close(fd) == 0);
  CHECK(!mpp_encoder_internal::is_dma_buf_fd(-1));
}

int main() {
  test_rejects_ordinary_files();
  std::puts("MPP encoder DMA-BUF validation tests passed");
}
