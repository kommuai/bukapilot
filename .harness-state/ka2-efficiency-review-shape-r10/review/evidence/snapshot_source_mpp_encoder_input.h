#pragma once

#include <cerrno>
#include <sys/ioctl.h>

#include <linux/dma-buf.h>

namespace mpp_encoder_internal {

inline bool is_dma_buf_fd(int fd) {
  if (fd < 0) return false;

  dma_buf_sync sync = {.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
  int ret;
  do {
    ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  } while (ret < 0 && errno == EINTR);
  if (ret < 0) return false;

  sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
  do {
    ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  } while (ret < 0 && errno == EINTR);
  return ret == 0;
}

}  // namespace mpp_encoder_internal
