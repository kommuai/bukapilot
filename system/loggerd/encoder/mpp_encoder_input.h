#pragma once

#include <cstdint>

#include <rk_mpi.h>

// Internal full-resolution input selection shared by the MPP adapter and its
// hardware-free behavior tests.
namespace mpp_encoder_internal {

inline void reset_import_state(MppBuffer &buffer, uint32_t &failures, bool &fallback_logged) {
  buffer = nullptr;
  failures = 0;
  fallback_logged = false;
}

template <typename ImportFn, typename PoolReadyFn, typename InitPoolFn,
          typename AcquireFn, typename CopyFn, typename SubmitFn, typename FallbackFn>
bool encode_full_resolution_frame(bool &use_zero_copy, MppBuffer &mpp_buf,
                                  uint32_t &zero_copy_import_failures, bool &import_fallback_logged,
                                  ImportFn import_fn, PoolReadyFn pool_ready_fn,
                                  InitPoolFn init_pool_fn, AcquireFn acquire_fn,
                                  CopyFn copy_fn, SubmitFn submit_fn,
                                  FallbackFn fallback_fn) {
  if (use_zero_copy) {
    mpp_buf = import_fn();
    if (mpp_buf == nullptr) {
      zero_copy_import_failures++;
      use_zero_copy = false;
      bool should_log_fallback = !import_fallback_logged;
      import_fallback_logged = true;
      fallback_fn(zero_copy_import_failures, should_log_fallback);
    } else {
      zero_copy_import_failures = 0;
    }
  }

  if (!use_zero_copy && !pool_ready_fn() && !init_pool_fn()) {
    mpp_buf = nullptr;
    return false;
  }

  if (mpp_buf == nullptr) {
    mpp_buf = acquire_fn();
    if (mpp_buf == nullptr) {
      return false;
    }
  }

  if (!use_zero_copy && !copy_fn(mpp_buf)) {
    mpp_buf = nullptr;
    return false;
  }
  if (!submit_fn(mpp_buf)) {
    mpp_buf = nullptr;
    return false;
  }
  return true;
}

}  // namespace mpp_encoder_internal
