#define __STDC_CONSTANT_MACROS

#include "system/loggerd/encoder/mpp_encoder_input.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

struct MppEncoderTestState {
  bool use_zero_copy = false;
  MppBuffer buffer = nullptr;
  uint32_t import_failures = 0;
  bool fallback_logged = false;
};

struct MppEncoderTestAccess {
  template <typename ImportFn, typename PoolReadyFn, typename InitPoolFn,
            typename AcquireFn, typename CopyFn, typename SubmitFn, typename FallbackFn>
  static bool encode(MppEncoderTestState &state, ImportFn import_fn,
                     PoolReadyFn pool_ready_fn, InitPoolFn init_pool_fn,
                     AcquireFn acquire_fn, CopyFn copy_fn, SubmitFn submit_fn,
                     FallbackFn fallback_fn) {
    return mpp_encoder_internal::encode_full_resolution_frame(
      state.use_zero_copy, state.buffer, state.import_failures, state.fallback_logged,
      import_fn, pool_ready_fn, init_pool_fn, acquire_fn, copy_fn, submit_fn, fallback_fn);
  }

  static void reset(MppEncoderTestState &state) {
    mpp_encoder_internal::reset_import_state(state.buffer, state.import_failures, state.fallback_logged);
  }
};

static void require(bool condition, const char *expression, int line) {
  if (!condition) {
    std::fprintf(stderr, "check failed at line %d: %s\n", line, expression);
    std::exit(1);
  }
}

#define REQUIRE(condition) require((condition), #condition, __LINE__)

static uint8_t buffer_tokens[256] = {};

static MppBuffer buffer_token(uintptr_t index) {
  return &buffer_tokens[index];
}

static void test_first_import_failure_copies_and_submits_same_frame() {
  MppEncoderTestState state{true, nullptr, 0, false};
  const MppBuffer copy_buffer = buffer_token(1);
  int frame_id = 41;
  int copied_frame_id = -1;
  int import_calls = 0;
  int pool_ready_calls = 0;
  int pool_init_calls = 0;
  int acquire_calls = 0;
  int copy_calls = 0;
  int submit_calls = 0;
  int fallback_calls = 0;
  std::vector<std::string> order;

  const bool submitted = MppEncoderTestAccess::encode(
    state,
    [&]() -> MppBuffer {
      order.push_back("import");
      ++import_calls;
      return nullptr;
    },
    [&]() {
      order.push_back("pool-ready");
      ++pool_ready_calls;
      return false;
    },
    [&]() {
      order.push_back("pool-init");
      ++pool_init_calls;
      return true;
    },
    [&]() {
      order.push_back("acquire");
      ++acquire_calls;
      return copy_buffer;
    },
    [&](MppBuffer buffer) {
      order.push_back("copy");
      REQUIRE(buffer == copy_buffer);
      ++copy_calls;
      copied_frame_id = frame_id;
      return true;
    },
    [&](MppBuffer buffer) {
      order.push_back("submit");
      REQUIRE(buffer == copy_buffer);
      REQUIRE(copied_frame_id == frame_id);
      ++submit_calls;
      return true;
    },
    [&](uint32_t failures, bool should_log) {
      order.push_back("fallback");
      REQUIRE(failures == 1);
      REQUIRE(should_log);
      ++fallback_calls;
    });

  REQUIRE(submitted);
  REQUIRE(!state.use_zero_copy);
  REQUIRE(state.import_failures == 1);
  REQUIRE(state.fallback_logged);
  REQUIRE(import_calls == 1);
  REQUIRE(pool_ready_calls == 1);
  REQUIRE(pool_init_calls == 1);
  REQUIRE(acquire_calls == 1);
  REQUIRE(copy_calls == 1);
  REQUIRE(submit_calls == 1);
  REQUIRE(fallback_calls == 1);
  REQUIRE(order == std::vector<std::string>({"import", "fallback", "pool-ready",
                                            "pool-init", "acquire", "copy", "submit"}));
}

static void test_latch_survives_close_open_state_reset() {
  MppEncoderTestState state{true, nullptr, 0, false};
  int initial_import_calls = 0;
  int initial_copy_calls = 0;
  int initial_submit_calls = 0;

  const bool fallback_submitted = MppEncoderTestAccess::encode(
    state,
    [&]() -> MppBuffer {
      ++initial_import_calls;
      return nullptr;
    },
    []() { return true; },
    []() {
      REQUIRE(false);
      return false;
    },
    []() { return buffer_token(2); },
    [&](MppBuffer) {
      ++initial_copy_calls;
      return true;
    },
    [&](MppBuffer) {
      ++initial_submit_calls;
      return true;
    },
    [](uint32_t failures, bool should_log) {
      REQUIRE(failures == 1);
      REQUIRE(should_log);
    });

  REQUIRE(fallback_submitted);
  REQUIRE(!state.use_zero_copy);
  REQUIRE(initial_import_calls == 1);
  REQUIRE(initial_copy_calls == 1);
  REQUIRE(initial_submit_calls == 1);

  // Production calls this same helper from both encoder_close() and encoder_open().
  MppEncoderTestAccess::reset(state);
  MppEncoderTestAccess::reset(state);
  REQUIRE(!state.use_zero_copy);
  REQUIRE(state.buffer == nullptr);
  REQUIRE(state.import_failures == 0);
  REQUIRE(!state.fallback_logged);

  const MppBuffer second_frame_buffer = buffer_token(3);
  int frame_id = 42;
  int copied_frame_id = -1;
  int import_calls = 0;
  int copy_calls = 0;
  int submit_calls = 0;

  const bool submitted = MppEncoderTestAccess::encode(
    state,
    [&]() -> MppBuffer {
      ++import_calls;
      return buffer_token(99);
    },
    []() { return true; },
    []() {
      REQUIRE(false);
      return false;
    },
    [&]() { return second_frame_buffer; },
    [&](MppBuffer buffer) {
      REQUIRE(buffer == second_frame_buffer);
      ++copy_calls;
      copied_frame_id = frame_id;
      return true;
    },
    [&](MppBuffer buffer) {
      REQUIRE(buffer == second_frame_buffer);
      REQUIRE(copied_frame_id == frame_id);
      ++submit_calls;
      return true;
    },
    [](uint32_t, bool) { REQUIRE(false); });

  REQUIRE(submitted);
  REQUIRE(!state.use_zero_copy);
  REQUIRE(import_calls == 0);
  REQUIRE(copy_calls == 1);
  REQUIRE(submit_calls == 1);
}

static void test_pool_allocation_failure_does_not_submit() {
  MppEncoderTestState state{true, nullptr, 0, false};
  int import_calls = 0;
  int pool_init_calls = 0;
  int acquire_calls = 0;
  int copy_calls = 0;
  int submit_calls = 0;

  const bool submitted = MppEncoderTestAccess::encode(
    state,
    [&]() -> MppBuffer {
      ++import_calls;
      return nullptr;
    },
    []() { return false; },
    [&]() {
      ++pool_init_calls;
      return false;
    },
    [&]() {
      ++acquire_calls;
      return buffer_token(4);
    },
    [&](MppBuffer) {
      ++copy_calls;
      return true;
    },
    [&](MppBuffer) {
      ++submit_calls;
      return true;
    },
    [](uint32_t failures, bool should_log) {
      REQUIRE(failures == 1);
      REQUIRE(should_log);
    });

  REQUIRE(!submitted);
  REQUIRE(!state.use_zero_copy);
  REQUIRE(state.buffer == nullptr);
  REQUIRE(import_calls == 1);
  REQUIRE(pool_init_calls == 1);
  REQUIRE(acquire_calls == 0);
  REQUIRE(copy_calls == 0);
  REQUIRE(submit_calls == 0);
}

static void test_copy_failure_does_not_submit() {
  MppEncoderTestState state{true, nullptr, 0, false};
  int copy_calls = 0;
  int submit_calls = 0;

  const bool submitted = MppEncoderTestAccess::encode(
    state,
    []() -> MppBuffer { return nullptr; },
    []() { return true; },
    []() {
      REQUIRE(false);
      return false;
    },
    []() { return buffer_token(5); },
    [&](MppBuffer) {
      ++copy_calls;
      return false;
    },
    [&](MppBuffer) {
      ++submit_calls;
      return true;
    },
    [](uint32_t failures, bool should_log) {
      REQUIRE(failures == 1);
      REQUIRE(should_log);
    });

  REQUIRE(!submitted);
  REQUIRE(!state.use_zero_copy);
  REQUIRE(state.buffer == nullptr);
  REQUIRE(copy_calls == 1);
  REQUIRE(submit_calls == 0);
}

static void test_successful_import_submits_without_copy() {
  MppEncoderTestState state{true, nullptr, 4, false};
  const MppBuffer imported_buffer = buffer_token(6);
  int import_calls = 0;
  int pool_ready_calls = 0;
  int submit_calls = 0;

  const bool submitted = MppEncoderTestAccess::encode(
    state,
    [&]() {
      ++import_calls;
      return imported_buffer;
    },
    [&]() {
      ++pool_ready_calls;
      return false;
    },
    []() {
      REQUIRE(false);
      return false;
    },
    []() {
      REQUIRE(false);
      return buffer_token(7);
    },
    [](MppBuffer) {
      REQUIRE(false);
      return false;
    },
    [&](MppBuffer buffer) {
      REQUIRE(buffer == imported_buffer);
      ++submit_calls;
      return true;
    },
    [](uint32_t, bool) { REQUIRE(false); });

  REQUIRE(submitted);
  REQUIRE(state.use_zero_copy);
  REQUIRE(state.buffer == imported_buffer);
  REQUIRE(state.import_failures == 0);
  REQUIRE(import_calls == 1);
  REQUIRE(pool_ready_calls == 0);
  REQUIRE(submit_calls == 1);
}

int main() {
  test_first_import_failure_copies_and_submits_same_frame();
  test_latch_survives_close_open_state_reset();
  test_pool_allocation_failure_does_not_submit();
  test_copy_failure_does_not_submit();
  test_successful_import_submits_without_copy();
  std::puts("MPP fallback helper behavior checks passed");
  return 0;
}
