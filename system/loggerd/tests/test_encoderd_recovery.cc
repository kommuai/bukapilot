#define main encoderd_program_main
#include "system/loggerd/encoderd.cc"
#undef main

#include <cstring>
#include <array>
#include <fcntl.h>
#include <iostream>
#include <linux/memfd.h>
#include <rga/rga.h>
#include <rga/im2d.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

static void check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

struct ScopedFd {
  int value = -1;
  ~ScopedFd() { if (value >= 0) close(value); }
};

struct ScopedMapping {
  void *address = MAP_FAILED;
  size_t size = 0;
  ~ScopedMapping() { if (address != MAP_FAILED) munmap(address, size); }
};

static void test_strict_authority_parse() {
  int value = -1;
  check(parse_nonnegative_int_strict("0", &value) && value == 0, "zero authority should parse");
  check(parse_nonnegative_int_strict("123", &value) && value == 123, "decimal authority should parse");
  for (const std::string bad : {"", "-1", "+1", "1junk", " 1", "2147483648"}) {
    check(!parse_nonnegative_int_strict(bad, &value), "malformed or negative authority must be rejected");
  }
}

static void test_reopen_counter_deltas() {
  MppEncoder qroad(qcam_encoder_info, 1920, 1200);
  check(qroad.segment_transition_reopen_count() == 0 &&
        qroad.failure_recovery_reopen_count() == 0 && qroad.failure_retry_count() == 0,
        "qRoad reopen counters must start at zero");
  qroad.note_segment_transition_reopen();
  check(qroad.segment_transition_reopen_count() == 1 &&
        qroad.failure_recovery_reopen_count() == 0 && qroad.failure_retry_count() == 0,
        "a planned qRoad segment reopen must not count as failure recovery or retry");
  qroad.note_failure_retry_started();
  qroad.note_failure_recovery_reopen();
  check(qroad.segment_transition_reopen_count() == 1 &&
        qroad.failure_recovery_reopen_count() == 1 && qroad.failure_retry_count() == 1,
        "qRoad retry and failure-recovery counters must remain distinct from planned transitions");
}

using RecoveryTimePoint = EncoderOutputRecovery::TimePoint;

static void test_packet_progress_watchdog_policy() {
  const RecoveryTimePoint started = RecoveryTimePoint{} + std::chrono::seconds(10);
  EncoderOutputRecovery qroad;
  qroad.reset_after_reopen(true, true, started, started, 0, 0);
  check(qroad.observe_submitted(1, started), "first qRoad submit must start the no-progress interval");
  qroad.observe_pending(1, started);

  const auto grace_ends = started + std::chrono::seconds(3);
  check(!qroad.packet_watchdog_expired(grace_ends - std::chrono::milliseconds(1), false),
        "qRoad must retain its full three-second startup grace");
  check(!qroad.packet_watchdog_expired(grace_ends, false),
        "qRoad must not expire at the end of its startup grace before the one-second stall window");
  check(!qroad.packet_watchdog_expired(grace_ends + std::chrono::milliseconds(999), false),
        "qRoad watchdog must not expire before the exact one-second no-packet window");
  check(!qroad.packet_watchdog_expired(grace_ends + std::chrono::seconds(1), true),
        "rotation barrier must suppress the packet watchdog at expiry");
  check(qroad.packet_watchdog_expired(grace_ends + std::chrono::seconds(1), false),
        "qRoad watchdog must expire exactly at grace plus the one-second stall timeout");

  check(qroad.observe_published(1, grace_ends + std::chrono::seconds(1)),
        "a published packet must count as actual progress");
  const auto next_submit = grace_ends + std::chrono::seconds(1);
  check(qroad.observe_submitted(2, next_submit), "a later submitted frame must restart stall tracking after packet progress");
  qroad.observe_pending(1, next_submit);
  check(!qroad.packet_watchdog_expired(next_submit + std::chrono::milliseconds(999), false),
        "a packet must restart the no-progress interval");
  check(qroad.packet_watchdog_expired(next_submit + std::chrono::seconds(1), false),
        "the restarted one-second stall must expire at its exact boundary");
}

static void test_error_locality_retry_backoff_and_progress_reset() {
  const RecoveryTimePoint started = RecoveryTimePoint{} + std::chrono::seconds(20);
  EncoderOutputRecovery faulty;
  EncoderOutputRecovery healthy;
  for (int i = 0; i < 9; ++i) faulty.note_input_error();
  check(!faulty.input_error_limit_reached(10), "nine consecutive errors must stay below the reopen threshold");
  check(faulty.consecutive_input_errors == 9 && healthy.consecutive_input_errors == 0 && healthy.active(),
        "input error accounting must stay local to the faulty output");
  faulty.note_input_error();
  check(faulty.input_error_limit_reached(10), "the tenth consecutive error must reach the local reopen threshold");
  check(healthy.active() && healthy.consecutive_input_errors == 0,
        "ten errors on one output must not recover, retry, or count errors on its sibling");

  const std::array<int, 7> expected_delays = {1, 2, 4, 8, 16, 30, 30};
  RecoveryTimePoint scheduled_at = started;
  for (size_t i = 0; i < expected_delays.size(); ++i) {
    const int delay = faulty.schedule_retry(scheduled_at);
    check(delay == expected_delays[i], "retry delay must follow 1/2/4/8/16 then capped 30 seconds");
    check(faulty.state == EncoderOutputState::Unavailable,
          "failed output must stay unavailable while waiting for its retry deadline");
    const auto due_at = scheduled_at + std::chrono::seconds(delay);
    check(!faulty.begin_due_retry(due_at - std::chrono::nanoseconds(1)),
          "retry policy must not reopen before its scheduled deadline");
    check(faulty.begin_due_retry(due_at), "retry policy must become due at its exact deadline");
    check(faulty.state == EncoderOutputState::Recovering,
          "only the failed output should enter its retry transition");
    check(healthy.active() && healthy.retry_backoff_level == 0,
          "one output's retry schedule must not change a healthy sibling");
    scheduled_at = due_at;
  }

  faulty.retry_opened(started, true, 0, 0);
  check(faulty.awaiting_progress_reset, "successful reopen must wait for actual packet progress before clearing backoff");
  check(faulty.observe_published(1, started + std::chrono::milliseconds(500)),
        "the first valid packet after reopen must start the progress-reset window");
  check(faulty.retry_backoff_level == 6 && faulty.awaiting_progress_reset,
        "one packet alone must not immediately clear the retry backoff");
  check(faulty.observe_published(2, started + std::chrono::milliseconds(1499)),
        "additional packet progress before one second must be observed");
  check(faulty.retry_backoff_level == 6 && faulty.awaiting_progress_reset,
        "backoff must remain until a full second of valid packet progress");
  check(faulty.observe_published(3, started + std::chrono::milliseconds(1500)),
        "packet progress at the one-second boundary must be observed");
  check(faulty.retry_backoff_level == 0 && !faulty.awaiting_progress_reset,
        "one second of valid packet progress must reset the retry backoff");
  check(healthy.active() && healthy.retry_backoff_level == 0,
        "progress reset on the recovered output must leave the healthy sibling unchanged");
}

static void allocate_qroad_shm_input(ScopedFd &fd, ScopedMapping &mapping, VisionBuf &buffer) {
  constexpr int width = 1920;
  constexpr int height = 1200;
  constexpr size_t size = width * height * 3 / 2;
  fd.value = static_cast<int>(syscall(SYS_memfd_create, "loggerd-test-retry-qroad", MFD_CLOEXEC));
  check(fd.value >= 0, "memfd_create retry qRoad shared-memory input");
  check(ftruncate(fd.value, size) == 0, "ftruncate retry qRoad shared-memory input");
  mapping.address = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd.value, 0);
  mapping.size = size;
  check(mapping.address != MAP_FAILED, "mmap retry qRoad shared-memory input");
  std::memset(mapping.address, 0x80, size);
  buffer.width = width;
  buffer.height = height;
  buffer.fd = fd.value;
  buffer.addr = mapping.address;
  buffer.len = size;
  buffer.mmap_len = size;
}

static void test_watchdog_quarantine_retries_without_identity_shift() {
  const RecoveryTimePoint started = RecoveryTimePoint{} + std::chrono::seconds(30);
  const auto watchdog_due = started + std::chrono::seconds(4);
  EncoderOutputRecovery health;
  MppEncoder qroad(qcam_encoder_info, 1920, 1200);
  SubMaster subscriber({"qRoadEncodeData"});
  qroad.encoder_open();
  check(qroad.encoder_is_open(), "qRoad MPP must open for watchdog retry integration");
  qroad.set_segment_num(0);
  health.reset_after_reopen(true, true, started, started,
                            qroad.packet_published_count(), qroad.input_submitted_count());

  VisionBuf buffer;
  ScopedFd shm_fd;
  ScopedMapping shm_mapping;
  allocate_qroad_shm_input(shm_fd, shm_mapping, buffer);
  VisionIpcBufExtra silent_extra = {.frame_id = 41, .timestamp_sof = 1000, .timestamp_eof = 2000, .valid = true};
  check(qroad.encode_frame(&buffer, &silent_extra) == 1,
        "test silent fault must accept the first frame into the pending queue");
  check(qroad.pending_frame_ids() == std::vector<uint32_t>{41},
        "pending frame 41 must remain bound to its exact identity before watchdog expiry");
  check(health.observe_submitted(qroad.input_submitted_count(), started),
        "qRoad submitted count must start watchdog tracking");
  health.observe_pending(qroad.pending_frame_count(), started);
  check(!health.packet_watchdog_expired(watchdog_due - std::chrono::nanoseconds(1), false),
        "the watchdog must not quarantine the pending frame early");
  check(health.packet_watchdog_expired(watchdog_due, false),
        "the watchdog must expire at the exact qRoad grace and stall boundary");

  const int first_delay = schedule_output_retry_policy(
    health, qroad, [&] { return watchdog_due; }, true, "road", "qRoadEncodeData", "packet progress watchdog");
  check(first_delay == 1 && health.state == EncoderOutputState::Unavailable,
        "watchdog quarantine must schedule the first retry without a tight loop");
  check(qroad.pending_frame_ids().empty() && qroad.pending_drop_count() == 1,
        "quarantine must retire pending frame 41 exactly once");
  qroad.encoder_close();
  check(qroad.pending_drop_count() == 1,
        "closing an already quarantined output must not retire the same identity twice");

  int encode_failures = 10;
  int encode_failure_burst = 10;
  const auto retry_at = watchdog_due + std::chrono::seconds(1);
  check(!retry_output_policy(health, qroad, retry_at - std::chrono::nanoseconds(1),
                             [&] { return retry_at; }, 7, true, "road", "qRoadEncodeData",
                             encode_failures, encode_failure_burst),
        "a quarantined output must not retry before its scheduled deadline");
  check(qroad.failure_retry_count() == 0 && !qroad.encoder_is_open(),
        "early retry check must leave the encoder closed and retry count unchanged");
  check(retry_output_policy(health, qroad, retry_at, [&] { return retry_at; },
                            7, true, "road", "qRoadEncodeData", encode_failures, encode_failure_burst),
        "the output retry must run at its due deadline");
  check(qroad.encoder_is_open() && qroad.failure_retry_count() == 1 &&
        qroad.failure_recovery_reopen_count() == 1,
        "a due recovery must reopen only this qRoad output once");
  check(encode_failures == 0 && encode_failure_burst == 0,
        "a successful reopen must clear this output's consecutive error counters");

  unsetenv("LOGGERD_TEST_MPP_FAULT_STREAM");
  unsetenv("LOGGERD_TEST_MPP_FAULT_MODE");
  uint32_t next_frame_id = 42;
  bool saw_first_packet = false;
  for (int attempt = 0; attempt < 20 && !saw_first_packet; ++attempt) {
    const uint32_t frame_id = next_frame_id++;
    VisionIpcBufExtra extra = {
      .frame_id = frame_id,
      .timestamp_sof = static_cast<uint64_t>(frame_id) * 33333333ULL,
      .timestamp_eof = static_cast<uint64_t>(frame_id) * 33333333ULL + 1000000ULL,
      .valid = true,
    };
    check(qroad.encode_frame(&buffer, &extra) >= 0,
          "valid qRoad input after retry must not inherit the retired frame identity");
    for (int poll = 0; poll < 10; ++poll) {
      subscriber.update(20);
      if (!subscriber.updated("qRoadEncodeData")) continue;
      const auto index = subscriber["qRoadEncodeData"].getQRoadEncodeData().getIdx();
      check(index.getSegmentNum() == 7,
            "first valid packet after recovery must publish on the current authoritative segment");
      check(index.getFrameId() == 42,
            "retired frame 41 must not shift or be reassigned to the first post-retry packet");
      saw_first_packet = true;
      break;
    }
  }
  check(saw_first_packet, "qRoad must publish a valid post-retry packet on segment 7");

  const auto after_first_progress = retry_at + std::chrono::milliseconds(500);
  const uint64_t first_packet_count = qroad.packet_published_count();
  check(first_packet_count > 0 && health.observe_published(first_packet_count, after_first_progress),
        "actual MPP publication must feed the post-retry progress policy");
  check(health.awaiting_progress_reset && health.retry_backoff_level != 0,
        "a single valid post-retry packet must not clear retry backoff immediately");

  bool saw_second_packet = false;
  for (int attempt = 0; attempt < 20 && !saw_second_packet; ++attempt) {
    const uint32_t frame_id = next_frame_id++;
    VisionIpcBufExtra extra = {
      .frame_id = frame_id,
      .timestamp_sof = static_cast<uint64_t>(frame_id) * 33333333ULL,
      .timestamp_eof = static_cast<uint64_t>(frame_id) * 33333333ULL + 1000000ULL,
      .valid = true,
    };
    check(qroad.encode_frame(&buffer, &extra) >= 0, "second valid qRoad frame must be accepted after retry");
    for (int poll = 0; poll < 10; ++poll) {
      subscriber.update(20);
      if (!subscriber.updated("qRoadEncodeData")) continue;
      const auto index = subscriber["qRoadEncodeData"].getQRoadEncodeData().getIdx();
      check(index.getSegmentNum() == 7, "continued qRoad progress must remain on the current segment");
      check(index.getFrameId() == 43, "the next packet must preserve the next frame identity");
      saw_second_packet = true;
      break;
    }
  }
  check(saw_second_packet && qroad.packet_published_count() > first_packet_count,
        "a second real qRoad packet must arrive after the first without ID shifting");
  check(health.observe_published(qroad.packet_published_count(), after_first_progress + std::chrono::seconds(1)),
        "second observed packet at one second must complete the reset window");
  check(!health.awaiting_progress_reset && health.retry_backoff_level == 0,
        "sustained packet progress must reset the retry backoff after one second");
  qroad.encoder_close();
}

static void test_drain_timeout_quarantines_only_failed_output() {
  const RecoveryTimePoint started = RecoveryTimePoint{} + std::chrono::seconds(40);
  EncoderOutputRecovery road_health;
  EncoderOutputRecovery qroad_health;
  MppEncoder road(main_road_encoder_info, 1920, 1200);
  MppEncoder qroad(qcam_encoder_info, 1920, 1200);
  road.encoder_open();
  qroad.encoder_open();
  check(road.encoder_is_open() && qroad.encoder_is_open(),
        "both MPP outputs must open before testing shared drain isolation");
  road_health.reset_after_reopen(true, false, started, RecoveryTimePoint{}, 0, 0);
  qroad_health.reset_after_reopen(true, true, started, started, 0, 0);

  VisionIpcBufExtra blocked = {.frame_id = 80, .timestamp_sof = 1000, .timestamp_eof = 2000, .valid = true};
  check(road.inject_test_drain_timeout(blocked), "road output must accept the drain-timeout identity once");
  check(road.pending_frame_ids() == std::vector<uint32_t>{80},
        "timed-out road output must retain its exact frame ID before quarantine");
  road_health.observe_pending(road.pending_frame_count(), started);
  qroad_health.observe_pending(qroad.pending_frame_count(), started);
  check(road_health.should_quarantine_after_drain(road.pending_frame_count()),
        "an active output with a pending identity must be quarantined after shared drain expiry");
  check(!qroad_health.should_quarantine_after_drain(qroad.pending_frame_count()),
        "a healthy qRoad output without pending identities must not be quarantined by its sibling");

  const auto delay = schedule_output_retry_policy(
    road_health, road, [&] { return started + std::chrono::milliseconds(900); }, true,
    "road", "roadEncodeData", "segment drain deadline");
  check(delay == 1 && road_health.state == EncoderOutputState::Unavailable,
        "the failed road output must receive its own retry schedule at drain expiry");
  check(road.pending_frame_ids().empty() && road.pending_drop_count() == 1,
        "drain quarantine must retire the failed output's pending ID exactly once");
  check(road.pending_drop_count() == 1,
        "the shared drain cleanup must not double-retire the pending identity");
  check(qroad.encoder_is_open() && qroad_health.active() &&
        qroad_health.retry_backoff_level == 0 && qroad.failure_retry_count() == 0 &&
        qroad.failure_recovery_reopen_count() == 0,
        "road drain expiry must leave healthy qRoad active without sibling-triggered retry/recovery");

  qroad.encoder_close();
  qroad.encoder_open();
  qroad.set_segment_num(1);
  qroad.note_segment_transition_reopen();
  qroad_health.planned_segment_opened(started + std::chrono::milliseconds(900), true,
                                      qroad.packet_published_count(), qroad.input_submitted_count());
  check(qroad.encoder_is_open() && qroad_health.active(),
        "healthy qRoad must complete its planned segment transition after the sibling is quarantined");
  check(qroad.segment_transition_reopen_count() == 1 &&
        qroad.failure_recovery_reopen_count() == 0 && qroad.failure_retry_count() == 0,
        "qRoad boundary must count one planned reopen and zero failure reopen/retry events");
  road.set_flush_pending_on_close(false);
  road.encoder_close();
  qroad.encoder_close();
}

static void test_mpp_fault_mode(const std::string &mode) {
  MppEncoder encoder(main_road_encoder_info, 1920, 1200);
  encoder.encoder_open();
  check(encoder.encoder_is_open(), "MPP test encoder must open on KA2");
  VisionBuf buffer;
  buffer.width = 1920;
  buffer.height = 1200;
  buffer.fd = -1;
  VisionIpcBufExtra extra = {.frame_id = 41, .timestamp_sof = 1000, .timestamp_eof = 2000, .valid = true};

  if (mode == "reject") {
    check(encoder.encode_frame(&buffer, &extra) == -1, "reject seam must return an input error");
    check(encoder.rejected_count() == 1 && encoder.input_submitted_count() == 0,
          "reject seam must not count a submitted input");
    check(encoder.pending_frame_ids().empty(), "rejected input must not leave a pending identity");
  } else if (mode == "silent") {
    check(encoder.encode_frame(&buffer, &extra) == 1, "silent seam accepts the input");
    check(encoder.input_submitted_count() == 1 && encoder.packet_published_count() == 0,
          "silent input submission must not count as packet progress");
    const auto ids = encoder.pending_frame_ids();
    check(ids.size() == 1 && ids.front() == extra.frame_id,
          "silent output must retain the exact pending frame identity");
  } else if (mode == "drain_timeout") {
    check(encoder.inject_test_drain_timeout(extra), "drain timeout seam must arm once");
    const auto ids = encoder.pending_frame_ids();
    check(ids.size() == 1 && ids.front() == extra.frame_id,
          "drain timeout must retain the exact pending identity");
    check(!encoder.flush_pending_packets(1, false), "drain timeout must remain pending");
  } else if (mode == "invalid_fd") {
    ScopedFd fd{open("/dev/null", O_RDONLY | O_CLOEXEC)};
    check(fd.value >= 0, "open /dev/null");
    buffer.fd = fd.value;
    check(encoder.encode_frame(&buffer, &extra) == -1, "ordinary file descriptor must be rejected");
    check(encoder.rejected_count() == 1 && encoder.input_submitted_count() == 0,
          "invalid DMA-BUF must be rejected before input submission");
    check(encoder.pending_frame_ids().empty(), "invalid DMA-BUF must not retain a frame identity");
  } else if (mode == "shm") {
    constexpr size_t shm_size = 1920 * 1200 * 3 / 2;
    ScopedFd fd{static_cast<int>(syscall(SYS_memfd_create, "loggerd-test-shm-nv12", MFD_CLOEXEC))};
    check(fd.value >= 0, "memfd_create shared-memory input");
    check(ftruncate(fd.value, shm_size) == 0, "ftruncate shared-memory input");
    ScopedMapping mapping{mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd.value, 0), shm_size};
    check(mapping.address != MAP_FAILED, "mmap shared-memory input");
    std::memset(mapping.address, 0x80, shm_size);
    buffer.fd = fd.value;
    buffer.addr = mapping.address;
    buffer.len = shm_size;
    buffer.mmap_len = shm_size;
    check(encoder.encode_frame(&buffer, &extra) == -1,
          "full-resolution MPP must reject a memfd-backed non-DMA-BUF NV12 input without a copy route");
    check(encoder.rejected_count() == 1 && encoder.input_submitted_count() == 0,
          "one shared-memory DMA-BUF validation rejection must happen before any MPP input submission");
    check(encoder.pending_frame_ids().empty(), "shared-memory rejection must not retain a frame identity");
  } else {
    throw std::runtime_error("unknown MPP fault mode");
  }

  check(encoder.packet_published_count() == 0, "fault modes must not fabricate published packets");
  encoder.set_flush_pending_on_close(false);
  encoder.encoder_close();
}

static void test_shm_qroad_rga() {
  constexpr int input_width = 1920;
  constexpr int input_height = 1200;
  constexpr size_t input_size = input_width * input_height * 3 / 2;
  ScopedFd fd{static_cast<int>(syscall(SYS_memfd_create, "loggerd-test-shm-qroad-rga", MFD_CLOEXEC))};
  check(fd.value >= 0, "memfd_create qRoad RGA shared-memory input");
  check(ftruncate(fd.value, input_size) == 0, "ftruncate qRoad RGA shared-memory input");
  ScopedMapping input{mmap(nullptr, input_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd.value, 0), input_size};
  check(input.address != MAP_FAILED, "mmap qRoad RGA shared-memory input");
  std::memset(input.address, 0x80, input_size);

  const int output_width = qcam_encoder_info.frame_width;
  const int output_height = qcam_encoder_info.frame_height;
  const int output_stride = (output_width + 15) & ~15;
  const int output_vstride = (output_height + 15) & ~15;
  const size_t output_size = output_stride * output_vstride * 3 / 2;
  ScopedMapping output{mmap(nullptr, output_size, PROT_READ | PROT_WRITE,
                            MAP_PRIVATE | MAP_ANONYMOUS, -1, 0), output_size};
  check(output.address != MAP_FAILED, "mmap qRoad RGA output buffer");

  const auto src = wrapbuffer_virtualaddr(input.address, input_width, input_height, RK_FORMAT_YCbCr_420_SP);
  const auto dst = wrapbuffer_virtualaddr(output.address, output_stride, output_vstride, RK_FORMAT_YCbCr_420_SP);
  const int rga_status = imresize(src, dst,
                                  static_cast<double>(output_width) / input_width,
                                  static_cast<double>(output_height) / input_height, IM_SYNC);
  std::cout << "RGA_SHM_API_" << (rga_status >= 0 ? "ACCEPTED" : "REJECTED")
            << " status=" << rga_status << " detail=" << imStrError(static_cast<IM_STATUS>(rga_status)) << '\n';
  if (rga_status < 0) return;

  MppEncoder qroad(qcam_encoder_info, input_width, input_height);
  qroad.encoder_open();
  if (!qroad.encoder_is_open()) {
    std::cout << "QROAD_SHM_MPP_OPEN_FAILED api_only=1\n";
    return;
  }
  qroad.set_segment_num(0);
  VisionBuf buffer;
  buffer.width = input_width;
  buffer.height = input_height;
  buffer.fd = fd.value;
  buffer.addr = input.address;
  buffer.len = input_size;
  buffer.mmap_len = input_size;

  int encode_error = 0;
  for (uint32_t frame_id = 1; frame_id <= 12; ++frame_id) {
    VisionIpcBufExtra extra = {
      .frame_id = frame_id,
      .timestamp_sof = static_cast<uint64_t>(frame_id) * 33333333ULL,
      .timestamp_eof = static_cast<uint64_t>(frame_id) * 33333333ULL + 1000000ULL,
      .valid = true,
    };
    const int result = qroad.encode_frame(&buffer, &extra);
    if (result < 0) {
      encode_error = result;
      break;
    }
  }
  qroad.encoder_close();
  const uint64_t published = qroad.packet_published_count();
  std::cout << "QROAD_SHM_MPP submitted=" << qroad.input_submitted_count()
            << " published=" << published
            << " rejected=" << qroad.rejected_count()
            << " empty=" << qroad.empty_packet_count()
            << " encode_error=" << encode_error
            << " result=" << (published > 0 ? "PACKET" : "NO_PACKET") << '\n';
}

int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("expected one mode: parse|counter_contract|policy_state|watchdog_retry|drain_quarantine|reject|silent|drain_timeout|invalid_fd|shm|shm_rga");
    const std::string mode = argv[1];
    if (mode == "parse") test_strict_authority_parse();
    else if (mode == "counter_contract") test_reopen_counter_deltas();
    else if (mode == "policy_state") {
      test_packet_progress_watchdog_policy();
      test_error_locality_retry_backoff_and_progress_reset();
    }
    else if (mode == "watchdog_retry") test_watchdog_quarantine_retries_without_identity_shift();
    else if (mode == "drain_quarantine") test_drain_timeout_quarantines_only_failed_output();
    else if (mode == "shm_rga") test_shm_qroad_rga();
    else test_mpp_fault_mode(mode);
    std::cout << (mode == "shm" ? "SHM non-DMA-BUF rejection" :
                  mode == "shm_rga" ? "qRoad RGA shared-memory probe" : "encoderd recovery contract")
              << " passed: " << mode << '\n';
  } catch (const std::exception &error) {
    std::cerr << "encoderd recovery contract failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
