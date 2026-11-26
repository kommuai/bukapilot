#define main loggerd_program_main
#include "system/loggerd/loggerd.cc"
#undef main

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>

static void check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

static std::string exact_empty_report() {
  LoggerdDropCounts drops;
  return make_rotation_report("route", 1, 2, false, {}, 0, drops);
}

static std::string wait_for_param_contains(Params *params, const std::string &key,
                                           const std::vector<std::string> &expected_fragments) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    std::string value = params->get(key);
    if (!value.empty() && std::all_of(expected_fragments.begin(), expected_fragments.end(),
                                      [&value](const std::string &fragment) {
                                        return value.find(fragment) != std::string::npos;
                                      })) {
      return value;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw std::runtime_error("timed out waiting for expected isolated Params contents in " + key);
}

static void clear_param_if_present(Params *params, const std::string &key) {
  if (!params->get(key).empty()) check(params->remove(key) == 0, "remove stale isolated Params value");
}

static void test_rotation_clock_and_report() {
  LoggerdState state;
  state.max_waiting = 2;
  logger_rotate(&state);
  Params params;
  check(params.get("LoggerdRotationReport").empty(), "startup logger_rotate must not emit a runtime report");

  loggerd_test_advance_monotonic_ms(5000);
  std::chrono::steady_clock::time_point decision{};
  check(!rotate_if_needed(&state, &decision), "readiness deadline must start without committing early");
  check(state.rotation_started, "rotation must start at configured segment duration");
  const auto started_at = state.rotation_started_at;

  loggerd_test_advance_monotonic_ms(4999);
  check(!rotate_if_needed(&state, &decision), "rotation must wait until the exact readiness deadline");
  loggerd_test_advance_monotonic_ms(1);
  check(rotate_if_needed(&state, &decision), "rotation must commit at the five second deadline");
  check(std::chrono::duration_cast<std::chrono::milliseconds>(decision - started_at).count() == 5000,
        "commit decision must carry exact fake-clock elapsed time");

  LoggerdState ready_state;
  ready_state.max_waiting = 1;
  logger_rotate(&ready_state);
  loggerd_test_advance_monotonic_ms(5000);
  check(!rotate_if_needed(&ready_state, &decision), "ready-state rotation must first publish started");
  ready_state.ready_to_rotate = ready_state.max_waiting;
  check(rotate_if_needed(&ready_state, &decision), "all-ready rotation must commit immediately");
  check(decision == ready_state.rotation_started_at, "all-ready commit must have zero deadline wait");
  logger_rotate(&ready_state);
  check(ready_state.ready_to_rotate == 0 && !ready_state.rotation_started,
        "every commit must reset readiness for the next epoch");

  const std::string expected =
    "{\"route\":\"route\",\"epoch\":1,\"segment\":2,\"degraded\":false,"
    "\"missing_streams\":[],\"elapsed_ms\":0,\"known_drop_counts\":{"
    "\"stale_video_messages\":0,\"future_video_queue_drops\":0,"
    "\"audio_no_writer_events\":0,\"audio_expired_events\":0,\"audio_capacity_drops\":0}}";
  check(exact_empty_report() == expected, "rotation report must have the exact seven-key contract");
}

static void test_missing_stream_report() {
  LoggerdState state;
  logger_rotate(&state);
  state.rotation_started = true;
  state.rotation_started_at = loggerd_monotonic_now();
  RemoteEncoder ready;
  RemoteEncoder missing;
  ready.marked_ready_to_rotate = true;
  std::unordered_map<std::string, RemoteEncoder *> remotes{{"road", &ready}, {"qRoad", &missing}};
  std::map<std::string, EncoderInfo> infos;
  LoggerdAudioQueue queue;
  Params params;
  clear_param_if_present(&params, "LoggerdRotationReport");
  commit_deterministic_runtime_rotation(&state, {"road", "qRoad"}, remotes, infos,
                                       &queue, nullptr, &params, loggerd_monotonic_now());
  const std::string report = wait_for_param_contains(
    &params, "LoggerdRotationReport", {"\"segment\":1", "\"epoch\":2", "\"missing_streams\":[\"qRoad\"]"});
  std::cout << "MISSING_REPORT " << report << '\n';
  check(report.find("\"missing_streams\":[\"qRoad\"]") != std::string::npos,
        "missing-stream report must preserve configured order and exact names");
  check(report.find("\"degraded\":true") != std::string::npos,
        "missing-stream report must be degraded");
}

static void test_audio_order_capacity_and_terminal_drop() {
  LoggerdState state;
  LoggerdAudioQueue queue;
  RemoteEncoder remote;
  std::vector<uint8_t> payload(200 * 1024, 0x5a);
  admit_audio_event(&state, &queue, &remote, 0, 200, 16000, payload.data(), payload.size());
  admit_audio_event(&state, &queue, &remote, 0, 100, 16000, payload.data(), 32 * 1024);
  check(queue.events.size() == 2, "admitted audio events must be retained whole");
  check(queue.events.front().log_mono_time == 100 && queue.events.back().log_mono_time == 200,
        "queued audio must be timestamp ordered");

  const size_t before_capacity_rejection = queue.bytes;
  std::vector<std::pair<uint64_t, std::vector<uint8_t>>> before_events;
  for (const auto &event : queue.events) before_events.emplace_back(event.log_mono_time, event.data);
  admit_audio_event(&state, &queue, &remote, 0, 300, 16000, payload.data(), 64 * 1024);
  check(queue.bytes == before_capacity_rejection && queue.events.size() == 2,
        "capacity rejection must preserve the existing queue unchanged");
  for (size_t i = 0; i < queue.events.size(); ++i) {
    check(queue.events[i].log_mono_time == before_events[i].first && queue.events[i].data == before_events[i].second,
          "capacity rejection must preserve queued payload bytes and timestamp order");
  }
  check(state.drop_counts.audio_capacity_drops == 1,
        "one over-capacity input event must count once");

  LoggerdAudioQueue expiry_queue;
  LoggerdState expiry_state;
  admit_audio_event(&expiry_state, &expiry_queue, &remote, 0, 1, 16000, payload.data(), 100);
  const auto expires_at = expiry_queue.events.front().expires_at;
  expire_queued_audio(&expiry_state, &expiry_queue, expires_at - std::chrono::milliseconds(1));
  check(expiry_state.drop_counts.audio_expired_events == 0, "audio must remain until its exact expiry");
  expire_queued_audio(&expiry_state, &expiry_queue, expires_at);
  check(expiry_state.drop_counts.audio_expired_events == 1 && expiry_queue.events.empty(),
        "expiry must be terminal at the exact expiry timestamp");
}

static void test_audio_expiry_wins_commit_tie() {
  LoggerdState state;
  logger_rotate(&state);
  state.rotation_started = true;
  state.rotation_started_at = loggerd_monotonic_now();
  LoggerdAudioQueue queue;
  RemoteEncoder remote;
  const uint8_t payload[] = {1, 2, 3, 4};
  admit_audio_event(&state, &queue, &remote, state.logger.segment(), 100, 16000,
                    payload, sizeof(payload));
  const auto decision = queue.events.front().expires_at;
  std::unordered_map<std::string, RemoteEncoder *> remotes{{"qRoadEncodeData", &remote}};
  std::map<std::string, EncoderInfo> infos;
  Params params;
  clear_param_if_present(&params, "LoggerdRotationReport");
  commit_deterministic_runtime_rotation(&state, {}, remotes, infos, &queue, &remote, &params, decision);

  check(queue.events.empty(), "commit must dispose old-segment queued audio");
  check(state.drop_counts.audio_expired_events == 0,
        "commit resets the local drop window after emitting its report");
  const std::string report = wait_for_param_contains(
    &params, "LoggerdRotationReport", {"\"segment\":1", "\"epoch\":2", "\"audio_expired_events\":1"});
  std::cout << "REPORT " << report << '\n';
  check(report.find("\"audio_expired_events\":1") != std::string::npos,
        "an exact expiry/commit tie must be reported as expired");
  check(report.find("\"audio_no_writer_events\":0") != std::string::npos,
        "an expiry/commit tie must not be double-counted as no-writer");
}

static void test_video_writer_closes_before_first_keyframe() {
  char dir[] = "/tmp/loggerd-empty-segment-XXXXXX";
  char *temp_dir = mkdtemp(dir);
  check(temp_dir != nullptr, "create temporary empty-segment directory");
  const std::string path(temp_dir);
  const std::string video_path = path + "/qcamera.ts";
  const std::string lock_path = video_path + ".lock";
  {
    VideoWriter writer(path.c_str(), "qcamera.ts", true, 526, 330, 20,
                       cereal::EncodeIndex::Type::QCAMERA_H264);
    const uint8_t audio[] = {0, 0, 0, 0};
    writer.write_audio(const_cast<uint8_t *>(audio), sizeof(audio), 0, 16000);
    // No codec header/keyframe is submitted. Destruction must safely discard
    // buffered audio and avoid calling the muxer's trailer without a header.
  }
  struct stat st = {};
  check(stat(video_path.c_str(), &st) == 0, "headerless segment output remains present");
  check(unlink(video_path.c_str()) == 0, "remove temporary empty-segment output");
  check(access(lock_path.c_str(), F_OK) != 0, "writer removes temporary empty-segment lock");
  check(rmdir(path.c_str()) == 0, "remove temporary empty-segment directory");
}

static void wait_for_param(Params *params, const std::string &key, const std::string &expected) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (params->get(key) == expected) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw std::runtime_error("timed out waiting for isolated Params key " + key + "=" + expected);
}

static void test_no_message_event_loop_deadline() {
  Params params;
  clear_param_if_present(&params, "LoggerdRotationReport");
  do_exit = false;
  std::thread logger_thread([]() { loggerd_thread(); });
  try {
    wait_for_param(&params, "LoggerdSegment", "0");
    loggerd_test_advance_monotonic_ms(5000);
    wait_for_param(&params, "LoggerdRotateState", "started");
    loggerd_test_advance_monotonic_ms(5000);
    wait_for_param(&params, "LoggerdSegment", "1");
    const std::string report = wait_for_param_contains(
      &params, "LoggerdRotationReport", {"\"segment\":1", "\"epoch\":2", "\"elapsed_ms\":5000"});
    std::cout << "NO_MESSAGE_REPORT " << report << '\n';
    check(report.find("\"elapsed_ms\":5000") != std::string::npos,
          "no-message runtime commit must report the exact five-second deadline");
    check(report.find("\"degraded\":true") != std::string::npos,
          "no-message rotation must identify missing configured streams");
  } catch (...) {
    do_exit = true;
    logger_thread.join();
    throw;
  }
  do_exit = true;
  logger_thread.join();
}

static void test_non_rk3588_loggerd_legacy_guard() {
#if defined(RK3588)
  throw std::runtime_error("compile-only legacy guard target still defines RK3588");
#else
  check(!deterministic_ka2_mode(), "non-RK3588 build must disable deterministic KA2 mode");
  check(!deterministic_ka2_protocol_active(),
        "LOGGERD_TEST_DETERMINISTIC must not activate KA2 protocol in a non-RK3588 build");

  Params params;
  clear_param_if_present(&params, "LoggerdRotationReport");
  LoggerdState state;
  state.max_waiting = 1;
  logger_rotate(&state);
  check(params.get("LoggerdRotationReport").empty(), "non-RK3588 startup must not publish a rotation report");
  const int startup_segment = state.logger.segment();
  state.ready_to_rotate = state.max_waiting;
  std::chrono::steady_clock::time_point decision{};
  check(!rotate_if_needed(&state, &decision), "legacy rotation path must not return a deterministic commit decision");
  check(state.logger.segment() == startup_segment + 1,
        "legacy all-ready rotation must proceed without waiting for a deterministic deadline");
  check(params.get("LoggerdRotationReport").empty(), "non-RK3588 runtime rotation must not publish a report");
  std::cout << "NON_RK3588_LOGGERD_GUARD deterministic_test_mode=" << deterministic_test_mode
            << " protocol_active=0 startup_report=none runtime_report=none all_ready_rotated=1\n";
#endif
}

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--non-rk3588-legacy-guard") {
      test_non_rk3588_loggerd_legacy_guard();
      return 0;
    }
    check(argc == 1, "unexpected loggerd rotation test argument");
    check(loggerd_test_monotonic_ms() == 0, "test fake clock must start at zero");
    loggerd_test_enable_monotonic_clock();
    check(loggerd_test_monotonic_ms() == 0, "explicit fake-clock enable must preserve the zero origin");
    test_rotation_clock_and_report();
    test_missing_stream_report();
    test_audio_order_capacity_and_terminal_drop();
    test_audio_expiry_wins_commit_tie();
    test_video_writer_closes_before_first_keyframe();
    test_no_message_event_loop_deadline();
    std::cout << "loggerd rotation, report, and audio contracts passed\n";
  } catch (const std::exception &error) {
    std::cerr << "loggerd rotation contract failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
