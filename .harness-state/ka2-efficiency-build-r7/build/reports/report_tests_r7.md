# Test report — r7 MPP copy fallback

## Outcome

**Pass.** The focused pytest invocation ran one test item. The test compiled and ran a native C++ harness that calls the same `mpp_encoder_internal` helpers used by the production MPP encoder. The runner log is `build/evidence/tests-pytest.log` (SHA-256 `5475017575d3243b8b6bf21300c9407ed97553550e9f764538146db7eac5108e`).

## Command and build details

From `/tmp/ka2-efficiency-r7`:

```sh
pytest -q --confcutdir=system/loggerd/tests system/loggerd/tests/test_mpp_encoder.py
```

Pytest 9.0.2 ran under Python 3.12.3. The test wrapper compiles the adjacent C++ harness with clang++ 18.1.3 using:

```sh
clang++ -std=c++1z \
  -I/tmp/ka2-efficiency-r7 \
  -I/tmp/ka2-efficiency-r7/third_party/linux/include \
  -I/usr/local/include/rockchip \
  /tmp/ka2-efficiency-r7/system/loggerd/tests/test_mpp_encoder.cc \
  -o <pytest-tmp-path>/mpp_encoder_helper_test
```

The wrapper checks the temporary executable's ELF dependencies with `readelf -d` before running it and fails if it links `librockchip_mpp`, `librga`, or `libzmq`. No MPP libraries are linked. It does not construct `MppEncoder`, create `PubMaster`/IPC sockets, launch encoderd/loggerd/camerad/sensord, or call MPP APIs.

## Behavior covered

- First import failure switches the explicit mode state to copy, initializes the pool, copies the currently selected frame, and submits that same frame once. Callback order and frame identity are asserted.
- The same state remains copy mode through two calls to the production `reset_import_state` helper (used by encoder close/open); the next frame skips import and copies/submits.
- Pool initialization failure and copy failure return failure without submission.
- Successful import submits the imported handle without pool initialization or copy.

## Limits

This test exercises the shared helper with fake callbacks; it does not call actual `encoder_open()`/`encoder_close()` methods or MPP. It does not verify MPP import retry/cleanup, imported-buffer lifetime through pending hardware work, full encoded output/decoding, or qRoad behavior. G0 marks the hardware synthetic A/B unavailable on the current board because `CmaFree` and `nr_free_cma` are zero and userspace MPP readiness remains unproven. No hardware A/B or zero-loss/performance claim is made.

The repository declares Python 3.13 or newer; the available interpreter is Python 3.12.3. This wrapper uses pytest and the standard library only and passed under the available interpreter without importing project Python modules.

## Inputs

- `system/loggerd/encoder/mpp_encoder.cc`: `c98b128bd2d7a14177ac3ed34194001e7277c117d98ed5f8c743ef1ab3340b48`
- `system/loggerd/encoder/mpp_encoder.h`: `c6893a6df9ae79c55e90897d96a1ad72bc3fc648759a123114f2ec21140265a7`
- `system/loggerd/encoder/mpp_encoder_input.h`: `7c05efe9159fdd0dcb9e30a5540941b2c22fc7d83413b4f58011a3d2d511953f`
- `system/loggerd/tests/test_mpp_encoder.cc`: `68d6ad08fa4a8b44df103fca82b701c97837a2560861f13b00fb97a227544a12`
- `system/loggerd/tests/test_mpp_encoder.py`: `155ce3a252319a4baec661549f4a55472b820c744ff182a4819952d1c8280b15`
