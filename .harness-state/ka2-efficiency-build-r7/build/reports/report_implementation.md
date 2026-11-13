# Implementation report — r7 MPP copy fallback

**Run:** `loggerd-ka2-efficiency-20261029`  
**Base revision:** `103d166829b3be04d735005ee83c24bd119764e2`  
**Candidate worktree:** `/tmp/ka2-efficiency-r7` (detached at the pinned base)  
**Design spec:** r7 implementation spec, SHA-256 `a7f06c00f8c671aa8c6a980c75e1abe1ae6b62f11df8db5bb66651033bfcb347`

## Outcome

The approved narrow change makes a full-resolution MPP import failure switch immediately to sticky copy mode. If DMA import fails, the same frame initializes/uses the copy pool, acquires a buffer, copies the frame, and submits it in that call. Pool initialization, buffer acquisition, or copy failure returns an error without submitting. The failure path no longer waits for a run of repeated import failures before falling back.

The constructor still leaves `ENCODER_ZERO_COPY=0` as the default. The copy fallback latch remains false across `encoder_close()` and `encoder_open()` on the same `MppEncoder` object. Existing downscale/qRoad handling, public encoder interface, codec settings, rotation, and writer behavior are outside this slice.

No efficiency gain, zero-loss result, or decoded-content equivalence is claimed. The hardware A/B comparison was not run.

## Production-shared deterministic test seam

`system/loggerd/encoder/mpp_encoder_input.h` contains the narrow internal templated `encode_full_resolution_frame` helper and the `reset_import_state` helper used by production `encoder_open()` and `encoder_close()`. Production and tests call these same helpers. The tests inject import, pool, acquire, copy, and submit callbacks; they do not construct `MppEncoder`, start IPC, or initialize MPP.

The focused test cases cover first-import-failure same-frame copy and single submit, no submit after pool/allocation or copy failure, successful import without copy, and the false latch surviving the production reset hook after a real import failure. The clean focused run passed **1 pytest item** (the item builds and runs the native helper harness).

## Validation and builds

- Focused test command from `/tmp/ka2-efficiency-r7`: `pytest -q --confcutdir=system/loggerd/tests system/loggerd/tests/test_mpp_encoder.py` — exit 0, `1 passed in 3.33s`. The `--confcutdir` option keeps this standalone helper harness independent of repository-root fixtures and project Python modules.
- Clean candidate rebuild: after removing generated build outputs only inside the detached worktree, `OPENPILOT_BUILD_GUI=0 scons --no-cache system/loggerd/encoderd` completed successfully. The binary was not launched. SHA-256 `655d3449384899c05f33ac98955a96671e1aae9fe1883b91946f98d416a44798`; ELF Build ID `f9eea4e181c7d0bfad3ddb5d41c5e9f1ff169723`.
- Merged-root overlay rebuild: the current merged encoder/helper files and `encoderd.cc` from the dirty root checkout were overlaid into separate clean pinned worktree `/tmp/ka2-efficiency-r7-mergecheck`. `OPENPILOT_BUILD_GUI=0 scons --no-cache system/loggerd/encoderd` completed successfully. This is a supplementary merge-compatibility build, not the clean candidate. Its binary SHA-256 is `c30b920d13b072d271122408822192e6e427b8ecac4ed289e3cecd64b7d37b47`; ELF Build ID `a956b9960bf8e8ebb08b7e560570ff4c85449b1d`.
- `ldd -r` on both uninvoked binaries reported no unresolved symbols or missing libraries. Both resolve `librockchip_mpp.so.1` to `/lib/aarch64-linux-gnu/librockchip_mpp.so.1`, the older system copy described in [G0 preflight](../evidence/g0-preflight.md). The distinct `/usr/local/lib` MPP library remains installed.
- `git diff --check` and the whitespace scan over the three added files passed.
- No camera, encoder, sensor, or production process was run. The board comparison remains unavailable: G0 found `CmaFree=0` / `nr_free_cma=0` and userspace MPP readiness unproven. Runtime-health evidence is therefore not available under the no-encoder execution boundary.

## Candidate changed artifacts

All five hashes below are from the clean detached candidate, not the dirty root checkout.

| Path | SHA-256 | Classification |
| --- | --- | --- |
| `system/loggerd/encoder/mpp_encoder.cc` | `c98b128bd2d7a14177ac3ed34194001e7277c117d98ed5f8c743ef1ab3340b48` | First-party source |
| `system/loggerd/encoder/mpp_encoder.h` | `c6893a6df9ae79c55e90897d96a1ad72bc3fc648759a123114f2ec21140265a7` | First-party source |
| `system/loggerd/encoder/mpp_encoder_input.h` | `7c05efe9159fdd0dcb9e30a5540941b2c22fc7d83413b4f58011a3d2d511953f` | First-party internal helper |
| `system/loggerd/tests/test_mpp_encoder.cc` | `68d6ad08fa4a8b44df103fca82b701c97837a2560861f13b00fb97a227544a12` | First-party test |
| `system/loggerd/tests/test_mpp_encoder.py` | `155ce3a252319a4baec661549f4a55472b820c744ff182a4819952d1c8280b15` | First-party test harness |

## Dirty-root merge verification

The parent merged the approved patch into `/data/openpilot` with a three-way merge, preserving the checkout's pre-existing dirty work. That checkout remains dirty and is not the candidate. Its changed-file snapshot and hashes, including unrelated existing files, are in [impl-merged-root-hashes.txt](../evidence/impl-merged-root-hashes.txt). The merge-check worktree contained the merged `mpp_encoder.cc`, `mpp_encoder.h`, `mpp_encoder_input.h`, and current `encoderd.cc`; those inputs are distinguished from the clean candidate's five-file change set above.

## Evidence index

| Evidence | SHA-256 |
| --- | --- |
| Candidate implementation diff (`impl.diff`, five files) | `cdf2a3721a82e686de9f6ee6ee26cef923cd2f2d42c4763368eeb9c86ad39396` |
| Candidate final build log (`impl-candidate-build-final.log`) | `3493a0f30f597a117e17e66223c15076e5ab9c4af4a0615d32a71612e6310bf9` |
| Merged-root overlay build log (`impl-merged-overlay-build.log`) | `3493a0f30f597a117e17e66223c15076e5ab9c4af4a0615d32a71612e6310bf9` |
| Diff and whitespace check (`impl-diff-check.log`) | `7c28d9e2a57c7e03a0026d43dcc1d2d7755b51e011fdd1cf6a5ce0599f16bf6e` |
| Binary SHA/Build ID/dependency inspection (`impl-binary-inspection.log`) | `ab0f3ba322044f2f865e022f6d3eb0d476fa5504de69452f2b54cfa11779a37e` |
| Focused pytest log (`tests-pytest.log`) | `5475017575d3243b8b6bf21300c9407ed97553550e9f764538146db7eac5108e` |
| Typed focused-test probe (`tests-probe.json`) | `93f995f5e4ed2f8b63ab023f737992d1babb4b9d1dba0b3e3dfe79e4624421b5` |
| Focused-test report (`report_tests_r7.md`) | `a1ca76d29679e38f5e7d87946dbdb2c66afe89bccef9ccb2676793bf455918f2` |
| Merged dirty-root hash snapshot (`impl-merged-root-hashes.txt`) | `932fefd1de1789cbec3aae00c23e0aa83d42de7d976a6f75c287711fa8d12650` |

The earlier `impl-build.log` is retained as historical output from the pre-helper build and is superseded by the two final build logs above.

## Open risks and remaining build stages

- The DMA input lifetime and the installed MPP driver's completion behavior remain unproven. The imported-buffer references remain retained until close, but this does not prove when hardware stops reading them.
- The active system MPP library differs from `/usr/local/lib/librockchip_mpp.so.0`; only build-time loader resolution was checked. No encoder startup/runtime smoke was performed.
- No same-binary copy/import A/B, throughput/CPU measurement, loss report, or decoded-content comparison was possible. The design's hardware comparison remains bounded and pending a board with proven MPP/CMA readiness.
- Test evidence is complete for the pure state-machine helper. The approved r7 security seed finds no new or moved trust boundary; the build may use the sanctioned `security_evidence` applicability record `no trust-boundary change - security-builder not engaged`, which is not a security validation. Runtime-health and completeness remain separate build stages; this report does not assert a `build-to-review`-ready package.
