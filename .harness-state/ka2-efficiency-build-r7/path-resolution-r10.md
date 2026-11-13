---
run_id: loggerd-ka2-efficiency-20261029
submission_id: loggerd-ka2-efficiency-20261029-build-r10
revision: 231
---
# Build path-resolution addendum — r10

The r10 build package is stored in a project-contained snapshot so both build validators can resolve its canonical run ancestry while the phase package remains inside the workspace boundary.

## Layout and preservation

- Canonical logical build path: `skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/`.
- Previous target retained unchanged: `/data/openpilot/.harness-state/ka2-efficiency-build-r7/build`.
- Its tree digest before and after the symlink retarget is `d5215f9044bbf2e826f7d6035c1c07adcab4b51caabc6d15774f76247916fd02` (512 files, 33 directories, 6 symlinks).
- Current target: `/data/openpilot/.harness-state/ka2-efficiency-build-r7/layout-probe-r10/skillset-saves/runs/loggerd-ka2-efficiency-20261029/build`.
- Snapshot root: `/data/openpilot/.harness-state/ka2-efficiency-build-r7`; it contains copies of the run `_state.md`, Intake, complete r10 design package, current build evidence, and project-relative source inputs.
- All 18 test/runtime probe input paths and all 16 implementation change-set files were checked against their registered SHA-256 values in the snapshot.
- The approved design cross-stage verdict is `../design/verdict_design-to-build-r10.cross-stage.json`, SHA-256 `bc2d854ba4675408eb4c5329ee1252b4645f78075629aa4fe70d5f952042d75d`. The generic unsuffixed cross-stage file is an older r6 record and is not used.

## Save-registry relocation

After the symlink change, the save checkpoint showed that 311 historical build artifacts were registered at canonical paths but no longer present in the current r10 package. Checkpoint r237 moved those registrations to matching archive paths under `/data/openpilot/.harness-state/ka2-efficiency-build-r7/build`. The prior archive files and the old build tree were left unchanged. The save audit records the dropped and added paths; the complete pairwise path/hash map is also packaged at `evidence/path-resolution-r237-evidence-relocations.json`, SHA-256 `f4538435f665dd739e0913045fa215a5e4181fe8ddbd58862c9018338529b879` (archive copy: `.harness-state/ka2-efficiency-build-r7/path-resolution-r237-evidence-relocations.json`).

Two other historical evidence entries already had hash drift before this layout change. Their original expected hashes remain unchanged in the run registry; their existing bytes were copied into the snapshot so every registered logical path still resolves. They are not part of the r10 manifest:

- `skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence/loggerd-expected-emitted-frame-ledger.csv`: registered `e2e1ae2067eba4a01e7795cddce48ea27ba81ad523bd15cdb8a66174c362b70e`, observed `7b07ddd4252a7ef8ea15212d1e7ff9ae038495579580cd42a5a52de929e6f015`.
- `skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence/rerun_20261030_023059/frame_ledger.csv`: registered `82e18f684b2b086e22d9da3f496a1bcecf5032863ee40c2b89a47d19abf6fb9a`, observed `27625d6c70829ffd8fb8b46cfceaa5843d712cc3bc5ed6946598a70a6d0557c5`.

## Validator and output-path checks

Before retargeting, the temporary schema-2 package check exited 0 with `pass: true`, `evidence_root_kind: run`, and no failures. The phase package-shape check exited 0 with zero failures; its `NEEDS_JUDGMENT` result reflected only the disposable incomplete fixture. `output_paths.py` accepted the project-contained canonical destination.

After retargeting, the required boundary self-check again returned `pass: true` with no failures and an evidence root at the nested run path. The build shape check again exited 0 with zero failures, and `output_paths.py --kind manifest --mkdir` accepted the canonical destination. The historical report archive remains outside the build package scan.

No product source files or board workloads changed during the path correction. This does not close AC-12's genuine non-KA2 target requirement or qualify live V4L2 DMA-BUF export, fence synchronization, camera-buffer reuse, or the deployed MPP package.
