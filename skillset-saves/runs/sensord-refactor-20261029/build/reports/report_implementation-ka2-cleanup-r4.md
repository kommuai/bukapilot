# Implementation report — KA2-only sensord cleanup r4

## Outcome

Implemented the approved r4 delta on top of the existing r3 cleanup. No commit was created and no hardware or I2C access occurred.

The r4 delta removes the `KA2` import and early guard from `system/sensord/sensord.py`, removes the explicit `enabled=KA2` argument from the manager registration while retaining `only_onroad`, removes the r3-only KA2 gating test from `system/manager/test/test_manager.py`, and deletes the guard-only `system/sensord/tests/test_sensord.py`. No replacement platform test was added. The manager-test removal returns that file to its pre-r3 content, so it has no net diff against the repository base.

The KA2 path remains source-equivalent: realtime setup precedes construction; sensors remain ordered ICM42670 accelerometer, ICM42670 gyro, ICM42670 temperature, then LIS2MDL magnetometer; all use bus 3. `PythonProcess` defaults `enabled=True`, and sensord retains `only_onroad`. Existing initialization, polling, and shutdown code is unchanged by r4.

## Final change set

The final aggregate diff against the current repository base contains 12 paths (17 insertions, 848 deletions):

- Modified: `Jenkinsfile`, `selfdrive/test/test_onroad.py`, `system/loggerd/tests/test_encoder.py`, `system/manager/process_config.py`, `system/sensord/sensord.py`, `system/sensord/tests/test_event_construction.py`, `system/sensord/tests/test_fixed_calculations.py`.
- Deleted: `system/sensord/sensors/lsm6ds3_accel.py`, `system/sensord/sensors/lsm6ds3_gyro.py`, `system/sensord/sensors/lsm6ds3_temp.py`, `system/sensord/sensors/mmc5603nj_magn.py`, `system/sensord/tests/test_sensord.py`.
- `system/manager/test/test_manager.py` is restored to its repository-base content; no net tracked diff remains there.

The combined diff artifact is `artifacts/implementation-ka2-cleanup-r4.diff`, SHA-256 `6c5736af393cea0f0cbaeb5d9d73538dfb1afdfeeb4fc2e912903e84600d9d4`.

## Validation

- `git diff --check` over the approved paths: exit 0. Log: `evidence/implementation-r4-diff-check.log`, SHA-256 `5f58f2f2c38827d76a8fc5a7cd19fdb5f6a80c98f9f7eeabac736d20e453a6a8`.
- `python3 -m py_compile system/sensord/sensord.py system/manager/process_config.py system/manager/test/test_manager.py`: exit 0. Log: `evidence/implementation-r4-syntax.log`, SHA-256 `039c1875c1bcd3a7a786d93e37ad4ff4fcb07c30e13348eec45c50015655fcb7`.
- Source-level KA2 equivalence assertions: exit 0 for realtime-before-construction, sensor order, bus 3, `only_onroad`, and default-enabled manager configuration. Log: `evidence/implementation-r4-ka2-equivalence.log`, SHA-256 `ffb38911f469068e1e6593002d3c5d199fedcac4912f359e8694725d196ff504`.
- The retained sensord and manager test suite is owned by `build/test-builder`; this implementation stage did not run tests.

## Artifact hashes

Modified files:

| Path | SHA-256 |
| --- | --- |
| `Jenkinsfile` | `1fc1d75dde2b1c0d111d52094b8ad3c89e1fcbeaa3772129d0588d44801f2cd8` |
| `selfdrive/test/test_onroad.py` | `73d78c9af44aa9a45f50314aac9840b15ece9e746989419649c711e04ece7edd` |
| `system/loggerd/tests/test_encoder.py` | `b7b161ba6d5175a204be97ea97f99133cb1965130d26185d1b30a893566c4fe6` |
| `system/manager/process_config.py` | `bd66a0051d0310afa828a47a78a5b7bf0800c7b2f55c0c3b34f69ca3cca172e9` |
| `system/sensord/sensord.py` | `d513926a25141a05fe1ed0330a71ce03cf544fd4db4bf57b3d3f06fe5097a06f` |
| `system/sensord/tests/test_event_construction.py` | `7b72579a563af65a943c7c4c2ecd79a0c824dff48a81f3e91e4604efda812fb6` |
| `system/sensord/tests/test_fixed_calculations.py` | `ced678f349156c9c198509ba9d6a0c4240a64f2b0a6a50466b7c92185a87f045` |

Deleted files are absent from the working tree; their prior file-content hashes are bound by the diff: `lsm6ds3_accel.py` `2653028763b65af3b7489e646e4aec8190aae23f4a8d88e9e5587d5394b3221a`, `lsm6ds3_gyro.py` `830fd3bc58e40d5cafc23e44e1671df8c20e4b9fd30359cc3b2ebb54330c14b6`, `lsm6ds3_temp.py` `b27b9cddb0b762bd86dd4e02ee96ab1c7bcee2f35eef6a0a7db180216c3ccb36`, `mmc5603nj_magn.py` `ab1d1561405762aa189ced0e1ce54e36d79925c0f8eaf7bf532b508b9855b85c`, and the removed guard-only test file `9e887913d1210803d474d64d27ed0aaed244cd4d72e0325f50c29155a394658b`.

No placeholder or unowned follow-up marker remains in this implementation slice. Runtime startup, cadence, shutdown, and hardware identity remain unverified because no authorized target is available; this is not a hardware validation claim.
