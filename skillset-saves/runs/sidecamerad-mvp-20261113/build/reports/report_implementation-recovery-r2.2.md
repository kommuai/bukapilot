# Implementation evidence refresh — sidecamerad MVP

**Run:** `sidecamerad-mvp-20261113` · **Approved design:** r2 · **Evidence refresh:** r2.2

## Scope and result

The recovered working tree retains the approved side-camera implementation. The merge’s existing `NavLocationSource` and `NavRoadMatch` entries remain intact. The only current diff hunk in `common/params_keys.h` adds `SideCameraAvailability`; no Nav edit is part of this implementation patch. SCons rebuilt the generated Params binding from the restored header, and the recorded build artifact SHA matches the current binary.

No product source edits were made for this refresh. The scoped changed-file set remains ten paths: sidecamerad, the focused test, manager registration, Params declaration and generated binding, cereal schema/services, and three VisionIPC source files. There are no package changes. No placeholder or unowned follow-up marker remains in the implementation.

## Recovered artifact hashes

| Path | SHA-256 | Status |
|---|---|---|
| `system/sidecamerad.py` | `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` | added |
| `system/manager/test/test_sidecamerad.py` | `cf9fb49f514ac90458d47d825716a30b296b8cb25880167695c6a86a6660a059` | added |
| `system/manager/process_config.py` | `b7fdfda2878cae1fa6b354ed512bc33ad66f1abe1ed25c90ed980b547a8aa02d` | modified |
| `common/params_keys.h` | `c2925ebd8f9f26cc04951afb0e3c67ac7cb20a1c370b79aa93a25a4aaef2a031` | modified |
| `common/params_pyx.so` | `3108e29407f31b9bd86a86ccafc82a4f98a225884412987a24be5c357dbfb586` | modified |
| `cereal/log.capnp` | `d5c0372d714e65461942874c91c522b205ce208261b382283839ed03b9731f64` | modified |
| `cereal/services.py` | `cacd4b749d9a0a3e19e3c466784dcf34925f263fdca3c26af72578c47d740a6b` | modified |
| `msgq_repo/msgq/visionipc/visionbuf.h` | `c971a1afa601f5bb8f560de6fb1dd78c2121cee61ea41c169f00a4154566e870` | modified |
| `msgq_repo/msgq/visionipc/visionipc_pyx.pyx` | `36247e353901eb80e52735c91f466151b3026e77152dde9ad9e3bcd682c66f32` | modified |
| `msgq_repo/msgq/visionipc/visionipc_server.cc` | `36b383bfcaf10eaa69f6844a579c86204b4a07464a889208fe059a6c1202d198` | modified |


## Binding evidence

- Current header SHA-256: `c2925ebd8f9f26cc04951afb0e3c67ac7cb20a1c370b79aa93a25a4aaef2a031`.
- Current generated binding SHA-256: `3108e29407f31b9bd86a86ccafc82a4f98a225884412987a24be5c357dbfb586`.
- SCons build log: `evidence/native-binding-build-r2.log`, SHA-256 `476fa98acb97951dfc9fe89d7cb315552cd73d4703d7acc0f563f3d2753f9a09`; recorded artifact hash matches the current `common/params_pyx.so`.
- Existing isolated Params probe confirms the compiled binding recognizes the key, returns the exact all-false dictionary default, and round-trips a Python dictionary.

The implementation diff is `evidence/implementation-recovery-r2.2.diff`, SHA-256 `caa56a37320eb173be0d108ed995e9a3b543e750b5d3be821e8091d18f2b36ed`. Typed `change_set` record is `artifacts/implementation-recovery-r2.2.json`, SHA-256 `4da275a592a8901f3da0d168ff006fb35904e5e755fbe2faa28492f9f6b48a35`. The record is a proposal for build-management consolidation; this refresh did not write `build/manifest.json` or advance the shared run state.
