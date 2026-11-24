# Stack lock — sidecamerad MVP

**Decision:** No new runtime or framework; the existing project stack remains unchanged.

This slice adds Python process code and extends existing C++/pyx VisionIPC bindings and Cap'n Proto/service declarations. Capture uses the FFmpeg executable already present in the target environment. The approved scope adds no runtime, framework, or package, and explicitly forbids package installation. The only native build addition is the existing VisionIPC extension target, `scons msgq_repo/msgq/visionipc/visionipc_pyx.so`.

The required runtime detector was run from `/data/openpilot` with `--detect-project`. It found no supported stack classification and exited incomplete because its bounded inspection stopped at the 10 MB total-byte limit; therefore its empty stack result is not treated as proof that no project stack exists. Its separate Python 3.12.3 versus harness 3.13 recommendation is not a project incompatibility: `/data/openpilot/pyproject.toml` accepts Python `>=3.11,<3.13`. This design does not rely on the harness recommendation or add a runtime dependency.

```json
{
  "applicable": false,
  "reason": "no new runtime or framework - existing stack unchanged",
  "scope": "sidecamerad MVP reuses existing Python/C++/FFmpeg stack; no packages installed",
  "decided_by": "commander"
}
```
