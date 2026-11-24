# Stack lock — sidecamerad r2

**Run:** `sidecamerad-mvp-20261113` · **Revision:** `r2`

This revision keeps the existing openpilot Python/C++/Cap’n Proto/msgq and FFmpeg stack. It introduces no runtime, framework, or package. The default-on manager lifecycle and availability Param use repository facilities already present in the project.

```yaml
applicable: false
reason: "no new runtime or framework - existing stack unchanged"
scope: "sidecamerad r2 reuses the existing stack and FFmpeg executable; no packages installed"
decided_by: "commander"
```
