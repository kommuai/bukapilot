# Admiral checkpoint — AC-4 diagnostic authorization

- Run: `sidecamerad-mvp-20261113`
- Expected prior revision: `149`
- User authorization: “Can you find out why is there a huge gap? Is it a hardware issue or software issue. Keep testing and finding it out.” Recorded as authorization to continue bounded diagnosis.
- Approved investigation handoff: r8 `investigation-review` APPROVED; verdict ID `505603e061b86731f48b4e4fdac4d487c3d6bae32c5d202a9a3452de7f9f7099`.
- Bounded plan: `build/reports/report_ac4-same-frame-measurement-plan-r1.md`, SHA-256 `da5a7e36e58a99f0df989eaa7eb71dbe6aacdd9514d2be757623bbb2b8fc04fc`.
- Prior evidence leans toward post-dequeue software/host timing: direct V4L2-only capture showed regular source intervals; separate FFmpeg-to-raw-pipe captures showed long output gaps. Those probes were sequential and lack exact same-frame identity, so hardware versus software is not settled.
- Authorized diagnostic scope: one run, same assigned front-left UVC camera and 1280×720 MJPEG mode; unchanged production FFmpeg binary and default argv; at most 600 frames; no frame payload saved; stop after the first fully observed >60 ms EOF gap or at the cap. Correlate V4L2 DQBUF sequence/completion with full-frame output/read times and record narrowly scoped per-thread scheduling data only if the capture method can do so without changing production source or argv.
- No product source changes, acceptance-run repetition, or fixes are authorized by this checkpoint. If required frame identity or timestamps cannot be collected under these bounds, stop and return the evidence gap rather than broadening the measurement.
- Next action: BUILD runs the bounded same-run diagnostic, cleans up, hashes the metadata evidence, and returns a hardware/software localization with observer limitations. Admiral then decides whether a separate source fix or acceptance run is needed.
