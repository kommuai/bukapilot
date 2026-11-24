# Admiral checkpoint — AC-4 measurement plan ready for review

- Run: `sidecamerad-mvp-20261113`
- Expected prior revision: `148`
- Investigation-review revision 8 is APPROVED; its verdict is recorded in the canonical handoff and gate artifacts.
- BUILD plan: `build/reports/report_ac4-same-frame-measurement-plan-r1.md`
- Plan SHA-256: `da5a7e36e58a99f0df989eaa7eb71dbe6aacdd9514d2be757623bbb2b8fc04fc`.
- The plan scopes a max-600-frame, no-payload same-frame timeline on the same camera and production binary/default argv. It specifies V4L2 sequence/dequeue, FFmpeg decoder and filter boundaries, pipe writes, full-read EOF, scoped thread scheduling, observer controls, abort conditions, and interpretation branches.
- Authorization boundary: preparation only. No capture, trace, test, GDB/strace session, source instrumentation, source edit, or argv change was run or authorized. A concrete instrumentation diff/tool choice and overhead proposal must be submitted to Admiral before any execution.
- AC-4 remains failed: 6,050 frames over 302.471744064 s; mean 19.998562241636 fps; maximum EOF gap 90.981562 ms; 716/6,049 intervals >60 ms. The mechanism remains unresolved.
- Open items: AC-6 and AC-8 hardware deferrals; Minor FFmpeg child resource-cap residual.
- Next action: Admiral reviews the plan with the user and records a separate go decision or requests plan changes. Do not execute the proposal before that decision.
