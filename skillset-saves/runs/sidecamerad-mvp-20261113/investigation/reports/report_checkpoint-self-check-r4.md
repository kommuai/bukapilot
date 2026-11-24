# Checkpoint — investigation revision 4 self-check

Canonical `investigation-review` mechanical self-check: pass=true; failures=[]; warnings=[]; mechanical_only=true; revision 4; submission `sidecamerad-mvp-20261113_investigation-ac4-timing-r4`. No verdict was written and no gatekeeper was invoked.

- Manifest SHA-256: `72a75339145e4d031dec69c9c278409db619dbe54ac33bdad5c0a634a364722e`
- Package SHA-256: `39f762cc1f8002a5ea694cf59fe638e359a856ac8e206a8b1c52209900da08bc`
- Evidence-chain SHA-256: `efd09d6751444a40bdc5d370d0816c2de2061f259cb188201cd5c2df6884ca40`
- Accepted reproduction SHA-256: `de804d35326c6925862d2c8b36708d4a14a10b05120076924f523397976e8c55`
- Intake log SHA-256: `84a7e874f86ba50a6e4c5b3e3938e4a534099e518c4dea8c9ba8a8f4f2d97a0b`
- Preserved r3 package SHA-256: `e83647146bf85fb14b60b44c2aa8c6d209882ded91a2e7f3e5991a9106a1d573`; r2 archive: `b35b75239d451b80d9cf7015028d4598a821f0e110debd3927213fab67e44cfd`; r1 archive: `b4afe53167fb5ff722d3b8174f7e339f98d3cc66578d84d420e12acb8e6bc796`.

ABBA results do not reproduce the earlier flush-pacing improvement. The bounded next BUILD diagnostic correlates per-thread FFmpeg wait syscall IDs/durations and schedstat runtime/runqueue deltas with 600 full-read completion intervals. It remains unrun, diagnostic-only, and explicitly accounts for strace and /proc observer overhead. AC-4 remains failed on the accepted five-minute reproduction; timestamp semantics remain unchanged.
