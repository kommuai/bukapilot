# Checkpoint — evidence chain revision 3

- Resumed from run revision 97; accepted five-minute reproduction was reused without a rerun.
- Approved revision-2 package preserved byte-for-byte as `reports/investigation-package-r2.md`, SHA-256 `b35b75239d451b80d9cf7015028d4598a821f0e110debd3927213fab67e44cfd`. Revision-1 archive remains unchanged.
- New A/B evidence is a sequential traced pair with only output-side `-flush_packets 1` changed. Read gaps >60 ms fell 122/599 to 107/599; max 88.425 ms to 86.679 ms. Both arms still show substantial gaps, so the small shift is inconclusive and not causal proof.
- New evidence chain: `investigation/evidence/evidence-chain-r3.log`.
- Next: update mechanism/fix path and residual uncertainty, then assemble package and manifest revision 3.
