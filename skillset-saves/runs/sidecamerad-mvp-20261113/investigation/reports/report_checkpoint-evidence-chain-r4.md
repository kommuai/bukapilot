# Checkpoint — evidence chain revision 4

- Resumed from active run revision 107; the accepted five-minute reproduction was reused.
- Revision-3 package preserved byte-for-byte at `reports/investigation-package-r3.md`, SHA-256 `e83647146bf85fb14b60b44c2aa8c6d209882ded91a2e7f3e5991a9106a1d573`. R1 and R2 package archives remain unchanged.
- ABBA report, JSON, summary log, and four raw traces match their supplied hashes. Pooled read gaps >60 ms are 228/1,198 A vs 227/1,198 B, but B max is higher; write-boundary gaps are 228/1,198 in both groups, and B max is higher. The effect is not repeatable.
- New evidence chain: `investigation/evidence/evidence-chain-r4.log`.
- Next: revise mechanism and bounded diagnostic path, assemble package/manifest revision 4, run mechanical self-check only.
