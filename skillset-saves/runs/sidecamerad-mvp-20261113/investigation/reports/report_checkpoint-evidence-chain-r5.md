# Checkpoint — evidence chain, investigation revision 5

- Reused the accepted five-minute reproduction; no rerun.
- Offline futex analysis used the successful final2 trace; no new capture.
- Temporal same-address association: 598/598 successful output-writer waits >=1 ms had a positive same-address wake call from leader TID 863769 during the wait interval; this does not identify the kernel-awakened waiter.
- Long-gap overlap counts are separate: 133/133 >60 ms gaps overlapped >=1 ms futex waits by each of TIDs 863769, 863808, and 863811; all 133 overlapped at least one futex wait >=10 ms.
- Pairing log SHA-256: `b4da13d544b42b0acb8a1b79c2668ad1fa51af68f6bce1460ccb8f015562ceca`; structured pairing artifact SHA-256: `5c69e8b7f52201fef1ec9a4958bbd3e0b485b1a3f9ec58882847e0e1fdfc07f3`; evidence-chain SHA-256: `88939d93590a991028c66fa788980cbfef5b572152faac320f32a38f06acdf3c`.
- The approved revision-4 package archive remains byte-identical: `39f762cc1f8002a5ea694cf59fe638e359a856ac8e206a8b1c52209900da08bc`.
- Next stage completed in this submission: mechanism/fix path set to internal synchronization correlation with causal ambiguity; bounded GDB/ptrace feasibility check, no source or argv change.
