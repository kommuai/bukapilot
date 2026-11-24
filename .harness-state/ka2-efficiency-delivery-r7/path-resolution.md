# Project path relocation record

Recorded: 2026-10-30 16:38:44 UTC

The user directed that the files be moved into the project and checked there.

The previous delivery phase folder was moved from the canonical save location to the archive path below. Its six files retain the same tree SHA-256, so the prior package remains available for history.

- Archived prior delivery: /data/media/supremeteam/42c6eb550c629191187c/skillset-saves/runs/loggerd-ka2-efficiency-20261029/delivery-r6-archive
- Archived file count: 6
- Archived tree SHA-256: 1a1c219baa2425d09532e2b59d7e73cd8fe8e78c804d923cb9a55e90aac36199
- Current delivery package location: /data/openpilot/.harness-state/ka2-efficiency-delivery-r7/delivery
- Canonical delivery path: skillset-saves/runs/loggerd-ka2-efficiency-20261029/delivery
- Canonical path resolution: /data/openpilot/.harness-state/ka2-efficiency-delivery-r7/delivery
- The canonical delivery path now resolves inside /data/openpilot.

The existing build phase folder was also relocated so the next build gate can inspect it inside the project. Its 141 filesystem entries matched the source tree before the original was preserved at the archive path.

- Archived prior build folder: /data/media/supremeteam/42c6eb550c629191187c/skillset-saves/runs/loggerd-ka2-efficiency-20261029/build-media-archive
- Build tree SHA-256: 1ad709e3d6b28711afb976ab5ba0a450c0e1f4f2ae6e37dd24a52db96cc7f33e
- Current build package location: /data/openpilot/.harness-state/ka2-efficiency-build-r7/build
- Canonical build path: skillset-saves/runs/loggerd-ka2-efficiency-20261029/build
- Canonical build path resolution: /data/openpilot/.harness-state/ka2-efficiency-build-r7/build

The run record and design evidence remain in the existing save store. The prior build artifacts and their registered paths still resolve to identical content through the canonical build symlink. No encoder source files were changed and no encoder tests or performance runs were performed.
