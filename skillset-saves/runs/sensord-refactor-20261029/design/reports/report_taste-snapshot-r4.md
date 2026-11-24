---
submission_id: sensord-refactor-20261029-design-r4
revision: r4
---

# Taste snapshot revalidation — r4

**Run:** `sensord-refactor-20261029`  
**Phase:** design  
**Owner:** taste  
**Boundary:** design-to-build  
**Preamble tier:** 2 — read-only profile revalidation for a saved design revision.  
**Decision:** reuse the existing immutable snapshot; no profile drift found.

## Snapshot

- Artifact: `artifacts/taste_snapshot.json`
- File SHA-256: `4b29408d876a909bc0a1eec35f78abf384a72276b6f6972da35e2f99a6244a8f`
- Canonical digest: `sha256:1beb862ffc2faa7ade5f7b2308f2803bc9cea1981dbf2144ce044fa8ebd9132b`
- Snapshot source revisions: global `0`, project `0`
- Snapshot entries: resolved `{}`, shadowed `{}`, conflicts `[]`

The snapshot records `applicable: false` with the sanctioned reason `no saved Taste profile available`, scope `both`, and no resolved profile entries.

## Read-only revalidation

Using the sanctioned `taste_prefs.py` reader from the `/data/openpilot` project root:

- `status --scope global`: absent, revision `0`; status digest `sha256:d3b535d9d53ec95ca1efd0ce50d00e2789045467c5e87cc9124eefd1b4535cb3`
- `status --scope project`: absent, revision `0`; status digest `sha256:4446421026c219de9799227a42252fd845f544146e9fe31a717022af7d24dc94`
- `list --scope global` and `list --scope project`: both entry maps empty
- `effective --scope both`: effective entries empty

Both stores remain absent at their snapshot revisions, and the effective profile remains empty. The status digests differ from the snapshot's saved `source_digests` because `taste_prefs.py` constructs a synthetic blank record with a generated timestamp when a store is absent; the revision, absence, and entries are the stable comparison. This behavior is also recorded in `report_taste-snapshot.md`.

No Taste store or snapshot was changed. No replacement snapshot is needed for r4.
