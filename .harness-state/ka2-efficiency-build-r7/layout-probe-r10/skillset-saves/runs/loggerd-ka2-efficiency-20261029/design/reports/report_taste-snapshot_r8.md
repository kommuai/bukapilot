---
submission_id: loggerd-ka2-efficiency-20261029-design-r8
revision: r8
---

# Taste snapshot — KA2 rotation liveness, r8

The unconditional Taste resolution stage checked both scopes using `taste_prefs.py`. The project preference store is absent at revision 0, the global store is absent at revision 0, and `effective --scope both` succeeded with an empty resolved profile. This is the sanctioned no-profile case, not a resolution error.

**Source revisions:** project 0 (absent), global 0 (absent). **Resolved entries:** none. **Applicability:** no saved Taste profile is available for this design.

The immutable run artifact is `artifacts/taste_snapshot_r8.json` (SHA-256 `a5de5d229f79066b6fddc255439d4b0d70fb535a039835ff1c41e4073720989f`). It records `applicable: false`, reason `no saved Taste profile available`, scope `KA2 loggerd/encoderd rotation liveness design r8`, and `decided_by: taste`. There is no canonical effective-profile digest because there are no entries to digest. Neither preference store was modified.

| Preference id | Snapshot digest | Design artifact | Provenance | Rendered verification |
| --- | --- | --- | --- | --- |
| None — no effective Taste entries | Not applicable — no saved profile | No user-facing design artifact | Taste resolution; no preferences consumed | Not applicable — backend-only design |

Recheck project and global Taste store revisions immediately before `design-to-build`. If either changes from the absent revision-0 state, invalidate this record and ask Taste to resolve a new immutable snapshot before gate submission.
