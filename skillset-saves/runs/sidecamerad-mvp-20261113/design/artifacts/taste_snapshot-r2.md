# Effective Taste snapshot — revision 2

Resolved at UTC: 2026-11-13 06:53:27

Result classification: `no_profile` — both Taste stores were readable and absent at resolution time. This is not a lookup failure or a corrupt-store result.

Canonical digest: `sha256:f0e110a84f54c35f9a053dbb2bb9fd20d3477936c57c2b45e450175510a64ab4`

The effective profile is empty and not applicable. Its canonical digest is over `{"entries":{}}`, using Taste canonical JSON serialization (sorted keys, compact separators, trailing newline).

Resolution used the read-only `status`, `list`, and `effective` operations with `--scope both` and project root `/data/openpilot`. Both lists were empty and the merged effective entries were empty. Both stores are absent at revision 0.

The normal project-scope read path refused before loading because `/data/openpilot/skillset-saves` is an existing workspace-local directory. For this read only, the parent-authorized exact-root override checked that the project root is exactly `/data/openpilot`, the save root is the real non-symlink directory `/data/openpilot/skillset-saves`, and the preferences path is contained beneath it. The override did not create, write, or modify preference files. The global store was read from its normal per-user path.

Compared with r1 (`artifacts/taste_snapshot.md`, SHA-256 `98a92bd4d31c1ef3d50097f9e75a26db4a8c1f8121a99bc95971c933043e66e6`), the effective profile remains unchanged: no entries, no applicability, no conflicts, and revision 0 in both scopes. The source-record digests are refreshed because each absent-store resolution uses a newly generated blank record; the revisions and effective canonical digest are unchanged.

```json
{
  "applicable": false,
  "canonical_digest": "sha256:f0e110a84f54c35f9a053dbb2bb9fd20d3477936c57c2b45e450175510a64ab4",
  "decided_by": "taste",
  "reason": "no saved Taste profile available",
  "resolved_entries": {},
  "scope": "both",
  "shadowed_entries": {},
  "source_digests": {
    "global": "sha256:9a2dcfc1bdbfb04d53b756cccb504d7eb257599417ba19b5681ab364c40b3b24",
    "project": "sha256:428c0d9763918b0f7b8229b4ca52fc877e0d6f0cf2c9807cb649c6232c72fd9a"
  },
  "source_revisions": {
    "global": 0,
    "project": 0
  },
  "source_states": {
    "global": "absent",
    "project": "absent"
  },
  "unresolved_conflicts": [],
  "save_location_override": {
    "authorized_for": "read-only project Taste resolution",
    "project_root": "/data/openpilot",
    "save_root": "/data/openpilot/skillset-saves",
    "write_performed": false
  }
}
```
