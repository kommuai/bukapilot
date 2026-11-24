# Effective Taste snapshot

Result classification: `no_profile` — both Taste stores were readable and absent at resolution time. This is not a lookup failure or a corrupt-store result.

Canonical digest: `sha256:f0e110a84f54c35f9a053dbb2bb9fd20d3477936c57c2b45e450175510a64ab4`

The canonical digest is over the effective profile returned by `taste_prefs.py effective --scope both`, represented as `{"entries":{}}` and serialized with Taste's canonical JSON rules (sorted keys, compact separators, trailing newline). Source-store digests are recorded separately as resolution provenance.

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
    "global": "sha256:ca5b4a3db4e56c107e8abacc63a03c8fa58cc63aa01ae5b22df2c57fa155000f",
    "project": "sha256:13ca8850ff197e358752e5b87eb1f4bb4ed6ea7dd4c8c0f8ddc5fafff45090d3"
  },
  "source_revisions": {
    "global": 0,
    "project": 0
  },
  "source_states": {
    "global": "absent",
    "project": "absent"
  },
  "unresolved_conflicts": []
}
```

Resolution used the read-only `status`, `list`, and `effective` operations with project root `/data/openpilot`. Both list results were empty; the merged effective entries were empty. No preference store was modified.
