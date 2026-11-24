---
submission_id: sensord-refactor-20261029-design-r4
revision: r4
---

# Stack applicability — KA2-only sensord cleanup

**Decision owner:** commander  
**Boundary:** design-to-build  
**Design revision:** r4

## Applicability

This cleanup adds no runtime, framework, dependency, build target, or stack version. It narrows system/sensord to the already-supported KA2 sensors, changes the manager's platform enablement, removes obsolete sensor files and references, and relocates offline tests into an existing CI stage. The generic manager build and root SCons definitions remain unchanged.

The r4 amendment only removes r3's `KA2` startup guard and explicit manager enablement argument, and deletes the tests that existed solely for those checks. The existing no-new-runtime/framework stack decision remains applicable.

The sanctioned typed applicability decision is:

{"applicable":false,"reason":"no new runtime or framework - existing stack unchanged","scope":"design-to-build","decided_by":"commander"}

## Detection limits

The required project detection command was last run from /data/openpilot for design r3. The r4 amendment does not change the runtime, framework, dependency, or generic build surface, so that bounded stack-detection result carries forward:

/usr/local/venv/bin/python3 /data/kommu/.codex/skills/scripts/check_runtime.py --project-root . --detect-project

It reported Python 3.12.3 below the harness minimum 3.13, no supported runtime classification signal, and an incomplete project walk because its total-byte limit was exceeded. Therefore this scan does not establish a repository-wide stack slug or prove that no stack exists. It is not used as the basis for the no-new-runtime fallback.

The repository pyproject.toml declares Python >=3.11,<3.13, so the host interpreter is within the product declaration while still below the harness readiness minimum. Missing Codex hooks are recorded by intake. Neither the environment nor dependency configuration is changed by this design.

The no-new-runtime/framework decision rests on the approved scope and the explicit absence of dependency/build-system changes in the architecture and plan. Any later requirement to change a runtime, framework, dependency, or generic build definition must reopen the stack decision before implementation.
