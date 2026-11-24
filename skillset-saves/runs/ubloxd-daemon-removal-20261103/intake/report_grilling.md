# Intake decision register: UBLOXD and PIGEOND removal

Scope: remove the `ubloxd` and `pigeond` daemons and their daemon-only source, build, dependency, test, and documentation references. Remove this subsystem's TICI manager path, without adding a TICI GNSS replacement. Keep qcomgpsd enabled on KA2 with its existing GNSS assistance behavior and preserve historical UBLOX log schemas/replay support.

## Resolved decisions

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-1 | Daemon removal scope | Remove both daemons and daemon-only implementation/build/test tooling from the repository and manager configuration. | user |
| D-2 | Platform scope after daemon removal | Remove this subsystem's TICI-only manager support, including UBLOX daemons and qcomgpsd's TICI enablement. Keep qcomgpsd enabled on KA2 as it is today; do not add a TICI GNSS replacement. | user |
| D-3 | Historical UBLOX log support | Keep cereal service/schema definitions and legacy replay handling needed to parse old logs. | codebase |
| D-8 | KA2 qcomgpsd behavior | Preserve the existing KA2 qcomgpsd startup and XTRA assistance behavior without changing its provider, transport, or onroad lifecycle. | user |
| D-10 | KA2 live GPS service selection | Clear persistent `UbloxAvailable` at KA2 manager startup so KA2 consumers select `gpsLocation`; preserve external-GPS selection in PC simulation and route-derived values in historical replay. | codebase |

## Deferred decisions

| ID | Decision | Why deferral is safe | Owner | Reopen phase |
| --- | --- | --- | --- | --- |
| D-4 | Remove old UBLOX log schema and replay compatibility | The requested runtime daemons can be removed without changing the stored log format; retain compatibility unless a separate migration is authorized. | project maintainer | future log-format migration |

## Rejected options

| ID | Option | Rationale |
| --- | --- | --- |
| D-5 | Promote qcomgpsd to replace UBLOX on TICI | The user confirmed KA2 as the target and authorized removing this subsystem's TICI manager support rather than creating a TICI GNSS replacement. |
| D-6 | Keep `ubloxd` on TICI | It conflicts with the requested complete daemon removal. |

## Non-goals and YAGNI deferrals

| ID | Non-goal | Reopen trigger | Owner |
| --- | --- | --- | --- |
| D-7 | Remove qcomgpsd from KA2 or change its existing XTRA behavior | The user requests a separate KA2 GNSS change | user |
| D-9 | Remove TICI hardware/platform infrastructure beyond this GNSS manager path | The user explicitly authorizes a broader TICI codebase removal | user |
