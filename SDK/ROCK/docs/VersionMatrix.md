# ROCK API Version Matrix

Minimum ROCK mod version for this matrix: `0.5.0`.

| API | Public status | Surface |
| --- | --- | --- |
| v1 | Current | Frame snapshots, hand frames, detailed weapon evidence, body contacts, external body registration, owner-filtered external contact polling, offhand reservation, diagnostics, consumer registration, ROCK-issued owner tokens, capability grants, provider limits, and feature bits. |

`ROCKProviderApi.h` and `ROCKApi.h` both describe the same v1 ABI table. `ROCKAPI_GetProviderApi` and `ROCKAPI_GetApi` return that same table.

Queued interaction commands, public force grab, and public force release are not implemented in v1. Their feature bits are reserved in the header and remain unset by `getProviderLimitsV1`.
