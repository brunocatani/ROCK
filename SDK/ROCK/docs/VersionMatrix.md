# ROCK API Version Matrix

Minimum ROCK mod version for this matrix: `0.5.0`.

| API | Public status | Surface |
| --- | --- | --- |
| v1 | Current | Frame snapshots, hand frames, detailed weapon evidence, body contacts, external body registration, owner-filtered external contact polling, offhand reservation, consumer registration, ROCK-issued owner tokens, capability grants, provider limits, feature bits, and queued force-grab / force-release / thrown-drop interaction commands. |

`ROCKProviderApi.h` and `ROCKApi.h` both describe the same v1 ABI table. `ROCKAPI_GetProviderApi` and `ROCKAPI_GetApi` return that same table.

Public force grab is implemented as a queued v1 interaction command that executes through ROCK's existing dynamic grab path. Public force release and thrown drop are queued v1 interaction commands that execute through ROCK's existing release path. Force release defaults to a gentle non-throw physical drop and can accept trusted Havok linear/angular release velocity; thrown drop can use captured release motion or trusted caller-supplied Havok velocity.
