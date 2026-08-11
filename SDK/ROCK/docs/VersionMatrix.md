# API version matrix

Minimum provider mod version represented by this SDK snapshot: `0.5.0`.

| API | Status | Binary contract | Current surface |
| --- | --- | --- | --- |
| V1 | Current | Append-only, 90 x64 function pointers / 720 bytes | Complete public hand, weapon, contact, input, animation, authority, command, scoped publication, touch-grab, raycast, and diagnostic surface. |

## Compatibility rules

`ROCK_PROVIDER_API_VERSION` remains `1` while new functions are appended and structures are prefix-extended. A V1 version match alone does not prove that a newer slot exists.

Consumers requiring a newer family should initialize with its named `ROCK_PROVIDER_API_V1_*_TABLE_BYTES` constant. Consumers supporting multiple provider revisions can initialize V1 without a minimum extent and then use `getProviderLimitsV1`/`getProviderLimitsExtV1` plus the inline `supports...V1` helpers.

The safe descriptor is the only supported way to require a nonzero minimum table extent. Legacy provider-table discovery remains available for consumers that use only the original V1 prefix.

Public structures carry `size`/`version` where extensibility requires it. Initialize structures with their default constructor, preserve zeroed reserved fields, and use `getPublicStructureSizeV1` when adapting to another header revision.

## Provider and game versions

The SDK provider mod version and the API version are separate values. The example F4SE plugins additionally gate FO4VR identity and `Fallout4VR.exe` file version `1.2.72.0`; they do not confuse that executable version with F4SEVR's loader compatibility runtime.
