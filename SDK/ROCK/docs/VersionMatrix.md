# ROCK Provider API Version Matrix

Minimum ROCK mod version for this matrix: `0.5.0`.

| API | Public status | Added surface | Known consumers |
| --- | --- | --- | --- |
| v1-v2 | Internal/early | Initial provider snapshots and basic external body/contact support. | ROCK development builds |
| v3 | Compatibility | Weapon evidence detail counts and point-cloud copy functions. | PAPER reload evidence import |
| v4 | Compatibility | Diagnostic overlay publication. | PAPER probe tooling |
| v5 | Compatibility | Diagnostic input snapshot and suppression flags. | PAPER probe tooling |
| v6 | Compatibility | Body contact snapshots with body-zone metadata. | SCISSORS contact/impulse processing |
| v7 | Compatibility | Lifecycle and generated-body readiness fields through the frame snapshot. | PAPER, SCISSORS |
| v8 | Compatibility | Primary/offhand mapping and per-hand root-flattened frame snapshots. | PAPER reload authority |
| v9 | Public foundation | Result codes, consumer registration, ROCK-issued owner tokens, granted capabilities, provider limits, feature bits, and owner-filtered external contacts. | Public SDK consumers |

## v9 Added Types

- `RockProviderResultV9`
- `RockProviderConsumerCapabilityV9`
- `RockProviderFeatureBitV9`
- `RockProviderConsumerRegistrationV9`
- `RockProviderConsumerHandleV9`
- `RockProviderLimitsV9`

## v9 Added Functions

The v9 functions are appended after `getHandFrameV8` in `RockProviderApi`:

- `registerConsumerV9`
- `unregisterConsumerV9`
- `getGrantedCapabilitiesV9`
- `getProviderLimitsV9`
- `getExternalContactSnapshotForOwnerV9`

## Not Yet Public

Queued interaction commands, public force grab, and public force release are not implemented in v9. Their feature bits are reserved in the header and remain unset by `getProviderLimitsV9`.
