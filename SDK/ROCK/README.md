# ROCK SDK

This SDK is the public v1 ABI surface for FO4VR F4SE integrations with ROCK.

## Files

- `include/ROCKProviderApi.h` is the canonical header.
- `include/ROCKApi.h` is an alias header for the same v1 table.
- `docs/PublicApi.md` documents initialization, ownership, lifecycle, and feature discovery.
- `docs/VersionMatrix.md` records the public API version contract.
- `examples/MinimalProviderConsumer.cpp` shows registration, a frame callback, and owner-filtered contact polling.

## ABI Contract

ROCK v1 exposes one POD/value function table from `ROCK.dll`. Consumers should include only SDK headers and must not depend on private ROCK source headers.

The provider API version remains `ROCK_PROVIDER_API_VERSION == 1` until the public launch contract changes.

Use `RockProviderApi::initialize(rock::provider::ROCK_PROVIDER_API_VERSION)`, then query `getProviderLimitsV1` and feature bits before using optional surfaces.

## Ownership

Register with `registerConsumerV1` and use the returned ROCK-issued owner token for write/control calls. Do not invent global owner tokens. `unregisterConsumerV1` releases that owner's external bodies, offhand reservation, and queued interaction commands.

ROCK v1 exposes queued force grab, force release, and thrown drop through `requestForceGrabV1`, `getInteractionCommandResultV1`, `requestForceReleaseV1`, and `requestThrownDropV1`. Force release defaults to a gentle non-throw drop; force release and thrown drop can apply trusted caller-supplied Havok velocity when their `UseVelocityHavok` flag is set.
