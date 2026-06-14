# ROCK Public API

ROCK exposes one v1 C ABI table from `ROCK.dll`:

- `ROCKProviderApi.h`: the public API for FO4VR F4SE plugins.
- `ROCKApi.h`: an alias header for the same v1 API table.

The API is POD/value ABI and avoids private ROCK headers. It is the stable SDK surface for frame snapshots, hand frames, detailed weapon evidence, body contacts, external body registration, owner-filtered external contact polling, offhand reservation, diagnostic overlay, and diagnostic input.

## Initialization

Include the SDK header shipped at:

```text
SDK/ROCK/include/ROCKProviderApi.h
```

Initialize after `ROCK.dll` is loaded:

```cpp
#include "ROCKProviderApi.h"

using rock::provider::RockProviderApi;

bool initRock()
{
    const int err = RockProviderApi::initialize(rock::provider::ROCK_PROVIDER_API_VERSION);
    return err == 0 && RockProviderApi::inst;
}
```

`RockProviderApi::initialize` returns:

- `0`: initialized.
- `1`: `ROCK.dll` is not loaded.
- `2`: `ROCKAPI_GetProviderApi` was not exported.
- `3`: ROCK returned no provider table.
- `4`: the provider table is below the requested minimum version.

Consumers may call read-only queries from ordinary F4SE plugin code, but frame-sensitive decisions should be made from a ROCK provider frame callback. Runtime writes and control calls must be treated as ROCK-owned work and should use a ROCK-issued owner token.

## Consumer Ownership

API v1 uses ROCK-issued owner tokens:

```cpp
rock::provider::RockProviderConsumerRegistrationV1 registration{};
registration.version = rock::provider::ROCK_PROVIDER_API_VERSION;
std::snprintf(registration.modName, sizeof(registration.modName), "MyPlugin");
registration.requestedCapabilities =
    static_cast<std::uint32_t>(rock::provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
    static_cast<std::uint32_t>(rock::provider::RockProviderConsumerCapabilityV1::ExternalBodies) |
    static_cast<std::uint32_t>(rock::provider::RockProviderConsumerCapabilityV1::ExternalContacts);

rock::provider::RockProviderConsumerHandleV1 handle{};
const auto result = RockProviderApi::inst->registerConsumerV1(&registration, &handle);
```

Use `handle.ownerToken` for provider write/control calls. `handle.grantedCapabilities` is authoritative; unsupported requested capabilities are not granted. `InteractionCommands` is reserved for a future queued command API and is not granted by v1.

Call `unregisterConsumerV1(ownerToken)` during plugin shutdown. Unregistering clears that owner's external bodies, offhand reservation, and diagnostic input suppression.

Sibling plugins should register and must not invent global owner tokens.

## Provider Limits And Features

Call `getProviderLimitsV1` to discover current limits and feature bits. Do not hardcode queue or buffer sizes beyond the values returned by ROCK.

Implemented v1 feature bits:

- `FrameCallbacks`
- `LifecycleFields`
- `HandFrames`
- `WeaponEvidence`
- `BodyContacts`
- `ExternalContacts`
- `DiagnosticOverlay`
- `DiagnosticInput`
- `ConsumerRegistrationV1`
- `OwnerFilteredExternalContactsV1`

The interaction command feature bits are defined in the header but are not set by ROCK v1.

## Frame And Lifecycle Rules

Use `registerFrameCallback` to receive `RockProviderFrameSnapshot` from a ROCK-owned update point. The snapshot includes world pointers as integer addresses, provider readiness, menu/config blocking, lifecycle flags, world/skeleton/provider generations, weapon body IDs, hand transforms, hand state flags, and the current offhand reservation.

Before writing physics or visuals, check the lifecycle flags:

- `PhysicsWriteAllowed` must be set for physics-affecting work.
- `VisualWriteAllowed` must be set for visual authority work.
- Treat missing world, missing skeleton, provider loss, menu blocking, config blocking, and transition flags as fail-closed conditions.

Generation fields are guards. Cache them only long enough to validate same-frame or queued work.

## External Bodies And Contacts

Register external hknp body IDs with `registerExternalBodiesV1` using the ROCK-issued owner token. Bodies are replaced per owner; explicit clear/unregister drops the owner's registrations and pending contacts.

Use `getExternalContactSnapshotForOwnerV1` for integrations. It returns only contacts targeting bodies registered by that owner.

## Offhand Reservation

`setOffhandInteractionReservation` should use a registered owner token and should release the reservation by setting `Normal` when finished. Lease priority and expiry are not public in v1.

## Diagnostics

Diagnostic overlay frames and diagnostic input suppression should use registered owner tokens. Diagnostic input suppression is cleared automatically on `unregisterConsumerV1`.

## Interaction Commands

ROCK v1 does not expose public force-grab or force-release commands.

The planned public command model is a bounded ROCK-owned queue executed from a safe update point, with tokenized ownership and result polling. Do not implement external immediate grab/release behavior against ROCK internals.
