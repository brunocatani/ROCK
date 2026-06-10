# ROCK Public API

ROCK exposes two C ABI tables from `ROCK.dll`:

- `ROCKProviderApi.h`: the public foundation API for new FO4VR F4SE plugins.
- `ROCKApi.h`: the legacy/simple compatibility API. New integrations should not use it for write/control behavior.

The provider API is POD/value ABI and avoids private ROCK headers. It is the stable SDK surface for frame snapshots, hand frames, weapon evidence, body contacts, external body registration, external contact polling, offhand reservation, diagnostic overlay, and diagnostic input.

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
    const int err = RockProviderApi::initialize(9);
    return err == 0 && RockProviderApi::inst;
}
```

`RockProviderApi::initialize` returns:

- `0`: initialized.
- `1`: `ROCK.dll` is not loaded.
- `2`: `ROCKAPI_GetProviderApi` was not exported.
- `3`: ROCK returned no provider table.
- `4`: the provider table is older than the requested minimum version.

Consumers may call read-only queries from ordinary F4SE plugin code, but frame-sensitive decisions should be made from a ROCK provider frame callback. Runtime writes and control calls must be treated as ROCK-owned work and should use a ROCK-issued owner token.

## Consumer Ownership

Provider API v9 adds ROCK-issued owner tokens:

```cpp
rock::provider::RockProviderConsumerRegistrationV9 registration{};
registration.version = rock::provider::ROCK_PROVIDER_API_VERSION;
std::snprintf(registration.modName, sizeof(registration.modName), "MyPlugin");
registration.requestedCapabilities =
    static_cast<std::uint32_t>(rock::provider::RockProviderConsumerCapabilityV9::FrameSnapshots) |
    static_cast<std::uint32_t>(rock::provider::RockProviderConsumerCapabilityV9::ExternalBodies) |
    static_cast<std::uint32_t>(rock::provider::RockProviderConsumerCapabilityV9::ExternalContacts);

rock::provider::RockProviderConsumerHandleV9 handle{};
const auto result = RockProviderApi::inst->registerConsumerV9(&registration, &handle);
```

Use `handle.ownerToken` for provider write/control calls. `handle.grantedCapabilities` is authoritative; unsupported requested capabilities are not granted. `InteractionCommands` is reserved for a future queued command API and is not granted by v9.

Call `unregisterConsumerV9(ownerToken)` during plugin shutdown. Unregistering clears that owner's external bodies, offhand reservation, and diagnostic input suppression.

Existing sibling plugins may still use their historical caller-owned tokens through v8 functions. New public integrations should register and must not invent global owner tokens.

## Provider Limits And Features

Call `getProviderLimitsV9` to discover current limits and feature bits. Do not hardcode queue or buffer sizes beyond the values returned by ROCK.

Implemented v9 feature bits:

- `FrameCallbacks`
- `LifecycleFields`
- `HandFramesV8`
- `WeaponEvidenceV3`
- `BodyContactsV6`
- `ExternalContactsV2`
- `DiagnosticOverlayV4`
- `DiagnosticInputV5`
- `ConsumerRegistrationV9`
- `OwnerFilteredExternalContactsV9`

The interaction command feature bits are defined in the header but are not set by ROCK v9.

## Frame And Lifecycle Rules

Use `registerFrameCallback` to receive `RockProviderFrameSnapshot` from a ROCK-owned update point. The snapshot includes world pointers as integer addresses, provider readiness, menu/config blocking, lifecycle flags, world/skeleton/provider generations, weapon body IDs, hand transforms, hand state flags, and the current offhand reservation.

Before writing physics or visuals, check the lifecycle flags:

- `PhysicsWriteAllowed` must be set for physics-affecting work.
- `VisualWriteAllowed` must be set for visual authority work.
- Treat missing world, missing skeleton, provider loss, menu blocking, config blocking, and transition flags as fail-closed conditions.

Generation fields are guards. Cache them only long enough to validate same-frame or queued work.

## External Bodies And Contacts

Register external hknp body IDs with `registerExternalBodiesV2` using the ROCK-issued owner token. Bodies are replaced per owner; explicit clear/unregister drops the owner's registrations and pending contacts.

Use `getExternalContactSnapshotForOwnerV9` for public integrations. It returns only contacts targeting bodies registered by that owner. The older global contact snapshots remain for compatibility but require consumers to filter by owner.

## Offhand Reservation

`setOffhandInteractionReservation` remains compatible with current PAPER usage. Public consumers should use their registered owner token and should release the reservation by setting `Normal` when finished. Lease priority and expiry are not public in v9.

## Diagnostics

Diagnostic overlay frames and diagnostic input suppression should use registered owner tokens. Diagnostic input suppression is cleared automatically on `unregisterConsumerV9`.

## Interaction Commands

ROCK v9 does not expose public force-grab or force-release commands. The legacy `ROCKApi::forceDropObject` remains available for older consumers, but it is immediate, global, unowned, and not recommended for new public mods.

The planned public command model is a bounded ROCK-owned queue executed from a safe update point, with tokenized ownership and result polling. Do not implement external immediate grab/release behavior against ROCK internals.
