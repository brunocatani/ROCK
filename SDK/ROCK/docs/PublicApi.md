# ROCK Public API

ROCK exposes one v1 C ABI table from `ROCK.dll`:

- `ROCKProviderApi.h`: the public API for FO4VR F4SE plugins.
- `ROCKApi.h`: an alias header for the same v1 API table.

The API is POD/value ABI and avoids private ROCK headers. It is the stable SDK surface for frame snapshots, hand frames, detailed weapon evidence, body contacts, external body registration, owner-filtered external contact polling, offhand reservation, and queued interaction commands.

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
    static_cast<std::uint32_t>(rock::provider::RockProviderConsumerCapabilityV1::ExternalContacts) |
    static_cast<std::uint32_t>(rock::provider::RockProviderConsumerCapabilityV1::InteractionCommands);

rock::provider::RockProviderConsumerHandleV1 handle{};
const auto result = RockProviderApi::inst->registerConsumerV1(&registration, &handle);
```

Use `handle.ownerToken` for provider write/control calls. `handle.grantedCapabilities` is authoritative; unsupported requested capabilities are not granted.

Call `unregisterConsumerV1(ownerToken)` during plugin shutdown. Unregistering clears that owner's external bodies and offhand reservation.

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
- `ConsumerRegistrationV1`
- `OwnerFilteredExternalContactsV1`
- `InteractionCommandQueue`
- `ForceGrabCommand`
- `ForceReleaseCommand`
- `ThrownDropCommand`

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

## Interaction Commands

Interaction commands are queued through ROCK and executed from ROCK-owned update points. Provider API calls validate owner token, capability, struct shape, and queue capacity, then enqueue work; they do not mutate hand/grab state immediately.

To force grab a target, register with `InteractionCommands`, fill `RockProviderForceGrabRequestV1`, and call `requestForceGrabV1`. Poll the returned command id with `getInteractionCommandResultV1` until it reports `Succeeded`, `Rejected`, or `Cancelled`.

Initial force-grab scope:

- Explicit `Left` or `Right` hand only.
- Live loose-object / loose-weapon references only.
- Near force grab only. Targets beyond `maxDistanceGame`, or beyond ROCK's near-grab range when `maxDistanceGame` is zero, are rejected.
- Busy hands, invalid targets, stale generation guards, already-owned targets, missing bodies, and blocked physics writes fail closed.

Successful force grabs enter ROCK's existing dynamic grab path. Finger posing, grab settling, haptics, object ownership, and release behavior are therefore the same systems used by normal grabs.

To force release a held object, fill `RockProviderForceReleaseRequestV1` and call `requestForceReleaseV1`. The selected hand must be `Left` or `Right`. If a target ref, form id, or body id is supplied, the held object must match before ROCK releases it. A force release with no velocity flags is a gentle physical drop: ROCK detaches through the normal release path but does not reuse captured controller throw history.

Set `RockProviderForceReleaseFlagV1::UseVelocityHavok` to apply caller-supplied `linearVelocityHavok` and `angularVelocityRadiansPerSecond` after detach. The supplied force-release velocities are trusted and only finite-checked; ROCK does not clamp them.

To request a thrown drop, fill `RockProviderThrownDropRequestV1` and call `requestThrownDropV1`. Without `UseVelocityHavok`, ROCK captures the current held release motion before detaching. If `UseVelocityHavok` is set, ROCK applies the supplied linear and angular Havok velocities through the same release velocity path after detach. The supplied thrown-drop velocities are trusted and only finite-checked; ROCK does not clamp them.

Successful force release and thrown drop commands use ROCK's existing `Hand::releaseGrabbedObject` path. Collision restore, body lifecycle restore, claim release, release messages, hand pose cleanup, and release events therefore match normal releases.
