# Focused recipes

The buildable example plugins contain full lifecycle code. These fragments show the essential call shapes and assume a valid owner callback, owner token, granted capability, table guard, and current snapshot.

## Queue a force grab

```cpp
RockProviderForceGrabRequestV1 request{};
request.hand = RockProviderHand::Right;
request.targetFormId = targetFormId;
request.targetBodyId = targetBodyId;
request.worldGeneration = snapshot.worldGeneration;
request.skeletonGeneration = snapshot.skeletonGeneration;
request.providerGeneration = snapshot.providerGeneration;
request.maxDistanceGame = 32.0f;

std::uint64_t commandId{};
const auto admitted = RockProviderApi::inst->requestForceGrabV1(
    ownerToken,
    &request,
    &commandId);
```

Store `commandId` only when `admitted == RockProviderResultV1::Ok`. Poll slot 24 until the command is terminal; cancel it during feature shutdown when possible.

## Register bodies in a replaceable scope

```cpp
RockProviderExternalBodyRegistration body{};
body.bodyId = myLiveBodyId;
body.ownerToken = ownerToken;
body.generation = myBodyGeneration;
body.role = RockProviderExternalBodyRole::ActorRagdollBone;
body.contactPolicy =
    RockProviderExternalBodyContactPolicy::ReportAllSourceKinds;

(void)RockProviderApi::inst->registerExternalBodiesForScopeV1(
    ownerToken,
    scopeToken,
    &body,
    1);
```

Replace the scope when its body set changes and clear it before those bodies become invalid. Consume contacts with the cursor-based slot 63.

## Refresh input suppression

```cpp
RockProviderHandInputSuppressionRequestV1 request{};
request.hand = RockProviderHand::Left;
request.flags = static_cast<std::uint32_t>(
    RockProviderHandInputSuppressionFlagV1::SuppressNormalGrabPress);
request.leaseFrames = 2;
request.worldGeneration = snapshot.worldGeneration;
request.skeletonGeneration = snapshot.skeletonGeneration;
request.providerGeneration = snapshot.providerGeneration;

(void)RockProviderApi::inst->setHandInputSuppressionV1(
    ownerToken,
    &request);
```

Refresh only while the feature owns the interaction, then call slot 28 immediately.

## Publish a two-hand surface field

Use two `FixedAnchor` wildcard descriptors, one with `AllowRightHand` and one with `AllowLeftHand`. Give them distinct target IDs, the same short rolling lease, `MatchAnyBody`, the allowed motion flags, an intentional collision-layer mask, and current world/skeleton/provider generations. The complete pattern is `examples/mods/SurfaceClimber.cpp`.

Avoid one `AllowTwoHands` wildcard when each hand must be able to resolve a different surface: one wildcard descriptor owns at most one resolved body.

## Drive one gripped weapon part

Publish a non-exclusive target once for the current weapon generation so the
selected part gains `AttachOnly` grip behavior without disabling ordinary grip
behavior on every unmatched part:

```cpp
RockProviderWeaponPartTargetV1 target{};
target.flags =
    static_cast<std::uint32_t>(
        RockProviderWeaponPartTargetFlagV1::MatchBodyId) |
    static_cast<std::uint32_t>(
        RockProviderWeaponPartTargetFlagV1::NonExclusive);
target.grabMode = RockProviderWeaponPartGrabModeV1::AttachOnly;
target.weaponGenerationKey = snapshot.weaponGenerationKey;
target.bodyId = selectedPose.bodyId;
target.groupId = 1;
target.priority = 100;

(void)RockProviderApi::inst->setWeaponPartTargetsV1(
    ownerToken, &target, 1);
```

While the matching `getWeaponPartGripStateV1` report is active, refresh a
short drive lease from your bounded motion solver:

```cpp
RockProviderWeaponPartDriveTargetV1 drive{};
drive.flags = static_cast<std::uint32_t>(
    RockProviderWeaponPartTargetFlagV1::MatchBodyId);
drive.driveSpace = RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal;
drive.weaponGenerationKey = snapshot.weaponGenerationKey;
drive.bodyId = selectedPose.bodyId;
drive.groupId = 1;
drive.priority = 100;
drive.leaseFrames = 2;
drive.targetTransform = solvedWeaponRootLocalPose;

(void)RockProviderApi::inst->setWeaponPartDriveTargetsV1(
    ownerToken, &drive, 1);
```

Read `copyWeaponPartDriveApplicationResultsV1` inside the owner callback when
you need to distinguish applied, rejected, or superseded drives. Stop refreshing
and call `clearWeaponPartDriveTargetsV1` as soon as the grip, weapon generation,
motion path, or activation policy is no longer current. Clear the retained part
targets before shutdown. The complete buildable pattern is
`examples/mods/WeaponPartDriver.cpp`.

## Run a bounded world raycast

```cpp
RockProviderWorldRaycastRequestV1 request{};
request.startGame = origin;
request.directionGame = normalizedDirection;
request.maxDistanceGame = 2048.0f;
request.worldGeneration = snapshot.worldGeneration;
request.skeletonGeneration = snapshot.skeletonGeneration;
request.providerGeneration = snapshot.providerGeneration;

RockProviderWorldRaycastResultV1 result{};
const auto queryResult = RockProviderApi::inst->queryWorldRaycastV1(
    ownerToken,
    &request,
    &result);
```

Issue the call only inside the owner callback and stay within `maxWorldRaycastsPerOwnerPerFrame`.

## Publish overlay diagnostics

Build bounded arrays of `RockProviderDebugOverlayLineV1` and `RockProviderDebugOverlayTextV1`, point a `RockProviderDebugOverlayPublicationV1` at them, copy current generations, and use a short lease. ROCK copies the publication during the call. See `examples/mods/ContactVisualizer.cpp`.

## Safely hand a touch mechanism to another system

Call `requestTouchGrabYieldV1(ownerToken, scopeToken, targetId, targetGeneration)`. Stop accepting new work for that target immediately, but do not move it yet. Poll slot 84 until phase is `Yielded`; only then begin native/scripted motion. Remove or republish with a new generation to establish the next ownership cycle.
