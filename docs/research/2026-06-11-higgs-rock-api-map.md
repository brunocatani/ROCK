# 2026-06-11 HIGGS / ROCK API Map

Project: ROCK

Source authority:

- Current ROCK local source: `F:\fo4dev\PROJECT_ROCK_V2\ROCK`
- Local HIGGS source explicitly requested by user: `F:\fo4dev\skirymvr_mods\source_codes\higgs`
- Current hFRIK provider source for ROCK API dependency context: `F:\fo4dev\PROJECT_ROCK_V2\hFRIK`

Verification method: local source inspection only. No web sources, Ghidra MCP, FO4 Mods MCP, deployed files, or runtime logs were used.

Confidence: high for declared and implemented API surfaces present in the inspected local source. Runtime semantics are summarized only where implementation was inspected in the same local source.

## Purpose

This note maps every public or API-facing surface found in local HIGGS and local ROCK so feature gaps can be discussed from source facts instead of memory or Skyrim VR assumptions.

HIGGS exposes one mod-support C++ vtable API and a Papyrus API. ROCK exposes direct C ABI tables, F4SE messages, and an SDK provider API. ROCK also consumes hFRIK's FRIK API; that is not a ROCK-owned API, but it matters because several HIGGS/VRIK-style use cases are now owned by hFRIK rather than ROCK.

## Acquisition And Export Model

| Surface | How a consumer gets it | Exported DLL symbols | Notes |
| --- | --- | --- | --- |
| HIGGS SKSE plugin | SKSE loads `higgs_vr` | `SKSEPlugin_Query`, `SKSEPlugin_Load` | `exports.def` exposes only SKSE entry points. |
| HIGGS C++ API | Consumer dispatches SKSE message `0xF9279A57` to sender `"HIGGS"` after PostPostLoad and receives `GetApi(1)` | none beyond SKSE entry points | Implemented by `HiggsPluginAPI::ModMessageHandler` and `GetHiggsInterface001`. |
| HIGGS Papyrus API | Registered through SKSE Papyrus interface under script/class name `HiggsVR` | none | Functions are native Papyrus registrations plus queued Papyrus events. |
| HIGGS VRIK client helper | HIGGS dispatches VRIK message `0xF2AFAEE6` to sender `"VRIK"` | none | This is a consumed VRIK API copy, not a HIGGS-provided API. |
| ROCK F4SE plugin | F4SE loads `ROCK.dll` | `F4SEPlugin_Query`, `F4SEPlugin_Load` | `exports.def` exposes F4SE entry points. |
| ROCK legacy/simple API | Consumer calls `GetModuleHandleA("ROCK.dll")` then `GetProcAddress("ROCKAPI_GetApi")` | `ROCKAPI_GetApi` | API version 4. Compatibility/simple surface. |
| ROCK provider API | Consumer calls `GetModuleHandleA("ROCK.dll")` then `GetProcAddress("ROCKAPI_GetProviderApi")` | `ROCKAPI_GetProviderApi` | API version 9. Public SDK foundation. |
| ROCK F4SE messages | Consumer registers with F4SE messaging and listens to ROCK dispatches | none | Message ids are part of `ROCKApi::PhysicsMessage`. |
| ROCK hFRIK dependency | ROCK calls `FRIKAPI_GetApi` from `FRIK.dll` and listens on F4SE sender `"F4VRBody"` | not ROCK-owned | Current provider authority is hFRIK API v5. |

## HIGGS C++ API

Declared interface: `HiggsPluginAPI::IHiggsInterface001`

Implementation: `HiggsPluginAPI::HiggsInterface001`

Acquisition helper: `HiggsPluginAPI::GetHiggsInterface001(pluginHandle, messagingInterface)`

Revision: `1`

Complete vtable entries:

| # | Function | Signature / payload | ROCK nearest surface |
| --- | --- | --- | --- |
| 1 | `GetBuildNumber` | `unsigned int()` | `ROCKApi::getVersion`, `ROCKApi::getModVersion`, `RockProviderApi::getVersion`, `RockProviderApi::getModVersion` |
| 2 | `AddPulledCallback` | `(bool isLeft, TESObjectREFR* pulledRefr)` | F4SE `kOnGrabEvent` with `GrabEventType::PullStarted`, `PullArrived`, `PullCatchAttempt`, `PullCatchSucceeded`; no C callback registration except provider frame callbacks |
| 3 | `AddGrabbedCallback` | `(bool isLeft, TESObjectREFR* grabbedRefr)` | F4SE `kOnGrab`, `kOnGrabEvent::GrabCommitted` |
| 4 | `AddDroppedCallback` | `(bool isLeft, TESObjectREFR* droppedRefr)` | F4SE `kOnRelease`, `kOnGrabEvent::Released` |
| 5 | `AddStashedCallback` | `(bool isLeft, TESForm* stashedForm)` | `GrabEventType::Stashed` exists and ROCK has shoulder stash runtime, but public compatibility depends on current dispatch path; no Papyrus bridge |
| 6 | `AddConsumedCallback` | `(bool isLeft, TESForm* consumedForm)` | `GrabEventType::Consumed` exists and ROCK has mouth consume runtime, but no Papyrus bridge |
| 7 | `AddCollisionCallback` | `(bool isLeft, float mass, float separatingVelocity)` | `GrabEventType::HeldImpact` carries mass/speed/intensity for held-object impact; provider body/external contacts expose richer contact snapshots |
| 8 | `GrabObject` | `(TESObjectREFR* object, bool isLeft)` | No public ROCK force-grab command; v9 feature bits reserve `InteractionCommandQueue`, `ForceGrabCommand`, `ForceReleaseCommand` but ROCK v9 does not set them |
| 9 | `GetGrabbedObject` | `(bool isLeft) -> TESObjectREFR*` | `ROCKApi::getHeldObject` |
| 10 | `IsHandInGrabbableState` | `(bool isLeft) -> bool` | No direct ROCK equivalent; provider hand state exposes holding/touching/disabled flags, but not full grabbability/reason |
| 11 | `DisableHand` | `(bool isLeft)` | `ROCKApi::disablePhysicsHand` |
| 12 | `EnableHand` | `(bool isLeft)` | `ROCKApi::enablePhysicsHand` |
| 13 | `IsDisabled` | `(bool isLeft) -> bool` | `ROCKApi::isPhysicsHandDisabled` |
| 14 | `DisableWeaponCollision` | `(bool isLeft)` | No public ROCK equivalent; ROCK owns weapon collision internally |
| 15 | `EnableWeaponCollision` | `(bool isLeft)` | No public ROCK equivalent |
| 16 | `IsWeaponCollisionDisabled` | `(bool isLeft) -> bool` | No public ROCK equivalent |
| 17 | `IsTwoHanding` | `() -> bool` | `ROCKApi::GrabEventType::TwoHandStarted/TwoHandStopped` are ABI-visible but not currently an implemented public state query |
| 18 | `AddStartTwoHandingCallback` | `() -> void` | No direct ROCK callback; event enum values are reserved |
| 19 | `AddStopTwoHandingCallback` | `() -> void` | No direct ROCK callback; event enum values are reserved |
| 20 | `CanGrabObject` | `(bool isLeft) -> bool` | No direct ROCK equivalent; closest is legacy `isHandHolding`, `getSelectedObject`, disabled state, and provider hand state |
| 21 | `AddCollisionFilterComparisonCallback` | `(void* collisionFilter, UInt32 filterInfoA, UInt32 filterInfoB) -> Continue/Collide/Ignore` | No direct ROCK equivalent; ROCK does not expose a hot collision-filter callback |
| 22 | `AddPrePhysicsStepCallback` | `(void* world)` before `hkpWorld::stepDeltaTime` | No direct public pre-step callback; provider frame callbacks are game/update frame snapshots, not raw world step callbacks |
| 23 | `GetHiggsLayerBitfield` | `() -> UInt64` | No public ROCK layer bitfield getter; internal layer masks are not public ABI |
| 24 | `SetHiggsLayerBitfield` | `(UInt64 bitfield)` | No public ROCK layer bitfield setter; layer policy remains ROCK-owned |
| 25 | `GetHandRigidBody` | `(bool isLeft) -> NiObject*` actually `bhkRigidBody` | Provider exposes hand body IDs and transforms, not live engine body pointers |
| 26 | `GetWeaponRigidBody` | `(bool isLeft) -> NiObject*` actually `bhkRigidBody` | Provider exposes weapon body IDs/evidence, not live engine body pointers |
| 27 | `GetGrabbedRigidBody` | `(bool isLeft) -> NiObject*` actually `bhkRigidBody` | Grab events expose body IDs; no public live body pointer |
| 28 | `ForceWeaponCollisionEnabled` | `(bool isLeft)` | No public ROCK equivalent |
| 29 | `IsHoldingObject` | `(bool isLeft) -> bool` | `ROCKApi::isHandHolding` |
| 30 | `GetFingerValues` | `(bool isLeft, float values[5])` | hFRIK owns pose/finger APIs; ROCK does not expose current finger curl values through its own API |
| 31 | `AddPreVrikPreHiggsCallback` | `() -> void` | No ROCK equivalent; ROCK uses FRIK lifecycle messages and its own main-loop hook |
| 32 | `AddPreVrikPostHiggsCallback` | `() -> void` | No ROCK equivalent |
| 33 | `AddPostVrikPreHiggsCallback` | `() -> void` | No ROCK equivalent |
| 34 | `AddPostVrikPostHiggsCallback` | `() -> void` | No ROCK equivalent |
| 35 | `Deprecated1` | `(std::string_view name, double& out) -> bool` | None; deprecated HIGGS setting getter |
| 36 | `Deprecated2` | `(std::string name, double val) -> bool` | None; deprecated HIGGS setting setter |
| 37 | `GetGrabTransform` | `(bool isLeft) -> NiTransform` | No public ROCK equivalent; ROCK has internal grab-frame telemetry and hand/held authority |
| 38 | `SetGrabTransform` | `(bool isLeft, const NiTransform&)` | No public ROCK equivalent |
| 39 | `GetSettingDouble` | `(const char* name, double& out) -> bool` | No public ROCK config setting API |
| 40 | `SetSettingDouble` | `(const char* name, double val) -> bool` | No public ROCK config setting API |
| 41 | `GetGrabbedNodeName` | `(bool isLeft) -> BSFixedString` | No public ROCK equivalent |

HIGGS callback categories:

- Object lifecycle: pulled, grabbed, dropped, stashed, consumed.
- Collision feedback: hand/held collision callback with mass and separating velocity.
- Two-handing: start and stop callbacks.
- Hot runtime callbacks: collision filter comparison and pre-physics-step callback.
- VRIK/HIGGS ordering callbacks: four no-arg callbacks around VRIK and HIGGS hook ordering.

HIGGS collision-filter callback result enum:

- `Continue`: do not affect collision decision.
- `Collide`: force collision.
- `Ignore`: force no collision.

## HIGGS Papyrus API

Script/class name: `HiggsVR`

Complete registered native functions:

| # | Function | Notes |
| --- | --- | --- |
| 1 | `GetSetting` | Returns numeric setting or `-2.71828f` sentinel on missing key. |
| 2 | `SetSetting` | Writes numeric setting through the same config table as C++ API. |
| 3 | `GrabObject` | Delegates to C++ `GrabObject`. |
| 4 | `GetGrabbedObject` | Delegates to C++ `GetGrabbedObject`. |
| 5 | `GetGrabbedNodeName` | Delegates to C++ `GetGrabbedNodeName`. |
| 6 | `CanGrabObject` | Delegates to C++ `CanGrabObject`. |
| 7 | `DisableHand` | Delegates to C++ `DisableHand`. |
| 8 | `EnableHand` | Delegates to C++ `EnableHand`. |
| 9 | `IsDisabled` | Delegates to C++ `IsDisabled`. |
| 10 | `DisableWeaponCollision` | Delegates to C++ `DisableWeaponCollision`. |
| 11 | `EnableWeaponCollision` | Delegates to C++ `EnableWeaponCollision`. |
| 12 | `IsWeaponCollisionDisabled` | Delegates to C++ `IsWeaponCollisionDisabled`. |
| 13 | `IsTwoHanding` | Delegates to C++ `IsTwoHanding`. |
| 14 | `RegisterForPullEvent` | Registers a `TESForm*` receiver. |
| 15 | `UnregisterForPullEvent` | Unregisters pull receiver. |
| 16 | `RegisterForGrabEvent` | Registers grab receiver. |
| 17 | `UnregisterForGrabEvent` | Unregisters grab receiver. |
| 18 | `RegisterForDropEvent` | Registers drop receiver. |
| 19 | `UnregisterForDropEvent` | Unregisters drop receiver. |
| 20 | `RegisterForStashEvent` | Registers stash receiver. |
| 21 | `UnregisterForStashEvent` | Unregisters stash receiver. |
| 22 | `RegisterForConsumeEvent` | Registers consume receiver. |
| 23 | `UnregisterForConsumeEvent` | Unregisters consume receiver. |
| 24 | `RegisterForStartTwoHandingEvent` | Registers two-hand start receiver. |
| 25 | `UnregisterForStartTwoHandingEvent` | Unregisters two-hand start receiver. |
| 26 | `RegisterForStopTwoHandingEvent` | Registers two-hand stop receiver. |
| 27 | `UnregisterForStopTwoHandingEvent` | Unregisters two-hand stop receiver. |

Papyrus event names HIGGS queues:

- `OnObjectPulled(TESObjectREFR*, bool isLeft)`
- `OnObjectGrabbed(TESObjectREFR*, bool isLeft)`
- `OnObjectDropped(TESObjectREFR*, bool isLeft)`
- `OnObjectStashed(TESForm*, bool isLeft)`
- `OnObjectConsumed(TESForm*, bool isLeft)`
- `OnStartTwoHanding()`
- `OnStopTwoHanding()`

## HIGGS Consumed VRIK API Copy

This is not provided by HIGGS, but HIGGS source includes the VRIK interface helper and uses it after SKSE PostPostLoad.

Complete `IVrikInterface001` functions in the HIGGS tree:

| # | Function | Purpose |
| --- | --- | --- |
| 1 | `getBuildNumber` | VRIK build number. |
| 2 | `getSettingDouble` | Numeric VRIK setting getter. |
| 3 | `setSettingDouble` | Numeric VRIK setting setter. |
| 4 | `getSettingString` | String VRIK setting getter. |
| 5 | `setSettingString` | String VRIK setting setter. |
| 6 | `saveSettings` | Persist settings. |
| 7 | `restoreSettings` | Restore non-numeric settings from disk. |
| 8 | `addGestureAction` | Register gesture action. |
| 9 | `beginGestureProfile` | Begin gesture profile creation. |
| 10 | `setProfileAction` | Bind gesture profile action. |
| 11 | `endGestureProfile` | End gesture profile creation. |
| 12 | `getFingerPos` | Read one finger position, 0 closed to 1 open. |
| 13 | `setFingerRange` | Override min/max finger ranges. |
| 14 | `restoreFingers` | Return finger control to VRIK. |
| 15 | `getCameraOffsettingAmount` | Pre-hook camera offset. |
| 16 | `getFinalCameraOffsettingAmount` | Post-hook camera offset. |
| 17 | `getFinalSmoothingOffsettingAmount` | Post-hook smoothing offset. |

## ROCK Legacy/Simple API

Header: `src/api/ROCKApi.h`

SDK copy: `SDK/ROCK/include/ROCKApi.h`

Export getter: `ROCKAPI_GetApi`

Version: `ROCK_API_VERSION = 4`

Complete function table:

| # | Function | Signature / payload |
| --- | --- | --- |
| 1 | `getVersion` | `() -> uint32_t` |
| 2 | `getModVersion` | `() -> const char*` |
| 3 | `isPhysicsInteractionReady` | `() -> bool` |
| 4 | `getPalmPosition` | `(Hand hand) -> NiPoint3` |
| 5 | `getPalmForward` | `(Hand hand) -> NiPoint3` |
| 6 | `isHandTouching` | `(Hand hand) -> bool` |
| 7 | `getLastTouchedObject` | `(Hand hand) -> TESObjectREFR*` |
| 8 | `getLastTouchedLayer` | `(Hand hand) -> uint32_t` |
| 9 | `isHandHolding` | `(Hand hand) -> bool` |
| 10 | `getHeldObject` | `(Hand hand) -> TESObjectREFR*` |
| 11 | `getSelectedObject` | `(Hand hand) -> TESObjectREFR*` |
| 12 | `disablePhysicsHand` | `(Hand hand) -> void` |
| 13 | `enablePhysicsHand` | `(Hand hand) -> void` |
| 14 | `isPhysicsHandDisabled` | `(Hand hand) -> bool` |
| 15 | `claimPhysicsObject` | `(TESObjectREFR* refr) -> bool` |
| 16 | `releasePhysicsObject` | `(TESObjectREFR* refr) -> bool` |
| 17 | `isPhysicsObjectClaimed` | `(TESObjectREFR* refr) -> bool` |
| 18 | `forceDropObject` | `(Hand hand) -> void` |
| 19 | `getLastTouchedWeaponPartKind` | `() -> uint32_t` |

Legacy ABI implementation note:

- `src/api/ROCKApi.cpp` retains six inert trailing table slots (`legacyUintSlot0`, `legacyUintSlot1`, `legacyUintSlot2`, `legacyBoolSlotWithUint`, `legacyBoolSlot0`, `legacyBoolSlot1`) to preserve older function-table shape after removed PAPER-owned entries. These slots are not in the public header.

ROCK legacy hand enum:

- `Primary`
- `Offhand`
- `Right`
- `Left`

ROCK legacy/simple physics messages:

- `kOnTouch = 100`
- `kOnTouchEnd = 101`
- `kOnGrab = 102`
- `kOnRelease = 103`
- `kOnPhysicsInit = 104`
- `kOnPhysicsShutdown = 105`
- `kOnGrabEvent = 200`

ROCK grab-event types:

- `Unknown = 0`
- `SelectionLocked = 1`
- `PullStarted = 2`
- `PullArrived = 3`
- `PullCatchAttempt = 4`
- `PullCatchSucceeded = 5`
- `GrabCommitted = 6`
- `HeldImpact = 7`
- `Released = 8`
- `TwoHandStarted = 9`
- `TwoHandStopped = 10`
- `StashCandidate = 11`
- `ConsumeCandidate = 12`
- `Stashed = 13`
- `Consumed = 14`
- `LootStarted = 15`
- `LootCompleted = 16`
- `SelectionUnlocked = 17`

Implementation caveat:

- `TwoHandStarted`, `TwoHandStopped`, `LootStarted`, and `LootCompleted` are ABI-visible enum values, but source inspection found them as reserved/scaffold values rather than active dispatch paths.

ROCK grab-event source kinds:

- `Unknown = 0`
- `Hand = 1`
- `HeldObject = 2`
- `PulledObject = 3`
- `Weapon = 4`
- `External = 5`

ROCK grab-event flags:

- `kGrabEventFlagHeldImpactDamped = 1 << 0`
- `kGrabEventFlagSuppressHaptic = 1 << 1`
- `kGrabEventFlagPositionValid = 1 << 2`
- `kGrabEventFlagVelocityValid = 1 << 3`
- `kGrabEventFlagMassValid = 1 << 4`
- `kGrabEventFlagSpeedValid = 1 << 5`
- `kGrabEventFlagIntensityValid = 1 << 6`

ROCK weapon part kinds:

- `Receiver`
- `Barrel`
- `Handguard`
- `Foregrip`
- `Pump`
- `Stock`
- `Grip`
- `Magazine`
- `Magwell`
- `Bolt`
- `Slide`
- `ChargingHandle`
- `BreakAction`
- `Cylinder`
- `Chamber`
- `Shell`
- `Round`
- `LaserCell`
- `Lever`
- `Sight`
- `Accessory`
- `CosmeticAmmo`
- `Other`

ROCK legacy/simple payload structs:

- `PhysicsEventData`: `isLeft`, `refr`, `formID`, `collisionLayer`.
- `GrabEventData`: size/version, event type, source kind, hand, reference/form id, primary and secondary body ids, collision layer, flags, frame index, position, velocity, mass, speed, intensity hint, reserved fields.

## ROCK Provider API

Header: `src/api/ROCKProviderApi.h`

SDK copy: `SDK/ROCK/include/ROCKProviderApi.h`

Export getter: `ROCKAPI_GetProviderApi`

Version: `ROCK_PROVIDER_API_VERSION = 9`

Complete function table:

| # | Function | Signature / payload |
| --- | --- | --- |
| 1 | `getVersion` | `() -> uint32_t` |
| 2 | `getModVersion` | `() -> const char*` |
| 3 | `isProviderReady` | `() -> bool` |
| 4 | `registerFrameCallback` | `(RockProviderFrameCallback callback, void* userData) -> uint64_t token` |
| 5 | `unregisterFrameCallback` | `(uint64_t callbackToken) -> bool` |
| 6 | `getFrameSnapshot` | `(RockProviderFrameSnapshot* outSnapshot) -> bool` |
| 7 | `queryWeaponContactAtPoint` | `(const RockProviderWeaponContactQuery*, RockProviderWeaponContactResult*) -> bool` |
| 8 | `getWeaponEvidenceDescriptors` | `(RockProviderWeaponEvidenceDescriptor* outDescriptors, uint32_t maxDescriptors) -> uint32_t count` |
| 9 | `registerExternalBodies` | `(uint64_t ownerToken, const RockProviderExternalBodyRegistration* bodies, uint32_t bodyCount) -> bool` |
| 10 | `clearExternalBodies` | `(uint64_t ownerToken) -> void` |
| 11 | `getExternalContactSnapshot` | `(RockProviderExternalContact* outContacts, uint32_t maxContacts) -> uint32_t count` |
| 12 | `setOffhandInteractionReservation` | `(uint64_t ownerToken, RockProviderOffhandReservation reservation) -> bool` |
| 13 | `registerExternalBodiesV2` | `(uint64_t ownerToken, const RockProviderExternalBodyRegistration* bodies, uint32_t bodyCount) -> bool` |
| 14 | `getExternalContactSnapshotV2` | `(RockProviderExternalContactV2* outContacts, uint32_t maxContacts) -> uint32_t count` |
| 15 | `getWeaponEvidenceDetailCountV3` | `() -> uint32_t` |
| 16 | `copyWeaponEvidenceDetailsV3` | `(RockProviderWeaponEvidenceDetailV3* outDetails, uint32_t maxDetails) -> uint32_t count` |
| 17 | `getWeaponEvidenceDetailPointCountV3` | `(uint32_t bodyId) -> uint32_t` |
| 18 | `copyWeaponEvidenceDetailPointsV3` | `(uint32_t bodyId, RockProviderPoint3* outPoints, uint32_t maxPoints) -> uint32_t count` |
| 19 | `publishDiagnosticOverlayV4` | `(const RockProviderDiagnosticOverlayFrameV4* frame) -> bool` |
| 20 | `setDiagnosticInputSuppressionV4` | `(uint64_t ownerToken, bool suppressPrimaryTrigger) -> bool` |
| 21 | `getDiagnosticInputSnapshotV5` | `(uint64_t ownerToken, RockProviderDiagnosticInputSnapshotV5* outSnapshot) -> bool` |
| 22 | `setDiagnosticInputSuppressionV5` | `(uint64_t ownerToken, uint32_t suppressionFlags) -> bool` |
| 23 | `getBodyContactSnapshotV6` | `(RockProviderBodyContactV6* outContacts, uint32_t maxContacts) -> uint32_t count` |
| 24 | `getPrimaryHandV8` | `() -> RockProviderHand` |
| 25 | `getOffhandHandV8` | `() -> RockProviderHand` |
| 26 | `getHandFrameV8` | `(RockProviderHand hand, RockProviderHandFrameV8* outFrame) -> bool` |
| 27 | `registerConsumerV9` | `(const RockProviderConsumerRegistrationV9*, RockProviderConsumerHandleV9*) -> RockProviderResultV9` |
| 28 | `unregisterConsumerV9` | `(uint64_t ownerToken) -> RockProviderResultV9` |
| 29 | `getGrantedCapabilitiesV9` | `(uint64_t ownerToken) -> uint32_t capability bits` |
| 30 | `getProviderLimitsV9` | `(RockProviderLimitsV9* outLimits) -> bool` |
| 31 | `getExternalContactSnapshotForOwnerV9` | `(uint64_t ownerToken, RockProviderExternalContactV2* outContacts, uint32_t maxContacts) -> uint32_t count` |

Provider constants and capacities:

- `ROCK_PROVIDER_FRAME_SNAPSHOT_V6_SIZE = 256`
- `ROCK_PROVIDER_MAX_WEAPON_BODIES = 8`
- `ROCK_PROVIDER_MAX_EVIDENCE_NAME = 64`
- `ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V2 = 2048`
- `ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V2 = 512`
- `ROCK_PROVIDER_MAX_DIAGNOSTIC_AXES_V4 = 16`
- `ROCK_PROVIDER_MAX_DIAGNOSTIC_MARKERS_V4 = 16`
- `ROCK_PROVIDER_MAX_DIAGNOSTIC_TEXT_V4 = 8`
- `ROCK_PROVIDER_MAX_DIAGNOSTIC_TEXT_CHARS_V4 = 128`
- `ROCK_PROVIDER_MAX_BODY_CONTACTS_V6 = 128`
- `ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V9 = 16`
- `ROCK_PROVIDER_MAX_CONSUMERS_V9 = 64`

Provider enums:

- `RockProviderHand`: `None`, `Right`, `Left`.
- `RockProviderHandStateFlag`: `None`, `Touching`, `Holding`, `PhysicsDisabled`.
- `RockProviderHandFrameFlagV8`: `None`, `Valid`, `Left`, `Primary`, `Offhand`, `HasSceneNode`, `RootFlattenedAuthority`.
- `RockProviderExternalBodyRole`: `Unknown`, `ReloadMobile`, `ReloadSocket`, `ReloadAction`, `ReloadVisualProxy`, `ActorRagdollBone`.
- `RockProviderExternalBodyContactPolicy`: `None`, `ReportHandContacts`, `ReportAllSourceKinds`, `SuppressRockDynamicPush`.
- `RockProviderExternalSourceKind`: `Unknown`, `Hand`, `Weapon`, `HeldObject`.
- `RockProviderExternalContactQuality`: `BodyPairOnly`, `AggregateImpulse`, `RawPoint`.
- `RockProviderOffhandReservation`: `Normal`, `ReloadReserved`, `ReloadPoseOverride`.
- `RockProviderBodyZoneSide`: `Center`, `Left`, `Right`.
- `RockProviderBodyZoneKind`: `Unknown`, `Pelvis`, `SpineLower`, `SpineUpper`, `Chest`, `NeckHead`, `LeftShoulder`, `LeftUpperArm`, `LeftForearmUpper`, `LeftForearmLower`, `LeftHand`, `RightShoulder`, `RightUpperArm`, `RightForearmUpper`, `RightForearmLower`, `RightHand`, `LeftHip`, `LeftThigh`, `LeftCalf`, `LeftFoot`, `RightHip`, `RightThigh`, `RightCalf`, `RightFoot`.
- `RockProviderBodyContactTargetKind`: `Unknown`, `Hand`, `Weapon`, `HeldObject`, `Body`, `External`, `WorldSurface`, `DynamicProp`, `Actor`, `QueryOnly`.
- `RockProviderLifecycleFlag`: `None`, `WorldAvailable`, `SkeletonReady`, `ProviderReady`, `MenuBlocking`, `ConfigBlocking`, `LoadingOrWorldTransition`, `GeneratedBodiesValid`, `PhysicsWriteAllowed`, `VisualWriteAllowed`.
- `RockProviderLifecycleReason`: `None`, `GameLoaded`, `SkeletonReady`, `SkeletonDestroying`, `PowerArmorChanged`, `WorldAvailable`, `WorldChanged`, `WorldUnavailable`, `ProviderReady`, `ProviderLost`, `MenuBlocked`, `ConfigBlocked`, `GeneratedBodiesRebuilt`, `GeneratedBodiesInvalidated`, `TransitionSettled`, `Shutdown`.
- `RockProviderResultV9`: `Ok`, `NotReady`, `InvalidArgument`, `InvalidSize`, `UnsupportedVersion`, `CapacityFull`, `OwnerNotRegistered`, `OwnerConflict`, `PermissionDenied`, `WorldNotReady`, `TargetInvalid`, `TargetUnavailable`, `HandUnavailable`, `HandBusy`, `ObjectAlreadyOwned`, `RequestQueued`, `RequestRejected`, `RequestNotFound`.
- `RockProviderConsumerCapabilityV9`: `None`, `FrameSnapshots`, `ExternalBodies`, `ExternalContacts`, `OffhandReservation`, `DiagnosticOverlay`, `DiagnosticInput`, `InteractionCommands`.
- `RockProviderFeatureBitV9`: `None`, `FrameCallbacks`, `LifecycleFields`, `HandFramesV8`, `WeaponEvidenceV3`, `BodyContactsV6`, `ExternalContactsV2`, `DiagnosticOverlayV4`, `DiagnosticInputV5`, `ConsumerRegistrationV9`, `OwnerFilteredExternalContactsV9`, `InteractionCommandQueue`, `ForceGrabCommand`, `ForceReleaseCommand`.
- `RockProviderDiagnosticOverlayFlagsV4`: `None`, `DrawAxes`, `DrawMarkers`, `DrawScreenText`.
- `RockProviderDiagnosticInputFlagsV5`: `None`, `PrimaryTriggerHeld`, `PrimaryTriggerPressed`, `PrimaryTriggerReleased`, `RightThumbstickLeftPressed`, `RightThumbstickRightPressed`, `RightThumbstickUpPressed`, `RightThumbstickDownPressed`.
- `RockProviderDiagnosticSuppressionFlagsV5`: `None`, `PrimaryTrigger`, `RightThumbstick`.

Provider structs:

- `RockProviderConsumerRegistrationV9`
- `RockProviderConsumerHandleV9`
- `RockProviderLimitsV9`
- `RockProviderTransform`
- `RockProviderFrameSnapshot`
- `RockProviderHandFrameV8`
- `RockProviderWeaponContactQuery`
- `RockProviderWeaponContactResult`
- `RockProviderWeaponEvidenceDescriptor`
- `RockProviderPoint3`
- `RockProviderBounds3`
- `RockProviderWeaponEvidenceDetailV3`
- `RockProviderBodyContactV6`
- `RockProviderExternalBodyRegistration`
- `RockProviderExternalContact`
- `RockProviderExternalContactV2`
- `RockProviderDiagnosticOverlayAxisV4`
- `RockProviderDiagnosticOverlayMarkerV4`
- `RockProviderDiagnosticOverlayTextV4`
- `RockProviderDiagnosticOverlayFrameV4`
- `RockProviderDiagnosticInputSnapshotV5`

Implemented v9 feature bits from source:

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

Defined but not implemented/set by v9:

- `InteractionCommandQueue`
- `ForceGrabCommand`
- `ForceReleaseCommand`

Implemented v9 capability grants from source:

- `FrameSnapshots`
- `ExternalBodies`
- `ExternalContacts`
- `OffhandReservation`
- `DiagnosticOverlay`
- `DiagnosticInput`

Defined but not granted by v9:

- `InteractionCommands`

## ROCK F4SE Message Surface

ROCK dispatches `PhysicsEventData` through F4SE messaging for:

- touch begin/end;
- grab;
- release;
- physics init/shutdown.

ROCK dispatches `GrabEventData` through F4SE messaging for the richer event stream. Source inspection shows active dispatch paths for selection lock/unlock, pull start/arrival/catch attempt/catch success, grab commit, held impact, release, stash candidate, and consume candidate. Stash/consume transfer also dispatches released events by form id. Two-hand start/stop and loot start/complete are currently enum-visible but not found as active dispatch paths.

## ROCK hFRIK Dependency API

This is not ROCK-owned, but ROCK's `src/api/FRIKApi.h` is byte-for-byte checked against `hFRIK/src/api/FRIKApi.h` by `tests/FrikApiParitySourceTests.ps1`.

Current hFRIK API version: `FRIK_API_VERSION = 5`

Complete function table ROCK expects:

| # | Function | Purpose |
| --- | --- | --- |
| 1 | `getVersion` | API version. |
| 2 | `getModVersion` | FRIK mod version string. |
| 3 | `isSkeletonReady` | Skeleton readiness. |
| 4 | `isConfigOpen` | FRIK UI/config blocking state. |
| 5 | `isSelfieModeOn` | Selfie mode state. |
| 6 | `setSelfieModeOn` | Selfie mode control. |
| 7 | `isOffHandGrippingWeapon` | FRIK offhand weapon grip state. |
| 8 | `isWristPipboyOpen` | Wrist Pip-Boy state. |
| 9 | `getIndexFingerTipPosition` | Finger tip position by hand. |
| 10 | `getHandPoseSetTagState` | Tagged hand-pose override state. |
| 11 | `getCurrentHandPose` | Current hand pose kind. |
| 12 | `setHandPose` | Tagged predefined pose. |
| 13 | `setHandPoseCustomFingerPositions` | Tagged five-finger scalar pose. |
| 14 | `clearHandPose` | Clear tagged pose. |
| 15 | `setHandPoseFingerPositions` | Deprecated untagged scalar pose. |
| 16 | `clearHandPoseFingerPositions` | Deprecated untagged clear. |
| 17 | `registerOpenModSettingButtonToMainConfig` | Add config UI button. |
| 18 | `blockOffHandWeaponGripping` | Tag-based offhand gripping block. |
| 19 | `setHandPoseCustom` | Full 22-float pose. |
| 20 | `setHandPoseWithPriority` | Priority predefined pose. |
| 21 | `getHandWorldTransform` | Final first-person hand transform. |
| 22 | `setHandPoseCustomFingerPositionsWithPriority` | Priority five-finger pose. |
| 23 | `setHandPoseCustomWithPriority` | Priority full 22-float pose. |
| 24 | `applyExternalHandWorldTransform` | Visual hand target authority through FRIK. |
| 25 | `clearExternalHandWorldTransform` | Clear visual hand target. |
| 26 | `setHandPoseCustomLocalTransformsWithPriority` | Priority local finger-bone transform overrides. |
| 27 | `getHandPoseLocalTransformsForPose` | Ask FRIK to generate local transforms for a full pose. |

Implication: HIGGS `GetFingerValues` and VRIK finger range APIs should not automatically be copied into ROCK. Current ROCK architecture routes visual/finger authority through hFRIK v5; any ROCK API addition in this area should expose ROCK-specific contact/finger-collider facts, not duplicate hFRIK's pose API.

## Cross-API Feature Comparison

| Capability | HIGGS | ROCK current | Feature gap / useful direction |
| --- | --- | --- | --- |
| Version/query API | Build number through C++ vtable | Version and mod version through both C ABI tables | ROCK is stronger for ABI versioning. |
| Object grab command | `GrabObject` C++ and Papyrus | No public force-grab; only legacy `forceDropObject` and object claim/release | High-value gap: implement queued, owner-tokened interaction commands instead of immediate foreign writes. |
| Object drop command | No direct drop command beyond release behavior/events | `forceDropObject` immediate/global legacy call | Provider replacement should be queued and owner-tokened. |
| Held object read | `GetGrabbedObject`, `IsHoldingObject` | `getHeldObject`, `isHandHolding`, hand state flags | Covered. |
| Selected object read | No direct selected object getter in HIGGS API | `getSelectedObject` | ROCK is stronger. |
| Hand touch read | Collision callback only | `isHandTouching`, `getLastTouchedObject`, `getLastTouchedLayer` | ROCK is stronger for current touch state. |
| Grabbability read | `IsHandInGrabbableState`, `CanGrabObject` | No direct reason-coded grabbability query | High-value gap: expose can-grab and reason fields through provider snapshots or a query struct. |
| Hand disable | `DisableHand`, `EnableHand`, `IsDisabled` | `disablePhysicsHand`, `enablePhysicsHand`, `isPhysicsHandDisabled` | Covered, but ROCK legacy API is global/unowned. |
| Weapon collision control | Disable/enable/is-disabled/force-enable per hand | No public equivalent | High-value gap if PAPER/weapon mods need controlled weapon-collision leases. Use owner-tokened reservations/leases, not raw toggles. |
| Two-handing state/events | `IsTwoHanding`, start/stop callbacks, Papyrus events | Reserved enum values; no active public query found | High-value gap: expose two-hand grip state, participant hands, weapon part, and start/stop events. |
| Stash/consume events | C++ callbacks and Papyrus events | Event enum values and runtime exist; no Papyrus bridge | Useful gap: make event dispatch compatibility explicit and add Papyrus bridge if quest mods are target consumers. |
| Collision feedback | Simple callback with mass and separating velocity | Held impact events, body contacts, external contacts | ROCK is stronger for structured contact data; HIGGS is simpler for consumers. |
| Collision filter override | Hot callback can force collide/ignore | No public equivalent | Avoid raw hot callback by default; safer alternative is declarative collision policy/lease registration. |
| Pre-physics callback | Raw world pointer before step | Provider frame callback after ROCK frame snapshot; internal physics step coordinator exists | Potential advanced API only if a safe, bounded, documented physics-step callback can be exposed. |
| Layer bitfield control | Get/set HIGGS layer bitfield | No public layer bitfield API | Avoid exposing raw layer masks unless a real external policy owner exists. |
| Live rigid body pointers | Hand, weapon, grabbed rigid bodies | Body IDs, transforms, evidence, contacts | ROCK's value ABI is safer. Pointer APIs should stay internal unless there is a specific trusted plugin need. |
| Grab transform read/write | Get/set hand-to-grab transform | No public equivalent | Useful read-only API for diagnostics; write API should be queued/leased if added. |
| Grabbed node name | `GetGrabbedNodeName`, Papyrus | No public equivalent | Useful low-risk addition: publish held node name/source path in held-object frame/event data if lifetime-safe. |
| Finger values | `GetFingerValues`; HIGGS consumes VRIK finger APIs | hFRIK v5 owns pose/finger APIs; ROCK does not publish finger collision/contact state | Add only ROCK-specific finger contact/collider snapshots if needed. |
| Settings API | Get/set double through C++ and Papyrus | No public config API | Useful for tools/MCM, but must distinguish read-only config, runtime mutable tuning, and production INI edits. |
| Papyrus API | 27 native functions plus 7 event names | None found | High-value gap for quest/ESP interoperability. |
| Diagnostic overlay/input | None in HIGGS API | Provider diagnostic overlay and input suppression/snapshot | ROCK is stronger. |
| External bodies/contacts | None | Provider external body registration, owner-filtered contacts | ROCK is stronger. |
| Weapon semantic evidence | Weapon rigid body pointer only | Weapon contact query, evidence descriptors/details/point clouds, part kinds | ROCK is much stronger. |
| Body contacts/zones | None | Body contact snapshots with zones/sides/target kinds | ROCK is stronger. |
| Consumer ownership/capabilities | Callback vectors, no unregister or owner token | v9 consumer registration, owner tokens, capabilities, owner-filtered contacts | ROCK is stronger and safer. |

## Feature Opportunities That Would Benefit ROCK

### 1. Provider interaction command queue

HIGGS has `GrabObject`; ROCK deliberately does not expose a public force-grab/force-release API yet. ROCK v9 already reserves feature bits and result codes for `InteractionCommandQueue`, `ForceGrabCommand`, and `ForceReleaseCommand`, but the feature bits are unset and no functions are exported.

Recommended ROCK-native shape:

- Append a provider API version with bounded queued command functions.
- Require a registered owner token with `InteractionCommands` granted.
- Accept command structs with `size`, `version`, target ref/form id, hand, optional body id, and request flags.
- Execute from a ROCK-owned safe update point.
- Return `RockProviderResultV9`-style status plus request ids for polling.
- Fail closed on not-ready, hand busy, object already owned, invalid target, world transition, config/menu block, and provider generation mismatch.

This would cover HIGGS `GrabObject`, improve on it with ownership and rollback, and avoid external immediate writes into hand state.

### 2. Reason-coded grabbability and hand interaction state query

HIGGS exposes `IsHandInGrabbableState` and `CanGrabObject`. ROCK consumers currently need to infer this from several weaker facts.

Recommended ROCK-native shape:

- Add a `RockProviderHandInteractionFrame` or extend a future hand-frame version.
- Include current hand state, selected ref/form/body, held ref/form/body, disabled state, selection lock, pull state, candidate stash/consume state, offhand reservation, and a `CanGrab` result code/reason.
- Keep it read-only and value-based.

This gives external mods the convenience HIGGS offers while preserving ROCK's state-machine authority.

### 3. Two-hand grip state and events

HIGGS exposes two-handing state and start/stop callbacks. ROCK has richer two-handed weapon and grab systems, but public enum values are reserved rather than fully surfaced.

Recommended ROCK-native shape:

- Add provider query for two-hand grip state: active, hands involved, weapon form id, support role, weapon part kind/body id, generation.
- Dispatch `TwoHandStarted` and `TwoHandStopped` when runtime ownership actually transitions.
- Include the same data in `GrabEventData` or a provider event extension.

This would benefit PAPER weapon/reload integrations and other weapon support mods.

### 4. Weapon collision lease API

HIGGS has direct per-hand weapon collision disable/enable/force-enable controls. ROCK has richer weapon collider semantics and generated weapon evidence, but no public lease/control surface.

Recommended ROCK-native shape:

- Do not copy HIGGS raw toggles.
- Add owner-tokened weapon collision reservations or leases with explicit role: reload, visual proxy, support grip, scope alignment, debug.
- Include priority/expiry/generation and automatic cleanup on provider loss/unregister.
- Expose read-only weapon collision state in provider snapshots.

This avoids hidden global toggles while supporting PAPER and weapon-mod coordination.

### 5. Held-object frame / grab transform snapshot

HIGGS exposes `GetGrabTransform`, `SetGrabTransform`, and `GetGrabbedNodeName`. ROCK has detailed internal grab-frame telemetry but no stable public held-object frame.

Recommended ROCK-native shape:

- Add a read-only `RockProviderHeldObjectFrame` first.
- Include held ref/form, primary body id, body set ids/count, held node name if copied into a fixed buffer safely, hand-to-held transform, body/world transform, mass, motion mode, and generation.
- Consider write/control only later through the same interaction command queue/lease model.

This would help diagnostics, mod interop, and external visualization without exposing live engine pointers.

### 6. Papyrus bridge for quest and ESP integrations

HIGGS has substantial Papyrus coverage. ROCK has none in inspected source.

Recommended ROCK-native shape:

- Add a small FO4VR Papyrus class only if there are concrete quest/ESP consumers.
- Prefer read-only/event functions first: held object, selected object, disabled state, two-hand state, register/unregister for grab events.
- Gate write operations behind the provider command queue instead of direct immediate state changes.
- Keep naming separate from HIGGS and document FO4VR semantics.

This would make ROCK usable by non-F4SE consumers without weakening C++ ownership.

### 7. Settings and runtime tuning API

HIGGS exposes numeric setting get/set APIs. ROCK currently uses config files and internal runtime config.

Recommended ROCK-native shape:

- Provide read-only config/status query first.
- If runtime writes are needed, expose a whitelist of hot-reload-safe tuning keys with typed units and validation.
- Do not expose arbitrary production INI mutation through public runtime API.

This supports tools/MCM-style integrations without making config lifetime ambiguous.

### 8. Safer collision policy registration instead of raw filter callbacks

HIGGS lets consumers force collide/ignore from a hot filter callback. That is powerful but risky. ROCK already has external body registration and contact policies.

Recommended ROCK-native shape:

- Extend external body registration with declarative contact/collision policies.
- Optionally add owner-tokened suppression/allow rules for specific external body roles, target kinds, or layers.
- Keep the decision data value-based and evaluated inside ROCK-owned hot paths.

This captures the useful part of HIGGS filter callbacks without arbitrary external code executing in collision filtering.

## Bottom Line

ROCK already exceeds HIGGS in structured provider data, lifecycle state, external body/contact interop, weapon semantic evidence, body-zone contacts, diagnostics, and consumer ownership.

The biggest HIGGS-inspired additions that would likely benefit ROCK are:

1. A real provider interaction command queue for force grab/release.
2. Reason-coded grabbability/hand-state snapshots.
3. Public two-hand grip state and start/stop events.
4. Owner-tokened weapon collision leases.
5. Held-object frame and grabbed-node/transform snapshots.
6. A narrow Papyrus bridge for quest/ESP mods.
7. Safe runtime tuning/config query APIs.
8. Declarative collision-policy registration instead of raw collision-filter callbacks.

These should be implemented as FO4VR-native ROCK provider API extensions, not as direct HIGGS-style pointer or hot-callback ports.
