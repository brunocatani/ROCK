# SCISSORS Active Ragdoll Port Context

**Date:** 2026-05-02  
**Workspace:** `E:\fo4dev\PROJECT_ROCK_V2`  
**Reference source:** `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll`  
**Target mod:** `SCISSORS`, a sibling/daughter FO4VR plugin that hard-depends on ROCK  
**Status:** Provider v3 and the SCISSORS milestone scaffold are implemented. Actor lifecycle and point impulses are present behind explicit config gates; in-game acceptance is still pending.

## Purpose

This document preserves the planning context for porting the Skyrim VR ActiveRagdoll/PLANCK codebase to Fallout 4 VR as SCISSORS. It is intended as memory for future sessions and as a tracking document before implementation begins.

The core design decision is that SCISSORS must not be a direct hkp-to-hknp transcription of PLANCK. PLANCK is the behavioral reference. ROCK is the player-hand, weapon, held-object, collision, and grab evidence provider. FO4VR's hknp runtime and verified ragdoll pipeline are the execution target.

The architecture is being split this way because ROCK already owns high-quality player-originated physics evidence: hand collider bodies on layer 43, weapon collider bodies on layer 44, held-object body tracking, contact subscription, and provider snapshots. SCISSORS should consume that evidence and own actor-ragdoll lifecycle plus actor force policy. This avoids duplicate collision listeners and avoids coupling active-ragdoll control into ROCK's grab system.

## Implementation Update - 2026-05-02

The first implementation was built as an additive provider boundary plus a separate SCISSORS consumer because ROCK already owns the player-originated collider and grab evidence, while active actor-ragdoll lifecycle needs to remain isolated from ROCK's hand/object control loop. The rejected alternatives were direct PLANCK hkp transcription and a duplicated SCISSORS contact listener; both would couple SCISSORS to FO4VR contact internals before the raw point payload is fully verified and would make double-force prevention harder.

### ROCK Additions

- Added ROCK provider API v2 while preserving the existing provider table shape and v1 functions.
- Added `RockProviderExternalBodyRole::ActorRagdollBone`.
- Added source kind metadata for hand, weapon, and held-object contacts.
- Added contact quality metadata: `BodyPairOnly`, `AggregateImpulse`, and `RawPoint`.
- Added external contact policy flags for all-source contact reporting and ROCK dynamic-push suppression.
- Expanded the external body and contact capacities to v2 limits:
  - `ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V2 = 2048`
  - `ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V2 = 512`
- Added `registerExternalBodiesV2` and `getExternalContactSnapshotV2`.
- Extended the external body registry to publish v2 contacts with owner token, generation, role, sequence, source kind, body IDs, source velocity, quality, and optional point/normal data.
- Added suppression checks so ROCK's dynamic push assist skips bodies registered by SCISSORS with `SuppressRockDynamicPush`.
- Routed ROCK hand, weapon, and held-object contact evidence into the provider v2 contact ring when the opposing body is registered as an external SCISSORS actor-ragdoll body.

Current contact quality state:

- ROCK now publishes all relevant source kinds for registered external bodies.
- ROCK now calls Bethesda's own FO4VR contact-signal point extraction helper at `0x14175c650`.
- If that helper returns a finite point and normal, ROCK publishes provider contacts as `RawPoint` and fills `contactPointHavok`, `contactNormalHavok`, and `aggregateImpulseMagnitude`.
- If extraction fails, ROCK preserves the previous fail-closed behavior and publishes `BodyPairOnly`.
- SCISSORS still requires `bEnablePointImpulses=true` before it applies impulses, and it continues to reject non-`RawPoint` contacts.

ROCK refactor interaction:

- `PhysicsInteraction.h` already had the concurrent `PhysicsFrameContext` refactor shape.
- The implementation side still had the old `resolveContacts(RE::bhkWorld*, RE::hknpWorld*)` definition, which caused the ROCK Release build to fail.
- The only refactor-adjacent touch made for this work was aligning the definition to `resolveContacts(const PhysicsFrameContext& frame)` and reading `bhkWorld/hknpWorld` from that context. No refactor files were reverted or replaced.

### SCISSORS Project

Created sibling project:

- `SCISSORS/CMakeLists.txt`
- `SCISSORS/CMakePresets.json`
- `SCISSORS/CMakeUserPresets.json`
- `SCISSORS/vcpkg.json`
- `SCISSORS/cmake/*`
- `SCISSORS/data/config/SCISSORS.ini`
- `SCISSORS/src/PCH.h`
- `SCISSORS/src/SCISSORSMain.cpp`
- `SCISSORS/src/scissors/*`
- `SCISSORS/tests/*`

Implemented subsystems:

- `ProviderDependency`: maps ROCK provider init failures into explicit SCISSORS dependency states.
- `RockProviderClient`: resolves ROCK provider API v3, registers the ROCK frame callback, polls v2 contacts, registers actor-ragdoll external bodies, and clears owner registrations.
- `ScissorsConfig`: reads and hot-reloads `C:\Users\SENECA\Documents\My Games\Fallout4VR\SCISSORS_Config\SCISSORS.ini`.
- `ScissorsRuntime`: runs from the ROCK frame callback, resets on provider/world readiness loss, drives actor runtime updates, and consumes v2 contacts.
- `ActorSelectionPolicy`: implements the PLANCK-style start/end distance gates, actor add cadence, max-active cap, and validity filters.
- `RagdollAccess`: wraps the currently verified FO4VR actor graph to ragdoll driver to hknp ragdoll body-map path.
- `RagdollBodyPolicy`: validates body candidates and builds provider registrations using `ActorRagdollBone`, `ReportContacts`, `ReportAllSourceKinds`, and `SuppressRockDynamicPush`.
- `ActorRagdollRuntime`: scans `ProcessLists::highActorHandles`, collects valid non-player actors, maps ragdoll bodies, registers them with ROCK, and clears stale owner tokens.
- `PointImpulsePolicy`: evaluates contacts by source kind, velocity, cooldown, clamp settings, and contact quality.

Default gates in `SCISSORS.ini`:

- `bEnableActorLifecycle = false`
- `bEnablePointImpulses = false`
- `bEnableDamageEvents = false`
- `bEnableMotorControl = false`

These gates are explicit runtime controls, not missing structure. The actor lifecycle and impulse scaffolding are compiled in, but actor registration and force application require config opt-in plus verified runtime evidence.

First in-game raw-point test profile:

- Set `bEnableActorLifecycle = true`.
- Keep `bEnablePointImpulses = false` for the first run and confirm actor registrations plus `RawPoint` provider contacts in logs.
- Then set `bEnablePointImpulses = true` with the current conservative clamps:
  - `fMinSourceSpeedHavok = 0.35`
  - `fMaxImpulseHavok = 2.0`
  - `fSourceTargetCooldownSeconds = 0.08`
  - `iMaxContactsPerFrame = 128`
- Leave `bEnableDamageEvents = false` and `bEnableMotorControl = false`.

Deployment note:

- `SCISSORS.dll` and `SCISSORS.pdb` were copied to `D:\FO4\mods\SCISSORS\F4SE\Plugins`.
- Because no active SCISSORS config existed, `C:\Users\SENECA\Documents\My Games\Fallout4VR\SCISSORS_Config\SCISSORS.ini` was created from the packaged config.
- The active test INI currently has both `bEnableActorLifecycle = true` and `bEnablePointImpulses = true`; damage events and motor control remain disabled.
- If the first in-game run needs lifecycle-only telemetry, flip only `bEnablePointImpulses = false` in that active INI.

### Additional FO4VR Ghidra Evidence

This implementation used the planning anchors above and the following extra binary checks:

| Address | Finding | Use in implementation |
|---|---|---|
| `0x1415481d0` | Body-scoped hknp world event signal lookup locks around the world dispatcher path. | Confirmed body-scoped signal dispatch exists, but did not prove a raw point payload layout for ROCK's current callback. |
| `0x1403b9e50` | Callback subscription helper allocates a Havok callback slot and stores member callback data. | Confirmed callback registration ABI shape for future body-scoped contact work. |
| `0x1416a3ee0` | `hkbnpPhysicsInterface::preStep` calls through the ragdoll driver pointer at offset `+0x208`. | Supports the actor graph to ragdoll driver wrapper path. |
| `0x14177e3d0` | `hkbRagdollDriver::setRagdoll` uses ragdoll interface at driver `+0x70` and pending add flag at `+0xB4`. | Supports the driver to ragdoll-interface lifecycle assumptions. |
| `0x1416aeb70` | BShkb animation graph vfunction reads the graph driver at `+0x208`. | Supports the graph driver offset used by SCISSORS wrappers. |
| `0x1417b6960` | `hkbnpRagdollInterface` constructor stores `hknpRagdoll*` at interface `+0x10`; body-map data/count are at ragdoll `+0x48/+0x50`. | Supports SCISSORS body-map reads. |
| `0x14175c650` | FO4VR helper decodes a contact signal's manifold pointer into contact count, normal, and up to four world-space contact points. | ROCK now uses this helper instead of reverse-copying the internal single/multi-point manifold layout. |
| `0x141916c20` | Dispatches extraction through the single-point or multi-point manifold path based on `world+0x310`. | Confirms `0x14175c650` is the stable wrapper around both contact manifold cases. |
| `0x1417339f0` | Multi-point path writes contact count at output `+0x00`, normal at `+0x10`, and contact points from output `+0x40`. | Confirms the output buffer layout used by ROCK. |
| `0x141a1abe0` | Single/alternate manifold path writes the same output shape, including four point slots at output `+0x40`. | Second witness for the output buffer layout. |
| `0x14103d8e0` | Bethesda's contact consumer receives averaged hit point and the helper normal, flipping normal sign by body side for its own event dispatch. | Confirms the helper normal field is treated as contact normal by game code. |

Offset caveats:

- SCISSORS uses the existing ROCK runtime body high-water validation offset currently used by ROCK's hknp body access code.
- SCISSORS has a wrapper for the request-ragdoll-hit-reaction function address, but does not call it during the default milestone path.
- No gameplay damage, combat hit reaction, powered motor control, or pose blending is invoked by SCISSORS in this implementation.
- ROCK publishes the native helper normal as received. SCISSORS currently computes impulse direction from source velocity, not from this normal, so normal orientation can be refined later without changing the point-force gate.

### Verification Run

Fresh local verification performed after the provider header changes:

- `cd SCISSORS; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target SCISSORS`
  - Exit code: 0
  - Output DLL: `SCISSORS/build/Release/SCISSORS.dll`
  - Package: `SCISSORS/build/package/SCISSORS - v0.1.0 - 20260502.7z`
- `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build-tests --config Release --target ROCKProviderBoundaryTests`
  - Exit code: 0
- `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target ROCK`
  - Exit code: 0
  - Output DLL: `ROCK/build/Release/ROCK.dll`
  - CMake copied ROCK to `D:/FO4/mods/ROCK/F4SE/Plugins/`
  - Package: `ROCK/build/package/ROCK - v0.1.0 - 20260502.7z`
- `cd ROCK; .\build-tests\Release\ROCKProviderBoundaryTests.exe`
  - Exit code: 0
- `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build-tests --config Release --target ROCKHavokRuntimePolicyTests ROCKProviderBoundaryTests`
  - Exit code: 0
- `cd ROCK; .\build-tests\Release\ROCKHavokRuntimePolicyTests.exe`
  - Exit code: 0
- `cd ROCK; .\build-tests\Release\ROCKProviderBoundaryTests.exe`
  - Exit code: 0
- `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target ROCK`
  - Exit code: 0
  - CMake copied ROCK to `D:/FO4/mods/ROCK/F4SE/Plugins/`
- `cd SCISSORS; .\build\Release\SCISSORSProviderDependencyTests.exe`
  - Exit code: 0
- `cd SCISSORS; .\build\Release\SCISSORSActorSelectionPolicyTests.exe`
  - Exit code: 0
- `cd SCISSORS; .\build\Release\SCISSORSPointImpulsePolicyTests.exe`
  - Exit code: 0
- `cd SCISSORS; .\build\Release\SCISSORSRagdollBodyPolicyTests.exe`
  - Exit code: 0
- `cd SCISSORS; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target SCISSORS`
  - Exit code: 0

Known build note:

- The SCISSORS Release link emits the existing F4VR-CommonFramework LTCG restart note from `CommonUtils.obj`. The build completes with exit code 0.

## User-Locked Decisions

- SCISSORS will be a sibling folder under `E:\fo4dev\PROJECT_ROCK_V2\SCISSORS`.
- ROCK is a hard runtime dependency. If ROCK is absent, SCISSORS does not work.
- ROCK provider API may be extended through a versioned, backward-compatible provider boundary.
- First implementation milestone should include live ragdolls, not only passive telemetry.
- Nearby NPC activation is enabled for the first milestone.
- Actor eligibility is "all NPCs" by default: no hostility, companion, essential, faction, or protected-state filter unless a later config option disables them.
- Physical pushes are part of the first milestone.
- Push sources are all ROCK sources: hands/fingers, weapon colliders, and held grabbed objects.
- Push model is point impulses, not only linear velocity nudges.
- No gameplay damage/hit reaction dispatch is included in the first milestone unless explicitly approved later.

## Repository And Runtime Context

- ROCK repo: `E:\fo4dev\PROJECT_ROCK_V2\ROCK`
- ROCK build command from AGENTS:
  - `cd ROCK && VCPKG_ROOT="C:/vcpkg" cmake --build build --config Release`
- ROCK output:
  - `ROCK/build/Release/ROCK.dll`
- Real ROCK config location:
  - `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config`
- Real ROCK deploy location:
  - `D:\FO4\mods\ROCK\F4SE\Plugins`
- Proposed SCISSORS config location:
  - `C:\Users\SENECA\Documents\My Games\Fallout4VR\SCISSORS_Config`
- Proposed SCISSORS deploy location:
  - `D:\FO4\mods\SCISSORS\F4SE\Plugins`

The root workspace is not a git repo. `ROCK` is the active git repo and currently has unrelated dirty worktree state. Future implementation must not revert unrelated changes.

## Reference Sources Already Mapped

### ActiveRagdoll / PLANCK Source

Primary source tree:

- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\main.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\include\main.h`
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\blender.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\RE\havok.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\RE\offsets.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\pluginapi.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\higgsinterface001.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\planckinterface001.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\config.cpp`

Important PLANCK structures and systems:

- `ActiveRagdoll` tracks animation pose, ragdoll pose, low-resolution world-space pose, stress, root offsets, state timestamps, original pivots, and blend state.
- `Blender` blends animation-to-ragdoll and ragdoll-to-animation pose tracks.
- `g_activeRagdolls` is keyed by `hkbRagdollDriver*`.
- `g_activeActors` tracks actors currently owned by PLANCK's active-ragdoll system.
- `PhysicsListener` handles hkp contact callbacks, collision cooldowns, hand/weapon/held-body contact, actor hits, and collision suppression.
- `ProcessHavokHitJobsHook` is the primary per-frame runtime loop in PLANCK.
- `PreDriveToPoseHook`, `PostDriveToPoseHook`, `PrePostPhysicsHook`, and `PostPostPhysicsHook` are the active-ragdoll control boundaries.

PLANCK first-mile behavior to port by intent:

- Nearby actor activation using start/end distance gates.
- Active actor set tracking.
- Ragdoll body registration and cleanup.
- Hand/weapon/held-object collision evidence routed into actor-ragdoll force decisions.
- Bounded impulse application with cooldowns.

PLANCK behavior intentionally not in SCISSORS milestone 1:

- Gameplay damage and combat hit reactions.
- Grip holds, yanks, foot yanks, two-hand yanks, and keep-offset behavior.
- Powered motor tuning and pose blending beyond what FO4VR already does.
- Aggression dialogue and stamina logic.
- Collision-filter rewrites for full biped-biped/player-biped policy.

### ROCK Provider Boundary

Relevant existing ROCK files:

- `ROCK/src/api/ROCKApi.h`
- `ROCK/src/api/ROCKProviderApi.h`
- `ROCK/src/api/ROCKProviderApi.cpp`
- `ROCK/src/physics-interaction/ExternalBodyRegistry.h`
- `ROCK/src/physics-interaction/PhysicsInteraction.h`
- `ROCK/src/physics-interaction/PhysicsInteraction.cpp`
- `ROCK/src/physics-interaction/Hand.h`
- `ROCK/src/physics-interaction/PushAssist.h`
- `ROCK/src/physics-interaction/PushAssist.cpp`
- `ROCK/src/physics-interaction/CollisionLayerPolicy.h`
- `ROCK/src/physics-interaction/HavokOffsets.h`

Current ROCK provider facts:

- `ROCK_API_VERSION = 4`.
- `ROCK_PROVIDER_API_VERSION = 1`.
- Provider lookup uses `GetModuleHandleA("ROCK.dll")` and `ROCKAPI_GetProviderApi`.
- `RockProviderFrameSnapshot` is size 256 and includes frame index, `bhkWorld`, `hknpWorld`, readiness flags, weapon bodies, hand transforms, hand body IDs, hand state flags, offhand reservation, and external body count.
- Current external body registration supports owner token, body ID, generation, role, contact policy, and owner hand.
- Current external registry capacity is 64 bodies and 64 contacts. This is enough for PAPER-style reload bodies but not enough for multiple active actor ragdolls.
- Current external contact records only publish hand-vs-external contacts, with source body ID and external body ID. They do not publish weapon/held-object sources or contact point/normal data.
- ROCK already subscribes to FO4VR contact events and classifies hand, weapon, held object, and external bodies.
- ROCK dynamic push assist currently applies linear impulses to dynamic prop/actor layers. SCISSORS must suppress that path for SCISSORS-owned active actor ragdoll bodies to avoid double force application.

Important ROCK layer facts:

- ROCK hand layer is 43.
- ROCK weapon layer is 44.
- Native biped layer is 8.
- Native dead biped layer is 32.
- Native biped-no-character-controller layer is 33.
- ROCK hand collision masks already include actor/biped layers for passive contact.

## Ghidra-Checked FO4VR Anchors

These were checked in the first Ghidra wave against the loaded FO4VR binary. The findings here are planning anchors, not permission to implement unverified offsets beyond the described use.

| Address | Existing/working label | Planning relevance | Verification note |
|---|---|---|---|
| `0x141a3a980` | `hkbRagdollSceneModifier::modify` | Top-level ragdoll processing pipeline | Decompile shows pass 1 calls `0x14177e7c0`, then physics step, then pass 2 calls `0x14177fce0`. |
| `0x14177e7c0` | `hkbRagdollDriver::driveToPose` | Active-ragdoll drive boundary | Decompile confirms enabled guard at `driver+0xB7`, ragdoll interface at `+0x70`, powered/keyframed active flags at `+0xB8/+0xB9`, and calls to drive controller. |
| `0x141781730` | `hkbRagdollDriver::addRagdollToWorld` | Live-ragdoll world lifecycle | Decompile shows pending flag behavior at `driver+0xB4` and call through ragdoll interface vtable when not already in world. |
| `0x14177fce0` | post-driver / map-back boundary | State sync after physics | Decompile confirms split behavior depending on powered/keyframed activity and calls to pose capture/map-back helpers. |
| `0x14192a6a0` | `hknpRagdollDriveController::driveBones` | Later motor-control work | Function exists and is called by `driveToPose`; first milestone should not modify its control data. |
| `0x14192b490` | apply keyframed transform helper | Later keyframed-drive work | Function exists and is called by `driveToPose`; first milestone should not depend on direct keyframed transform writes. |
| `0x14195e1a0` | `hknpRagdoll` constructor | Body-map layout anchor | Decompile confirms setup ref at `+0x40`, body-map data at `+0x48`, count at `+0x50`, capacity/ownership at `+0x54`. |
| `0x14153a4d0` | `hknpWorld::applyBodyImpulseAt` | Point impulse primitive | Decompile confirms params `(world, bodyId, impulse, worldPoint)`, checks body flag `0x02`, wakes body if needed, then updates linear and angular velocity. |
| `0x14061b5c0` | `FOCollisionListener::OnContactImpulseEvent` | Aggregate contact evidence layout | Decompile confirms aggregate event reads at `param_3+0x08/+0x0C/+0x10/+0x18/+0x19/+0x30`; this is not the raw contact event layout. |

Reverse-engineering caution:

- The aggregate contact impulse callback provides body IDs, manifold pointer, sub-index, manifold type flag, and per-contact impulse magnitudes.
- The raw `hknpContactEvent` documentation says explicit world-space contact point is at `+0x00`, normal at `+0x10`, body IDs at `+0x20/+0x28`.
- ROCK currently reads body IDs from `+0x08/+0x0C`, so it is using the aggregate contact impulse signal shape, not the raw contact event shape.
- Because the user chose point impulses, implementation must verify how ROCK will obtain a stable world-space application point before enabling SCISSORS point forces.

## Required ROCK Provider Boundary Shape

ROCK provider v2 preserves the v1 ABI and adds external body/contact functions. Provider v3 preserves that shape and adds frame snapshot scale fields required by SCISSORS.

Required provider concepts:

- `RockProviderExternalBodyRole::ActorRagdollBone`
- `RockProviderExternalSourceKind`
  - `Unknown`
  - `Hand`
  - `Weapon`
  - `HeldObject`
- `RockProviderExternalContactQuality`
  - `BodyPairOnly`
  - `AggregateImpulse`
  - `RawPoint`
- External contact policy flags:
  - `ReportContacts`
  - `ReportAllSourceKinds`
  - `SuppressRockDynamicPush`

Required contact payload:

- Source body ID.
- Target external body ID.
- Source kind.
- Source hand when applicable.
- Hand collider role/finger/segment metadata when applicable.
- Weapon body descriptor when applicable.
- Held-object owner hand when applicable.
- Owner token and generation for the target external body.
- Frame index and monotonically increasing sequence.
- Source linear velocity in Havok units.
- Contact normal when available.
- Contact point in world Havok coordinates when verified.
- Contact point quality enum.
- Aggregate impulse magnitude when available.

Capacity requirement:

- Provider v2 external body capacity should support at least 2048 registered bodies.
- Provider v2 external contact ring should support at least 512 recent contacts.
- Registration and contact recording must not allocate inside the physics callback.
- Overflow should keep deterministic behavior and log sampled diagnostics.

Double-force prevention:

- When an external body is registered with `SuppressRockDynamicPush`, ROCK must not apply its own dynamic push assist to that body.
- SCISSORS must be the only module applying actor-ragdoll point impulses to registered SCISSORS bodies.

## SCISSORS Subsystems To Implement

### `RockProviderClient`

Responsibilities:

- Resolve `ROCK.dll`.
- Resolve `ROCKAPI_GetProviderApi`.
- Require provider v3 because SCISSORS needs provider snapshot scale fields.
- Register frame callback.
- Cache `RockProviderFrameSnapshot`.
- Poll external contact snapshots.
- Clear SCISSORS registrations on shutdown/world reset.

Failure behavior:

- Missing ROCK: log and stay inactive.
- Provider version below v3: log and stay inactive.
- ROCK not physics-ready: wait without touching actor physics.
- ROCK world pointer changes: clear actor/ragdoll/contact state and rebuild after readiness returns.

### `ActorScanner`

Responsibilities:

- Read `RE::ProcessLists::GetSingleton()->highActorHandles`.
- Resolve actor handles safely.
- Exclude player, deleted, disabled, unloaded, or invalid actors.
- Include all non-player actors with a usable animation graph and ragdoll driver.
- Apply PLANCK-style distance gates:
  - Activate below `activeRagdollStartDistance = 50.0` Havok units.
  - Remove above `activeRagdollEndDistance = 60.0` Havok units.
- Respect `minFramesBetweenActorAdds = 2`.

Default eligibility:

- All NPCs are eligible if they expose a usable ragdoll driver.
- No hostile-only filter.
- No companion/essential/protected filter.
- Non-humanoid or nonstandard races are attempted only if the ragdoll driver and body map validate; otherwise they fail closed with a diagnostic.

### `RagdollLifecycle`

Responsibilities:

- Maintain actor state:
  - `Candidate`
  - `LiveRequested`
  - `LiveConfirmed`
  - `Registered`
  - `Removing`
  - `Suspended`
- Use FO4VR-supported ragdoll lifecycle paths and existing engine state.
- Do not manually insert hknp ragdoll bodies into the world.
- Confirm liveness by checking body IDs and body-map validity after engine activation.
- Remove actors from SCISSORS state when outside end distance, invalidated, disabled, unloaded, or on world reset.

First milestone scope:

- Live ragdoll activation and registration are in scope.
- Powered motor rewriting is out of scope.
- Animation pose blending is out of scope except engine-native behavior.

### `RagdollBodyMap`

Responsibilities:

- Read the FO4VR `hknpRagdoll` body map.
- Validate every body ID before registration:
  - Body ID is not `0x7FFFFFFF`.
  - Body slot is readable under world high-water/capacity checks.
  - Body array entry still matches the body ID.
  - Body has a usable motion index when force application is needed.
  - Body belongs to the actor's ragdoll ownership context.
- Register mapped ragdoll bodies with ROCK provider v2 under a stable owner token.
- Store actor handle/form ID, ragdoll generation, bone index, body ID, and last validated frame.

Open verification gate:

- Need a final Ghidra/code verification pass for the exact FO4VR route from actor animation graph to `hkbRagdollDriver*` and from driver to `hknpRagdoll*` in CommonLibF4VR-accessible code before writing the final wrappers.

### `PointImpulsePush`

Responsibilities:

- Consume provider v2 contacts where the target body is a registered SCISSORS actor ragdoll body.
- Accept source kinds:
  - Hand/finger collider.
  - ROCK weapon body.
  - ROCK held-object body.
- Compute point impulse:
  - Source velocity from ROCK source body motion.
  - Direction from contact normal, oriented by relative source-to-target motion.
  - Contact point from provider contact point if `RawPoint`.
  - If only aggregate evidence exists, implementation must pause at the verification gate rather than silently inventing a point.
- Apply with `hknpWorld::applyBodyImpulseAt`.
- Apply cooldown per `(sourceBodyId, targetBodyId)`.
- Clamp magnitude, per-frame count, and per-body accumulated impulse.
- Log sampled diagnostics for accepted, skipped, clamped, and applied impulses.

Default first-mile config:

- `bEnabled = true`
- `fActiveRagdollStartDistance = 50.0`
- `fActiveRagdollEndDistance = 60.0`
- `iMinFramesBetweenActorAdds = 2`
- `iMaxActiveActors = 8`
- `iMaxRegisteredBodies = 2048`
- `iMaxContactsPerFrame = 128`
- `fMinSourceSpeedHavok = 0.35`
- `fMaxImpulseHavok = 2.0`
- `fSourceTargetCooldownSeconds = 0.08`
- `bEnableHandSources = true`
- `bEnableWeaponSources = true`
- `bEnableHeldObjectSources = true`
- `bEnableDamageEvents = false`
- `bEnableMotorControl = false`

The `iMaxActiveActors = 8` default is a safety cap, not an eligibility filter. It bounds provider registration and force work while still following the user's all-NPC decision. Actor selection should be nearest valid actors first.

## Implementation Phases

### Phase 0: Documentation And Verification Gate

- Keep this document as the context anchor.
- Add a focused Ghidra audit note for any new binary claims before implementation.
- Verify raw contact point acquisition for ROCK provider v2.
- Verify actor graph to ragdoll driver access path in FO4VR/CommonLibF4VR.
- Verify any new offsets before use.

### Phase 1: Provider v2 In ROCK

- Add v2 structs/enums/functions without breaking v1.
- Expand external registry capacity.
- Record all-source external contacts.
- Add source velocity and contact quality.
- Add suppress-ROCK-dynamic-push policy.
- Add tests for struct sizes and contact routing.

### Phase 2: SCISSORS Plugin Skeleton

- Create `SCISSORS` sibling CMake plugin.
- Use the same CommonLibF4VR/F4VR-CommonFramework/vcpkg pattern as ROCK.
- Add logging and config.
- Add hard ROCK provider v3 init once scale fields become part of the dependency contract.
- Add no-op frame callback that logs readiness transitions.
- Build locally in Release.

### Phase 3: Actor Scan And Live Ragdoll Registration

- Implement high-process actor scanning.
- Implement actor state machine.
- Activate/confirm live ragdolls through FO4VR-supported lifecycle.
- Build `RagdollBodyMap`.
- Register bodies with ROCK provider v2.
- Add debug logging and optional overlay markers for registered bodies.

### Phase 4: Point Impulse Pushes

- Consume ROCK provider v2 contacts.
- Validate target body and source body.
- Apply point impulses with clamps and cooldowns.
- Suppress ROCK dynamic push on SCISSORS bodies.
- Confirm all source kinds work separately:
  - Hand/finger.
  - Weapon body.
  - Held-object body.

### Phase 5: Later Active-Ragdoll Behavior

Not part of first implementation approval:

- Motor tuning.
- Pose blending.
- Get-up blending.
- Yanks and grabs.
- Physical damage.
- Aggression/follower logic.
- Collision filter rewrites across biped groups.
- Full PLANCK-style stress propagation.

## Test And Acceptance Plan

Build commands:

- ROCK:
  - `cd ROCK`
  - `$env:VCPKG_ROOT="C:/vcpkg"; cmake --build build --config Release`
- SCISSORS after scaffold:
  - `cd SCISSORS`
  - `$env:VCPKG_ROOT="C:/vcpkg"; cmake --build build --config Release`

Provider tests:

- Provider v1 still initializes.
- Provider v3 initializes with expected struct sizes and scale fields.
- External body registration accepts actor-ragdoll role.
- Registry rejects duplicate body IDs.
- Registry rejects wrong owner tokens.
- Contact ring preserves sequence ordering under overflow.
- Suppress policy prevents ROCK dynamic push on registered bodies.

SCISSORS tests:

- Missing ROCK disables SCISSORS.
- ROCK provider v1-only disables SCISSORS.
- ROCK provider v3 initializes SCISSORS.
- Actor scanner ignores player/deleted/disabled/unloaded actors.
- Actor scanner includes all valid non-player high-process actors with usable ragdoll drivers.
- Distance gates activate below 50 and remove above 60.
- Actor cap chooses nearest valid actors first.
- Ragdoll body validation rejects invalid/stale/static/no-motion body IDs.
- Contact source classification accepts hand, weapon, and held-object sources.
- Point impulse math clamps magnitude and applies cooldowns.

In-game acceptance:

- With ROCK absent, SCISSORS logs the missing dependency and makes no physics changes.
- With ROCK present, SCISSORS logs provider v3 readiness and frame callbacks.
- Nearby NPCs enter SCISSORS live/registered state.
- ROCK hand/finger collision against a registered ragdoll body produces a SCISSORS point impulse.
- ROCK weapon collision against a registered ragdoll body produces a SCISSORS point impulse.
- A ROCK-held object colliding with a registered ragdoll body produces a SCISSORS point impulse.
- No duplicate push is applied by ROCK dynamic push assist to SCISSORS bodies.
- No damage events or combat hit reactions fire from SCISSORS in milestone 1.
- World/cell/menu transitions clear and rebuild state without stale body access.

## 2026-05-02 Code Review Fixes

- ROCK deployment copy no longer removes `ROCK.esp`, script folders, or mesh folders from the deployed mod path.
- ROCK provider frame callbacks are invoked through a fault-isolating boundary. A callback that raises an SEH fault or C++ exception is logged and unregistered instead of taking down ROCK's frame loop.
- ROCK provider contact snapshots now return the newest requested contact window in chronological order. SCISSORS still polls the full v2 buffer and applies its own per-frame budget.
- ROCK raw contact point extraction is lazy per contact event and is only evaluated when an external registered body contact is being published.
- Provider contact field semantics were clarified: the value formerly named as an aggregate impulse is a Bethesda contact point weight sum from contact-signal `+0x30`.
- SCISSORS ragdoll body registration now clamps to the verified FO4VR body-map limit of 255 bodies per actor, matching the raw body-map reader and registration buffer.
- SCISSORS raw ragdoll pointer reads now fail closed through pointer plausibility checks plus SEH-protected reads for graph/ragdoll/body-map/world body-array access.
- PLANCK load-reduction and collision-control pattern audit recorded in `docs/2026-05-02-planck-load-reduction-audit.md`.

## 2026-05-03 PLANCK Portable Load-Reduction Implementation

- Added SCISSORS `WorldRecoveryGate`, mirroring PLANCK's 0.4s world-change wait for the ROCK provider model. Provider/world outages now mark actor lifecycle unsafe; SCISSORS clears registrations and waits for `fWorldRecoveryWaitSeconds` before actor registration/contact processing resumes.
- Added `fWorldRecoveryWaitSeconds = 0.4` to `SCISSORS.ini` under `[ActorSelection]`.
- Added retained-contact discard during the world-recovery wait so ROCK contact snapshots from before a reset are advanced past instead of replayed after recovery.
- Added pruning for SCISSORS source-target point-impulse cooldown pairs, matching PLANCK's bounded cooldown-map pattern for the SCISSORS-owned cooldown layer.
- Deferred biped matrix pruning, hknp comparison callbacks, useful-only self-collision, listener ordering, and physics damage bypass because those require FO4VR/hknp binary proof instead of source-only porting.

## 2026-05-03 First In-Game No-Effect Fix Pass

This pass fixes the first test result where SCISSORS loaded but NPCs did not visibly move. The implementation follows the vanilla FO4VR ragdoll-hit path instead of applying raw hknp impulses to animation-owned bodies: direct `hknpWorld::ApplyBodyImpulseAt` only mutates bodies with dynamic motion, so SCISSORS now requests ragdoll hit reaction, resolves the actor root object, recursively switches the root physics to dynamic, then registers the verified body IDs with ROCK.

- ROCK provider v3 is now required by SCISSORS because v3 snapshots carry `gameToHavokScale`, `havokToGameScale`, and `physicsScaleRevision`.
- ROCK now fills the provider snapshot scale fields from `physics_scale::gameToHavok()`, `physics_scale::havokToGame()`, and `physics_scale::revision()`.
- SCISSORS actor selection now converts FO4VR actor/player position distance from game units to Havok distance before comparing to the PLANCK-style 50/60 gates. The fallback scale is `1 / 70` and is configurable as `fFallbackGameToHavokScale`.
- SCISSORS active config was updated in `C:\Users\SENECA\Documents\My Games\Fallout4VR\SCISSORS_Config\SCISSORS.ini` to require provider v3 and include the scale/recovery keys.
- Ghidra re-checks used for this pass:
  - `0x140dc0620` resolves the actor root object used by vanilla ragdoll paths.
  - `0x141df95b0` is Bethesda's recursive SetMotion wrapper; parameter order is root, preset, recursive, force, activate.
  - `0x140e62c80` is the ragdoll hit-reaction request path; vanilla impact calls use all three boolean flags enabled.
  - `0x141df5900` wraps `hknpWorld::ApplyBodyImpulseAt` and then runs Bethesda post-impulse bookkeeping.
- SCISSORS point impulses now fail closed unless the target body is readable, the body slot matches the requested body ID, and the body is dynamic. Accepted point impulses use Bethesda's wrapper at `0x141df5900` instead of the raw CommonLib method.
- Build and deploy results:
  - SCISSORS Release built and was copied to `D:\FO4\mods\SCISSORS\F4SE\Plugins`.
  - ROCK Release built with MSBuild node reuse disabled and was copied to `D:\FO4\mods\ROCK\F4SE\Plugins`.
  - Focused SCISSORS tests all pass: provider dependency, actor selection policy, point impulse policy, ragdoll body policy.

## 2026-05-03 No-Activation Log Pass

Latest in-game logs after the first fix pass show SCISSORS loading, reading the live config, and binding ROCK successfully, but there are no `Activated actor` lines and no activation failure warnings. That proves the chain is stopping before actor activation/registration rather than at point impulse application.

Added bounded info-level diagnostics to SCISSORS:

- `SCISSORS: frame ready ...` confirms ROCK frame callbacks are reaching SCISSORS and reports hknp world, FRIK/menu/config flags, provider scale, scale revision, and external body count.
- `SCISSORS: frame blocked ...` reports provider/world readiness failures if callbacks arrive before ROCK physics is usable.
- `SCISSORS: frame waiting for world recovery ...` reports the world-recovery delay gate.
- `SCISSORS: actor scan ...` reports high-process handle count, null handles, candidate count, start-gate count, active count, rejection buckets for player/deleted/disabled/unloaded/missing-driver/missing-bodies/gate, activation attempts/failures, throttle count, scale, and nearest eligible actor distance.

This instrumentation is deployed in `D:\FO4\mods\SCISSORS\F4SE\Plugins\SCISSORS.dll` built at 2026-05-03 00:53:44. The next in-game pass should make the stop point explicit without needing a debugger attached.

## Known Risks

- ROCK provider v1 registry capacity is too small for actor ragdolls.
- Point impulses require a verified contact point; ROCK now emits `RawPoint` contacts through Bethesda's contact-signal extraction helper, but in-game validation still needs to confirm every source path reports `RawPoint`.
- FO4VR ragdoll driver and hknp body-map access must be implemented with verified wrapper paths, not guessed from Skyrim hkp code.
- Applying point impulses to active ragdoll bodies may interact with engine-driven pose recovery. First milestone must use clamps, cooldowns, and logging.
- All-NPC activation is intentionally broad and must be bounded by `iMaxActiveActors`.
- Weapon and held-object sources may produce stronger impulses than hand/finger sources; source-specific multipliers may be needed after in-game measurement.

## Open Verification Gates Before In-Game Tuning

Remaining blockers before treating SCISSORS point impulses as accepted gameplay behavior:

1. Run the in-game acceptance pass with actor lifecycle enabled and point impulses disabled first.
2. Then enable point impulses and test each ROCK source separately:
   - Hand/finger colliders.
   - Weapon colliders.
   - Held-object bodies.
3. Confirm logs show provider contacts arriving as `RawPoint`, not `BodyPairOnly`.
4. Measure whether source-specific multipliers are needed for weapon and held-object hits.

Verified in the implementation pass:

- Raw contact point acquisition from ROCK's FO4VR contact path via Bethesda helper `0x14175c650`.
- Multi-point and single/alternate contact-manifold output layout through `0x1417339f0` and `0x141a1abe0`.
- Actor graph to `hkbRagdollDriver*` access in FO4VR using CommonLibF4VR/local wrappers.
- Driver to `hknpRagdoll*` pointer path and body-map data/count offsets.
- Point impulse wrapper parameter order for `hknpWorld::ApplyBodyImpulseAt`.
- Dynamic body registration policy behavior through focused provider and SCISSORS tests.

Ghidra protocol remains active: before every Ghidra operation, explain why it is needed and wait for approval.

## Implementation Guardrails

- Always check PLANCK source before implementing actor-ragdoll behavior.
- Always use ROCK provider API for player-originated collider/grab evidence.
- Do not include ROCK private headers in SCISSORS unless the user explicitly changes the boundary decision.
- Do not duplicate ROCK's contact listener unless provider v2 cannot expose required contact point data after verification.
- Do not add gameplay damage in milestone 1.
- Do not add powered motor control in milestone 1.
- Do not assume CommonLibF4VR layouts are correct without binary-backed verification for new ragdoll offsets.
- Record new findings in Markdown as they are discovered.

## Handoff Checklist

- [x] User approves implementation start.
- [x] Verify contact point acquisition gate.
- [x] Verify actor/ragdoll pointer path gate.
- [x] Implement ROCK provider v2.
- [x] Build ROCK Release.
- [x] Scaffold SCISSORS sibling plugin.
- [x] Implement SCISSORS provider client.
- [x] Implement actor scanner and live registration.
- [x] Implement point impulse path behind `RawPoint` and config gates.
- [x] Build SCISSORS Release.
- [x] Run focused tests.
- [ ] Run in-game acceptance pass.
- [x] Update this document with results, discoveries, and any changed assumptions.
