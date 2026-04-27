# Full PLANCK Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the FO4VR-native active-ragdoll collision architecture needed for NPCs to be physically pushed by ROCK hand bodies and weapon bodies.

**Architecture:** This is a full PLANCK-style system, not a collision-layer toggle. Existing ROCK hand and weapon colliders remain the player-side interaction bodies; the new work adds verified actor ragdoll lifecycle, group-aware collision policy, contact classification, pre-physics impulse work, and driver hooks around FO4VR's hkb/hknp ragdoll frame order.

**Tech Stack:** C++23, F4SE VR, CommonLibF4VR/F4VR-CommonFramework, FO4VR hknp/hkb Havok 2014, ROCK physics-interaction module, Ghidra MCP verification.

---

## Non-Negotiable Implementation Rules

- Do not trust existing `.md` files as proof for offsets, layouts, hook targets, or behavior.
- Use PLANCK source at `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll` as behavioral reference.
- Use HIGGS source at `E:\fo4dev\skirymvr_mods\source_codes\higgs` as hand, weapon, held-object, and collision-filter integration reference.
- Use FO4VR Ghidra verification as the authority for every native function, struct field, vtable slot, and hook site.
- Before each Ghidra operation, explain why it is needed and wait for user approval.
- Do not globally enable charcontroller layer `30` as the solution. NPC interaction must be actor/group-aware.
- Do not remove or weaken existing working hand/weapon collision behavior.
- Every implementation cluster must begin with a short decision note in code or in the verification document explaining:
  - why this approach is used,
  - which alternatives were rejected,
  - which Ghidra facts support the implementation.

---

## Current ROCK State

Existing files that must be treated as integration points:

- `src/physics-interaction/PhysicsInteraction.h`
  - Owns init/update/shutdown, hand bodies, weapon collision, contact callback dispatch, contact body tracking, and layer registration state.
- `src/physics-interaction/PhysicsInteraction.cpp`
  - Registers ROCK hand layer `43` and weapon layer `44`.
  - Subscribes to FO4VR contact events.
  - Reads contact body ids from event `+0x08/+0x0C`.
  - Tracks hand and weapon-part contact.
- `src/physics-interaction/CollisionLayerPolicy.h`
  - Owns helper policy for layer pair mutation and weapon projectile blocking.
- `src/physics-interaction/BethesdaPhysicsBody.h/.cpp`
  - Owns ROCK-created hknp bodies and already wraps filter info and impulse operations.
- `src/physics-interaction/HavokOffsets.h`
  - Central location for native offsets currently used by ROCK physics.
- `src/physics-interaction/PhysicsHooks.h/.cpp`
  - Installs current bump and charcontroller manifold hooks.
- `src/physics-interaction/ObjectDetection.cpp`
  - Resolves hknp bodies back to `TESObjectREFR`, currently skipping layer `30`.
- `src/RockConfig.h/.cpp`
  - Owns runtime config and INI loading.
- `data/config/ROCK.ini`
  - User-facing configuration shipped with the plugin.
- `tests/TransformConventionTests.cpp`
  - Existing focused test executable. It already tests `CollisionLayerPolicy`.

Existing behavior to preserve:

- Hand collision body layer: `43`.
- Weapon collision body layer: `44`.
- Weapon projectile/spell collision policy remains controlled by existing config.
- Existing grab, hand transform, weapon reload, held-object contact, and two-handed grip behavior must remain intact.
- Existing contact callback must continue to feed hand touch, held-object contact, and left-hand weapon-part detection.

---

## Ghidra-Verified Ground Truth

These facts were verified directly from the FO4VR binary during planning and are valid starting points for implementation. Any new use still needs a local implementation-time audit before code is written.

### Ragdoll And Frame Order

- `0x141a3a980` is `hkbRagdollSceneModifier::modify`-like frame orchestration.
  - It calls the ragdoll pre-drive path for character graphs.
  - It steps the world/scene.
  - It calls the ragdoll post path for character graphs.
- `0x14177e7c0` is the FO4VR hkb ragdoll pre-drive/drive path candidate.
  - It gates on driver state and generator output.
  - It reads driver fields around `+0x70`, `+0x80`, `+0x88`, `+0xB7`, `+0xB8`, and `+0xB9`.
  - It calls hknp ragdoll drive helpers.
- `0x14177fce0` is the FO4VR hkb ragdoll post-drive/post-physics wrapper candidate.
  - It captures/reconstructs ragdoll pose and final output pose depending on powered/rigid state.
- `0x141781730` is an add-ragdoll-to-world deferred path.
  - It checks dirty/in-world state and calls through the ragdoll interface.

### Collision Filter And Body Filter Info

- `bhkCollisionFilter` constructor uses vtable `0x142e84308`.
- Filter matrix offset is `filter + 0x1A0`.
- `0x141e115b0` is the collision filter comparison path.
  - Low 7 bits encode collision layer.
  - Bit 14 can suppress collision.
  - High 16 bits encode group/self comparison.
  - Layer matrix indexing uses `filter + 0x1A0 + layer * 8`.
  - Character controller layer `0x1E` has special behavior.
  - BIPED/DEADBIP/BIPED_NO_CC group handling exists for layers `8`, `32`, and `33`.
- hknp body filter info is at body `+0x44`.
- ROCK's existing `kFunc_SetBodyCollisionFilterInfo = 0x1DF5B80` is a valid wrapper target.
  - It writes body filter info and notifies the world dirty path.

### Contact Callback And Event Layout

- `0x1403b9e50` subscribes a callback to the contact signal shape ROCK already uses.
- `0x14061b5c0` verifies the FO4VR `FOCollisionListener` callback layout:
  - event `+0x08`: body id A
  - event `+0x0C`: body id B
  - event `+0x10`: manifold pointer
  - event `+0x18`: contact index
  - event `+0x19`: manifold type
  - event `+0x30`: separating velocities
- Body array lookup uses hknp world data with body stride `0x90`.
- Body filter info is read from body `+0x44`.
- Motion id is read from body `+0x68`.
- Body collision-object/backpointer paths are used by vanilla collision dispatch.

### Impulses

- `0x141e08520` applies a linear impulse through `bhkNPCollisionObject`.
- `0x141e08640` applies a point impulse through `bhkNPCollisionObject`.
- Deeper hknp body impulse paths call world/body functions and activate the body after the impulse.

### Existing Hook Candidates

- `0x141e24980` is the character controller bump handling path currently hooked by ROCK.
- `0x141e4b7e0` is a `bhkCharProxyController` hknp character-proxy listener path currently hooked by ROCK for held-object surface velocity handling.
- These hooks are useful context but must not be treated as the whole PLANCK collision solution.

---

## Reference Behavior To Map From PLANCK

The implementation agent must re-open and map these source areas before changing code:

- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\main.cpp`
  - Collision filter comparison callback.
  - Pre-physics callback.
  - actor update loop.
  - add/remove ragdoll to world.
  - pre-drive, drive, post-physics hooks.
  - hit, shove, yank, held-object, and active-ragdoll state transitions.
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\include\main.h`
  - `ActiveRagdoll` state fields.
  - active group sets.
  - state enums.
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\blender.cpp`
  - pose blending behavior.
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\include\config.h`
  - activation distances, shove thresholds, collision toggles, and impulse multipliers.
- `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\RE\havok.cpp`
  - Havok wrapper style and world-lock expectations.

Behavioral requirements to preserve from PLANCK:

- Active actors get ragdoll bodies in the physics world.
- Actors without active ragdoll support can still be marked as hittable through charcontroller fallback grouping.
- Player hands/weapons interact with NPC biped bodies through group-aware filtering.
- Player charcontroller interactions must be controlled separately from hand/weapon interaction.
- Self-collision, active biped collision, held actor collision, and no-player-charcontroller cases must be separate states.
- Physics-thread contact callbacks must enqueue work; unsafe game state changes belong in verified game-thread or pre-physics windows.

---

## File Structure To Create Or Modify

### Create

- `docs/planck-port-verification.md`
  - Running verification ledger. Every offset, hook, field, and native call used by the port gets a row with source reference and Ghidra evidence.
- `src/physics-interaction/PlanckTypes.h`
  - Shared enums and POD structs for actor state, collision classification, contact classification, and impulse jobs.
- `src/physics-interaction/PlanckCollisionPolicy.h`
  - Pure functions for filter-info encoding, group policy, layer decisions, and actor collision-state transitions.
- `src/physics-interaction/PlanckContactTracker.h/.cpp`
  - Contact classification and throttled contact state. Extends existing contact handling without replacing unrelated behavior.
- `src/physics-interaction/PlanckImpulseQueue.h/.cpp`
  - Bounded queue for physics-safe impulse and shove work flushed in the verified pre-physics window.
- `src/physics-interaction/PlanckRagdollManager.h/.cpp`
  - Actor scanning, active-ragdoll lifecycle, group ownership, fallback charcontroller state, cleanup.
- `src/physics-interaction/RagdollGraphBridge.h/.cpp`
  - Verified wrappers for FO4VR graph/ragdoll operations.
- `src/physics-interaction/RagdollDriverHooks.h/.cpp`
  - Hook installation and hook bodies for pre-drive and post-drive ragdoll paths.
- `src/physics-interaction/PlanckDebug.h/.cpp`
  - Throttled logging helpers and optional debug counters scoped to PLANCK behavior.
- `tests/PlanckPolicyTests.cpp`
  - Focused regression tests for pure policy and classification logic.

### Modify

- `src/physics-interaction/PhysicsInteraction.h/.cpp`
  - Own and update the PLANCK manager.
  - Route contact events into `PlanckContactTracker`.
  - Flush impulse queue at the verified physics point.
  - Preserve existing hand, weapon, grab, and reload logic.
- `src/physics-interaction/CollisionLayerPolicy.h`
  - Keep existing helper functions.
  - Add generic named constants for FO4 biped/charcontroller layers after Ghidra verification.
  - Delegate actor-specific policy to `PlanckCollisionPolicy`.
- `src/physics-interaction/HavokOffsets.h`
  - Add verified offsets only after a Ghidra verification row exists in `docs/planck-port-verification.md`.
- `src/physics-interaction/BethesdaPhysicsBody.h/.cpp`
  - Add narrowly scoped helper wrappers only if needed by impulse queue or filter policy.
- `src/physics-interaction/PhysicsHooks.h/.cpp`
  - Install ragdoll driver hooks through `RagdollDriverHooks`.
  - Keep bump/manifold hooks isolated from PLANCK driver hooks.
- `src/physics-interaction/ObjectDetection.cpp`
  - Add PLANCK-specific body classification path; do not make generic object selection start treating all layer `30` bodies as grabbable.
- `src/RockConfig.h/.cpp`
  - Add PLANCK configuration fields and INI loading.
- `data/config/ROCK.ini`
  - Add documented PLANCK configuration block.
- `CMakeLists.txt`
  - Include `tests/PlanckPolicyTests.cpp` in the test target or create a second focused executable.

---

## Public Interfaces And Types

Use these names unless implementation-time verification discovers a concrete reason to rename them.

### `PlanckTypes.h`

Define shared types as POD or near-POD structures. Keep this file free of hooks and native calls.

```cpp
namespace frik::rock::planck
{
    enum class ActorRagdollState : std::uint8_t
    {
        Inactive,
        Candidate,
        CharControllerFallback,
        AddingRagdoll,
        ActiveBlendIn,
        Active,
        ActiveBlendOut,
        RemovingRagdoll,
        Disabled
    };

    enum class ActorCollisionState : std::uint8_t
    {
        None,
        HittableCharController,
        ActiveBipedNoPlayerCC,
        ActiveBipedPlayerHandsWeapons,
        HeldByPlayer,
        DeadOrKnocked
    };

    enum class ContactSource : std::uint8_t
    {
        Unknown,
        RightHand,
        LeftHand,
        Weapon,
        HeldObject
    };

    enum class ContactTarget : std::uint8_t
    {
        Unknown,
        ActorBipedBody,
        ActorCharController,
        World,
        Clutter,
        PlayerBody
    };

    struct BodyRef
    {
        std::uint32_t bodyId = 0x7FFF'FFFF;
        std::uint32_t filterInfo = 0;
        std::uint32_t layer = 0;
        std::uint32_t group = 0;
    };

    struct ClassifiedContact
    {
        ContactSource source = ContactSource::Unknown;
        ContactTarget target = ContactTarget::Unknown;
        BodyRef sourceBody{};
        BodyRef targetBody{};
        RE::TESObjectREFR* targetRef = nullptr;
        std::uint32_t targetFormId = 0;
        float separatingVelocity = 0.0f;
        RE::NiPoint3 worldPoint{};
        RE::NiPoint3 worldNormal{};
        bool hasWorldPoint = false;
        bool hasWorldNormal = false;
    };

    struct ImpulseJob
    {
        RE::TESObjectREFR* targetRef = nullptr;
        std::uint32_t targetBodyId = 0x7FFF'FFFF;
        RE::NiPoint3 impulse{};
        RE::NiPoint3 worldPoint{};
        bool usePointImpulse = false;
        const char* reason = nullptr;
    };
}
```

### `PlanckCollisionPolicy.h`

Expose only pure policy functions where possible:

```cpp
namespace frik::rock::planck
{
    struct FilterInfoParts
    {
        std::uint32_t layer = 0;
        std::uint32_t group = 0;
        bool collisionSuppressed = false;
    };

    FilterInfoParts decodeFilterInfo(std::uint32_t filterInfo);
    std::uint32_t encodeFilterInfo(FilterInfoParts parts);

    bool shouldEnableLayerPair(std::uint32_t layerA, std::uint32_t layerB, const RockConfig& config);
    bool shouldActorBodyCollideWithPlayerSource(
        ActorCollisionState actorState,
        ContactSource source,
        bool isPlayerOwnedBody,
        bool isHeldActor);
}
```

### `PlanckRagdollManager.h`

Expose manager operations to `PhysicsInteraction`:

```cpp
namespace frik::rock::planck
{
    class PlanckRagdollManager
    {
    public:
        void init();
        void shutdown();
        void update(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, float deltaTime);
        void onContact(const ClassifiedContact& contact);
        void flushPrePhysics(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld);
        void onCellOrWorldChanged();
        bool ownsActorGroup(std::uint32_t group) const;
        ActorCollisionState getActorCollisionState(RE::TESObjectREFR* actorRef) const;
    };
}
```

### `RagdollDriverHooks.h`

Expose hook installation and hook lifecycle:

```cpp
namespace frik::rock::planck
{
    void installRagdollDriverHooks();
    void setRagdollHooksEnabled(bool enabled);
}
```

---

## Implementation Tasks

### Task 1: Create Verification Ledger And Native-Fact Gate

**Files:**

- Create: `docs/planck-port-verification.md`
- Modify: `src/physics-interaction/HavokOffsets.h`

- [ ] Create `docs/planck-port-verification.md` with this structure:

```markdown
# PLANCK Port Verification Ledger

This file records implementation-time verification only. It is not proof by itself; every row must point to a Ghidra observation and the source behavior it supports.

## Rules

- Existing markdown notes are not evidence.
- PLANCK/HIGGS source establishes intended behavior.
- FO4VR Ghidra establishes whether an offset, layout, hook, or native call is valid.
- No offset is added to `HavokOffsets.h` without a row in this file.

## Verified Entries

| ID | Purpose | FO4VR Address / Field | Ghidra Evidence | Reference Source | ROCK Use |
| --- | --- | --- | --- | --- | --- |
| V-001 | Ragdoll scene frame order | `0x141a3a980` | Calls pre-drive, world step, then post path for character graphs | PLANCK ragdoll scene hook flow | Hook-order model |
| V-002 | Ragdoll pre-drive path | `0x14177e7c0` | Gates on driver state and generator output, calls hknp ragdoll drive helpers | PLANCK PreDriveToPoseHook | Candidate pre-drive hook |
| V-003 | Ragdoll post path | `0x14177fce0` | Captures/reconstructs ragdoll pose and final output pose | PLANCK PostPhysicsHook | Candidate post hook |
| V-004 | Add ragdoll deferred path | `0x141781730` | Checks dirty/in-world state, calls ragdoll interface | PLANCK AddRagdollToWorld | RagdollGraphBridge candidate |
| V-005 | Collision matrix | `filter + 0x1A0` | Filter callback indexes matrix by layer | HIGGS/PLANCK filter policy | Layer policy |
| V-006 | hknp body filter info | body `+0x44` | Vanilla callback reads filter info from body | PLANCK group filtering | Body classification |
| V-007 | Contact event body ids | event `+0x08/+0x0C` | FOCollisionListener callback reads both body ids | HIGGS contact listener pattern | Contact tracker |
| V-008 | Linear impulse wrapper | `0x141e08520` | Resolves body and activates after impulse | PLANCK shove/hit impulses | Impulse queue |
| V-009 | Point impulse wrapper | `0x141e08640` | Resolves body and activates after point impulse | PLANCK shove/hit impulses | Impulse queue |
```

- [ ] Add candidate offsets to `HavokOffsets.h` only after expanding the ledger with implementation-time verification rows.
- [ ] Keep all new offsets named by behavior, not by copied Skyrim names. Example: `kFunc_HkbRagdollDriver_PreDriveCandidate`, not `kFunc_SkyrimDriveToPose`.

**Acceptance:**

- The ledger exists before source changes.
- Every new native offset added by later tasks has a matching ledger row.

### Task 2: Add Pure PLANCK Types And Policy Tests

**Files:**

- Create: `src/physics-interaction/PlanckTypes.h`
- Create: `src/physics-interaction/PlanckCollisionPolicy.h`
- Create: `tests/PlanckPolicyTests.cpp`
- Modify: `CMakeLists.txt`

- [ ] Create `PlanckTypes.h` using the public type shapes from this plan.
- [ ] Create `PlanckCollisionPolicy.h` with pure helpers:
  - `decodeFilterInfo`
  - `encodeFilterInfo`
  - `shouldEnableLayerPair`
  - `shouldActorBodyCollideWithPlayerSource`
- [ ] Define verified constants only after checking FO4VR:
  - player hand layer `43`
  - weapon layer `44`
  - charcontroller layer `30`
  - biped-related layers from FO4VR filter logic
- [ ] Write `PlanckPolicyTests.cpp` with these cases:
  - filter layer decodes from low 7 bits.
  - group decodes from high 16 bits.
  - bit 14 is represented as `collisionSuppressed`.
  - hand and weapon sources collide with active biped bodies.
  - player-owned bodies do not collide with player-owned hand/weapon sources.
  - held actor state suppresses player charcontroller collision while preserving hand/weapon collision.
  - charcontroller fallback is hittable by hands/weapons but not by broad player capsule rules.
- [ ] Update CMake so the tests build under `BUILD_ROCK_TESTS`.

**Acceptance:**

- `cmake --build build-tests --config Release` builds the policy test executable when the test preset is configured.
- Policy tests pass.
- No runtime hook or native call is introduced in this task.

### Task 3: Add PLANCK Configuration

**Files:**

- Modify: `src/RockConfig.h`
- Modify: `src/RockConfig.cpp`
- Modify: `data/config/ROCK.ini`

- [ ] Add config fields:

```cpp
bool rockPlanckEnabled = false;
bool rockPlanckDebugLogging = false;
bool rockPlanckEnableActorShove = true;
bool rockPlanckEnablePlayerBipedCollision = true;
bool rockPlanckEnableBipedSelfCollision = false;
bool rockPlanckEnableCharControllerFallback = true;
float rockPlanckActiveRagdollStartDistance = 50.0f;
float rockPlanckActiveRagdollEndDistance = 60.0f;
float rockPlanckShoveSpeedThreshold = 2.5f;
float rockPlanckHandImpulseMultiplier = 1.0f;
float rockPlanckWeaponImpulseMultiplier = 1.0f;
int rockPlanckMaxActiveActors = 8;
```

- [ ] Load each field from `[PhysicsInteraction]` in `RockConfig.cpp`.
- [ ] Add an INI block documenting that PLANCK behavior is disabled unless `bPlanckEnabled=true`.
- [ ] Keep `bCollideWithCharControllers` as legacy/debug behavior. Do not use it as the PLANCK implementation switch.

**Acceptance:**

- Defaults preserve current behavior with PLANCK disabled.
- INI load failure still falls back to compiled defaults.
- Debug logging can be enabled without enabling ragdoll hooks.

### Task 4: Create Contact Classification Without Changing Behavior

**Files:**

- Create: `src/physics-interaction/PlanckContactTracker.h`
- Create: `src/physics-interaction/PlanckContactTracker.cpp`
- Modify: `src/physics-interaction/PhysicsInteraction.h`
- Modify: `src/physics-interaction/PhysicsInteraction.cpp`

- [ ] Add a `PlanckContactTracker` member to `PhysicsInteraction`.
- [ ] Pass raw contact event data, current hand body ids, weapon collision lookup access, and world pointers into the tracker.
- [ ] Keep existing contact behavior in `PhysicsInteraction::handleContactEvent` intact:
  - held-body contact notifications,
  - left-hand weapon-part detection,
  - `_lastContactBodyRight`,
  - `_lastContactBodyLeft`,
  - throttled hand contact logging.
- [ ] In the tracker, parse:
  - body id A/B from event `+0x08/+0x0C`,
  - manifold pointer from `+0x10`,
  - contact index/type from `+0x18/+0x19`,
  - separating velocity from `+0x30` only after re-verifying type and size.
- [ ] Classify source as:
  - right hand,
  - left hand,
  - weapon body,
  - held object,
  - unknown.
- [ ] Classify target as:
  - actor biped body,
  - actor charcontroller,
  - clutter/world/player body/unknown.
- [ ] Store the last classified contacts in a bounded ring buffer for diagnostics and manager consumption.

**Acceptance:**

- With `rockPlanckEnabled=false`, behavior and logs match existing hand/weapon contact behavior.
- With debug logging enabled, classified contacts appear at a throttled rate.
- No impulse, ragdoll activation, or collision policy change happens in this task.

### Task 5: Add Impulse Queue With Existing Verified Wrappers

**Files:**

- Create: `src/physics-interaction/PlanckImpulseQueue.h`
- Create: `src/physics-interaction/PlanckImpulseQueue.cpp`
- Modify: `src/physics-interaction/PhysicsInteraction.h`
- Modify: `src/physics-interaction/PhysicsInteraction.cpp`
- Modify: `src/physics-interaction/BethesdaPhysicsBody.h/.cpp` only if a wrapper gap is found.

- [ ] Implement a bounded queue of `planck::ImpulseJob`.
- [ ] Use a mutex or lock-free structure only after confirming the call paths. The first implementation must be correct under physics callback and update-thread access.
- [ ] Add methods:
  - `bool enqueue(const ImpulseJob& job)`
  - `void clear()`
  - `void flush(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld)`
- [ ] Resolve target body to `bhkNPCollisionObject` using verified existing ROCK paths or a newly verified bridge.
- [ ] Apply linear impulses through `0x141e08520`.
- [ ] Apply point impulses through `0x141e08640`.
- [ ] Drop jobs whose target actor/body is stale, deleted, disabled, out of world, or has invalid body id.
- [ ] Add throttled logs for dropped jobs grouped by reason.

**Acceptance:**

- Queue flush with no jobs is a no-op.
- Queue overflow drops new jobs with throttled diagnostics.
- Existing hand/weapon behavior remains unchanged until later tasks enqueue jobs.

### Task 6: Verify Actor Enumeration And Create Ragdoll Manager Skeleton

**Files:**

- Create: `src/physics-interaction/PlanckRagdollManager.h`
- Create: `src/physics-interaction/PlanckRagdollManager.cpp`
- Modify: `src/physics-interaction/PhysicsInteraction.h`
- Modify: `src/physics-interaction/PhysicsInteraction.cpp`
- Modify: `docs/planck-port-verification.md`

- [ ] Use Ghidra to verify the FO4VR actor enumeration source before implementation.
  - Candidate sources may include process manager high actors, loaded cell actor lists, or graph manager ownership paths.
  - Do not use CommonLib field assumptions without binary verification.
- [ ] Record the chosen enumeration path in the verification ledger.
- [ ] Create `PlanckRagdollManager` with lifecycle methods:
  - `init`
  - `shutdown`
  - `update`
  - `onContact`
  - `flushPrePhysics`
  - `onCellOrWorldChanged`
- [ ] Track actors by stable identity:
  - `TESObjectREFR*`
  - form id
  - current 3D pointer
  - current actor state
  - current collision group
  - last seen frame/time
  - ragdoll driver pointer once verified
- [ ] Implement state transitions without enabling ragdolls:
  - `Inactive -> Candidate` when actor is valid and within start distance.
  - `Candidate -> Inactive` when actor becomes invalid or exceeds end distance.
  - any state -> `Disabled` on deleted/disabled/unloaded actor.
- [ ] Limit candidates by `rockPlanckMaxActiveActors`.

**Acceptance:**

- PLANCK disabled: manager does not enumerate or mutate actor state.
- PLANCK enabled with debug logging: candidates are logged with form id, distance, and chosen state.
- No ragdoll is added to the world in this task.

### Task 7: Verify Graph/Ragdoll Bridge And Add Wrappers

**Files:**

- Create: `src/physics-interaction/RagdollGraphBridge.h`
- Create: `src/physics-interaction/RagdollGraphBridge.cpp`
- Modify: `src/physics-interaction/HavokOffsets.h`
- Modify: `docs/planck-port-verification.md`

- [ ] Use Ghidra to verify FO4VR equivalents for:
  - actor has ragdoll graph,
  - graph manager lock/unlock requirements,
  - add ragdoll to world,
  - remove ragdoll from world,
  - set graph/ragdoll world,
  - set ragdoll constraints from bhk constraints,
  - resolve `hkbRagdollDriver*` from actor graph.
- [ ] Record each verified native operation in the ledger before adding offsets.
- [ ] Add bridge methods:

```cpp
namespace frik::rock::planck
{
    struct RagdollGraphHandles
    {
        void* graphManager = nullptr;
        void* behaviorGraph = nullptr;
        void* ragdollDriver = nullptr;
        void* ragdollInterface = nullptr;
        bool hasRagdoll = false;
        bool hasWorld = false;
    };

    RagdollGraphHandles resolveRagdollGraph(RE::TESObjectREFR* actorRef);
    bool ensureRagdollWorld(RE::TESObjectREFR* actorRef, RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld);
    bool addRagdollToWorld(RE::TESObjectREFR* actorRef, RagdollGraphHandles& handles);
    bool removeRagdollFromWorld(RE::TESObjectREFR* actorRef, RagdollGraphHandles& handles);
}
```

- [ ] Wrap native calls with SEH guards only at boundaries that can be reached during cell/world transitions.
- [ ] Return explicit failure reasons through logs; do not throw exceptions from bridge code.

**Acceptance:**

- Bridge can identify whether a nearby actor has ragdoll support.
- Bridge does not add/remove ragdolls until called by the manager in a later task.
- Failure paths leave actor state recoverable.

### Task 8: Add Charcontroller Fallback Grouping

**Files:**

- Modify: `src/physics-interaction/PlanckRagdollManager.cpp`
- Modify: `src/physics-interaction/PlanckCollisionPolicy.h`
- Modify: `src/physics-interaction/ObjectDetection.cpp`
- Modify: `tests/PlanckPolicyTests.cpp`

- [ ] For actors with no valid ragdoll graph or no verified add-ragdoll path, move state to `CharControllerFallback`.
- [ ] Identify the actor's collision group from verified body/filter info only.
- [ ] Add the group to a manager-owned `hittableCharControllerGroups` set.
- [ ] Update contact classification so layer `30` can be recognized for PLANCK contact purposes without making layer `30` grabbable by object detection.
- [ ] Enqueue shove/impulse jobs only when:
  - source is right hand, left hand, weapon, or held object,
  - target group is in `hittableCharControllerGroups`,
  - source is not player-owned charcontroller,
  - cooldown and threshold checks pass.

**Acceptance:**

- Hands/weapons can produce classified contacts against fallback NPC charcontrollers.
- Object grab selection does not start selecting generic charcontrollers.
- Player capsule collision is not globally enabled.

### Task 9: Add Active Ragdoll Lifecycle

**Files:**

- Modify: `src/physics-interaction/PlanckRagdollManager.cpp`
- Modify: `src/physics-interaction/RagdollGraphBridge.cpp`
- Modify: `src/physics-interaction/PlanckCollisionPolicy.h`
- Modify: `docs/planck-port-verification.md`

- [ ] Transition `Candidate -> AddingRagdoll` only when:
  - actor is valid,
  - actor has 3D,
  - actor has verified ragdoll graph support,
  - actor is within start distance,
  - active actor budget allows activation.
- [ ] Call `RagdollGraphBridge::ensureRagdollWorld`.
- [ ] Call `RagdollGraphBridge::addRagdollToWorld`.
- [ ] Resolve and store:
  - `hkbRagdollDriver*`,
  - ragdoll interface pointer,
  - actor group,
  - initial biped body ids if available.
- [ ] Transition to `ActiveBlendIn`.
- [ ] Keep active actor groups in manager-owned sets:
  - active biped groups,
  - self-collidable biped groups,
  - no-player-charcontroller groups,
  - hittable charcontroller groups.
- [ ] Transition active actors to blend-out/removal when:
  - actor exceeds end distance,
  - actor unloads,
  - actor is disabled/deleted,
  - ragdoll driver disappears,
  - world pointer changes.
- [ ] Cleanup must remove groups and clear stale body ids.

**Acceptance:**

- Ragdoll add/remove is visible in logs and bounded by distance/budget.
- Actors leave active state cleanly on unload or distance exit.
- No stale collision groups remain after shutdown or cell change.

### Task 10: Implement Group-Aware Collision Policy

**Files:**

- Modify: `src/physics-interaction/PhysicsInteraction.cpp`
- Modify: `src/physics-interaction/CollisionLayerPolicy.h`
- Modify: `src/physics-interaction/PlanckCollisionPolicy.h`
- Modify: `src/physics-interaction/PlanckRagdollManager.cpp`
- Modify: `tests/PlanckPolicyTests.cpp`

- [ ] Replace hand/weapon broad layer setup with named policy calls while preserving current masks when PLANCK is disabled.
- [ ] When PLANCK is enabled:
  - hand layer `43` and weapon layer `44` may collide with biped/fallback targets only through group-aware rules.
  - player-owned bodies do not collide with player-owned bodies.
  - active actor biped groups collide with hand and weapon sources.
  - actor self-collision follows `rockPlanckEnableBipedSelfCollision`.
  - actor charcontroller fallback groups collide with hand and weapon sources only.
  - held actor groups use held-state suppression for player charcontroller collision.
- [ ] Do not enable weapon-vs-projectile collision outside existing weapon projectile config.
- [ ] Re-register or repair the matrix if external code mutates the ROCK layer masks.

**Acceptance:**

- PLANCK disabled produces the same hand/weapon layer masks as before.
- PLANCK enabled produces deterministic masks and group policy decisions in tests.
- Layer `30` remains controlled by PLANCK policy, not by broad layer enabling.

### Task 11: Install Ragdoll Driver Hooks

**Files:**

- Create: `src/physics-interaction/RagdollDriverHooks.h`
- Create: `src/physics-interaction/RagdollDriverHooks.cpp`
- Modify: `src/physics-interaction/PhysicsHooks.h`
- Modify: `src/physics-interaction/PhysicsHooks.cpp`
- Modify: `src/physics-interaction/HavokOffsets.h`
- Modify: `docs/planck-port-verification.md`

- [ ] Use Ghidra to re-verify hook prologues, stolen bytes, calling convention, and trampoline safety at:
  - `0x14177e7c0`
  - `0x14177fce0`
- [ ] Record hook-site bytes and reasoning in the verification ledger.
- [ ] Implement hook installation with the same safety level as existing ROCK hooks or a stronger reusable trampoline helper.
- [ ] Add enabled guard:
  - if ROCK hooks disabled, call original immediately.
  - if PLANCK disabled, call original immediately.
  - if driver is unknown to manager, call original immediately.
- [ ] Pre-drive hook responsibilities:
  - find `ActiveRagdoll` state for driver,
  - capture animation pose inputs needed by manager,
  - apply blend/motor/control-track changes before original path,
  - avoid mutating unrelated actors.
- [ ] Post hook responsibilities:
  - capture resulting ragdoll pose,
  - update blend-out/restoration state,
  - hand off any deferred state transitions to manager.
- [ ] All hook bodies must catch SEH at native boundary and fail closed by disabling PLANCK hooks.

**Acceptance:**

- Hooks installed only once.
- Hooks can be disabled during shutdown/world transition.
- With PLANCK enabled but no active actor, originals run with no behavior change.

### Task 12: Port Active Ragdoll Blend And Motor Control

**Files:**

- Modify: `src/physics-interaction/PlanckTypes.h`
- Modify: `src/physics-interaction/PlanckRagdollManager.h/.cpp`
- Modify: `src/physics-interaction/RagdollDriverHooks.cpp`
- Create if needed: `src/physics-interaction/PlanckPoseBlender.h/.cpp`

- [ ] Map PLANCK `ActiveRagdoll` fields to FO4VR-native equivalents:
  - animation pose,
  - ragdoll pose,
  - low-resolution pose if available,
  - world-from-model,
  - sticky world-from-model,
  - blend timers,
  - stress,
  - knock state,
  - motor tuning state,
  - disable-motors-one-frame state.
- [ ] Verify FO4VR generator output track layout before reading or writing tracks.
- [ ] Implement blend-in and blend-out state transitions:
  - `ActiveBlendIn -> Active`
  - `Active -> ActiveBlendOut`
  - `ActiveBlendOut -> RemovingRagdoll`
- [ ] Port PLANCK motor behavior only after confirming hknp ragdoll controller fields:
  - gain,
  - tau,
  - damping,
  - max force,
  - gravity flag,
  - rigid/powered control mode.
- [ ] Keep all field writes localized to verified driver/controller objects.

**Acceptance:**

- Active actor enters ragdoll without snapping to invalid pose.
- Active actor returns to animation control without stale motor state.
- Failure to verify a field blocks that specific field write and is recorded in the ledger.

### Task 13: Connect Contacts To Shove And Push Impulses

**Files:**

- Modify: `src/physics-interaction/PlanckContactTracker.cpp`
- Modify: `src/physics-interaction/PlanckImpulseQueue.cpp`
- Modify: `src/physics-interaction/PlanckRagdollManager.cpp`
- Modify: `src/physics-interaction/PhysicsInteraction.cpp`

- [ ] Compute source velocity for:
  - right hand body,
  - left hand body,
  - weapon body,
  - held object.
- [ ] Use configurable threshold `rockPlanckShoveSpeedThreshold`.
- [ ] Scale impulse by:
  - hand/weapon config multiplier,
  - contact target type,
  - active ragdoll vs fallback charcontroller,
  - cooldown.
- [ ] For active ragdoll biped bodies:
  - apply point impulse to contacted body when collision object is resolvable.
  - mark actor stress/knock state through manager.
- [ ] For charcontroller fallback:
  - enqueue actor shove through verified character-controller or actor motion path.
  - if no safe native shove path is verified, log classification and do not fabricate one.
- [ ] Add cooldowns per actor/source pair to prevent repeated impulse spam.

**Acceptance:**

- Hands push active ragdoll bodies.
- Weapon body contacts push active ragdoll bodies.
- Fallback charcontroller actors receive only verified shove behavior.
- Repeated contact does not flood impulses or logs.

### Task 14: Integrate Pre-Physics Flush

**Files:**

- Modify: `src/physics-interaction/PhysicsInteraction.cpp`
- Modify: `src/physics-interaction/RagdollDriverHooks.cpp`
- Modify: `src/physics-interaction/PlanckImpulseQueue.cpp`

- [ ] Choose one verified pre-physics flush point:
  - preferred: inside verified ragdoll scene order after driver preparation and before world step,
  - alternate: existing update path only if Ghidra confirms it occurs before hknp step.
- [ ] Flush `PlanckImpulseQueue` in that window.
- [ ] Ensure queued work cannot run after shutdown begins.
- [ ] Clear queue on world/cell transition.

**Acceptance:**

- Impulses are applied during a physics-safe window.
- Shutdown and cell transition do not leave queued native pointers behind.

### Task 15: Add Debugging And Diagnostics

**Files:**

- Create: `src/physics-interaction/PlanckDebug.h`
- Create: `src/physics-interaction/PlanckDebug.cpp`
- Modify: `src/physics-interaction/PlanckRagdollManager.cpp`
- Modify: `src/physics-interaction/PlanckContactTracker.cpp`
- Modify: `src/physics-interaction/PlanckImpulseQueue.cpp`
- Modify: `data/config/ROCK.ini`

- [ ] Add throttled debug categories:
  - actor candidate/activation,
  - group policy,
  - contact classification,
  - impulse enqueue/drop/flush,
  - driver hook state,
  - cleanup.
- [ ] Add counters:
  - active actor count,
  - fallback actor count,
  - contacts classified,
  - impulses enqueued,
  - impulses dropped by reason,
  - stale actors cleaned.
- [ ] Logs must include form id and group id when known.
- [ ] Logs must avoid per-contact spam unless `rockPlanckDebugLogging=true`.

**Acceptance:**

- Debug output is enough to diagnose why an actor did or did not become pushable.
- Default config does not generate heavy logs.

### Task 16: In-Game Acceptance Pass

**Files:**

- No source file required unless bugs are found.
- Update: `docs/planck-port-verification.md` with final observed runtime notes.

- [ ] Build:

```powershell
cd E:\fo4dev\PROJECT_ROCK_V2\ROCK
$env:VCPKG_ROOT = "C:\vcpkg"
cmake --build build --config Release
```

- [ ] Test with PLANCK disabled:
  - hand collision still works,
  - weapon collision still works when enabled,
  - grabbing and release still work,
  - reload/two-handed behavior remains intact.
- [ ] Test with PLANCK enabled against one idle NPC:
  - hand contact classifies actor target,
  - weapon contact classifies actor target,
  - actor enters active or fallback state,
  - actor receives push/shove behavior,
  - no player capsule instability.
- [ ] Test against multiple NPCs:
  - active actor cap respected,
  - groups do not leak,
  - no self-collision leak to player.
- [ ] Test transitions:
  - leave activation range,
  - change cell,
  - disable/delete target actor,
  - kill target actor,
  - load save with PLANCK enabled.
- [ ] Test failure cases:
  - actor without valid ragdoll graph,
  - actor with missing 3D,
  - stale body id contact,
  - missing bhk/hknp world pointer.

**Acceptance:**

- NPCs can be pushed by hands and held guns through verified active-ragdoll or verified fallback behavior.
- Current ROCK hand/weapon/grab/reload behavior is preserved.
- No unverified native write remains in the implementation.
- Verification ledger is complete enough for a blind audit.

---

## Build And Test Commands

Configure tests if needed:

```powershell
cd E:\fo4dev\PROJECT_ROCK_V2\ROCK
$env:VCPKG_ROOT = "C:\vcpkg"
cmake -S . -B build-tests -DBUILD_ROCK_TESTS=ON -DCMAKE_BUILD_TYPE=Release
```

Build plugin:

```powershell
cd E:\fo4dev\PROJECT_ROCK_V2\ROCK
$env:VCPKG_ROOT = "C:\vcpkg"
cmake --build build --config Release
```

Build tests:

```powershell
cd E:\fo4dev\PROJECT_ROCK_V2\ROCK
$env:VCPKG_ROOT = "C:\vcpkg"
cmake --build build-tests --config Release
```

Run existing focused test executable:

```powershell
E:\fo4dev\PROJECT_ROCK_V2\ROCK\build-tests\Release\ROCKTransformConventionTests.exe
```

Run new PLANCK policy test executable if implemented as a separate target:

```powershell
E:\fo4dev\PROJECT_ROCK_V2\ROCK\build-tests\Release\ROCKPlanckPolicyTests.exe
```

---

## Verification Questions That Must Be Answered In Ghidra Before Code Uses Them

- What is the FO4VR-authoritative actor enumeration path for high/loaded actors?
- Which FO4VR graph manager calls correspond to:
  - has ragdoll,
  - add ragdoll to world,
  - remove ragdoll from world,
  - set graph world,
  - set ragdoll constraints from bhk constraints?
- How does FO4VR expose or store `hkbRagdollDriver*` per actor graph?
- Which exact driver fields correspond to PLANCK's Skyrim driver fields?
- What is the FO4VR generator output track layout for:
  - pose,
  - world-from-model,
  - keyframed ragdoll bones,
  - rigid body ragdoll controls,
  - powered ragdoll controls?
- Which hknp ragdoll controller fields safely control:
  - gain,
  - tau,
  - damping,
  - max force,
  - gravity,
  - powered/rigid mode?
- What is the safest verified actor/charcontroller shove path for fallback actors?
- What is the correct body-to-actor ownership mapping for biped ragdoll bodies?
- What hook prologue/stolen-byte lengths are safe at `0x14177e7c0` and `0x14177fce0`?
- Which world/cell transition signals should trigger PLANCK cleanup?

---

## Success Criteria

- With PLANCK disabled, ROCK behaves as it does before the port.
- With PLANCK enabled, nearby NPCs become physically interactable through active ragdoll when verified support exists.
- NPCs without active ragdoll support use verified charcontroller fallback behavior without enabling broad player capsule collision.
- Hands and held guns can push NPCs through contact and impulse paths.
- Multiple NPCs can be active without stale groups, self-collision leaks, or player-body instability.
- All new offsets and native writes are supported by Ghidra verification rows.
- The final implementation can survive a blind verification audit without relying on existing markdown claims.

