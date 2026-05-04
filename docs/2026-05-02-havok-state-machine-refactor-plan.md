# ROCK Havok Facade and Hand State Machine Refactor Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement this plan task by task. Steps use checkbox syntax for tracking.

**Goal:** Replace scattered hand interaction state writes and duplicated Havok access with explicit internal architecture: one hand state machine, one Havok runtime facade, and one per-frame physics context.

**Architecture:** The first implementation pass is behavior-preserving. ROCK keeps the current dynamic grab model, direct skeleton bone data, and FRIK IK/pose integration while moving repeated low-level runtime work into focused internal files. HIGGS is used as the structural reference for having a dedicated Havok/RE layer and explicit hand state lifecycle, but Skyrim offsets and HIGGS keyframed held-object behavior are not copied into ROCK.

**Tech Stack:** C++23, CMake, VS2022, F4VR-CommonFramework, CommonLibF4VR, FO4VR hknp runtime wrappers, existing ROCK focused tests under `ROCK/tests`.

---

## Decision Note

This refactor is being planned as a staged architecture change because the current code has correct domain pieces but too many hidden coupling points: direct `_state` writes, repeated body/motion array access, duplicated Havok allocator wrappers, and repeated frame-time/world/hand transform resolution. The alternatives were to keep cleaning individual call sites or to split large files immediately. Cleaning isolated call sites would leave the same structural problem in place, and splitting large files before ownership boundaries are explicit would make later review harder. The chosen approach first creates stable internal APIs, then migrates call sites through those APIs, then removes duplicates once the code proves equivalent.

## Activation Gate

- [ ] Do not start implementation while another agent is actively rewriting reload internals inside ROCK.
- [ ] Treat reload-related compile failures as outside this plan until the reload extraction lands or the user says to repair that surface here.
- [ ] Do not use `git` for inventory, verification, checkpointing, or rollback. The user stated the repository state is not authoritative.
- [ ] Do not use Ghidra during this refactor unless a new offset, signature, or FO4VR runtime claim is required. If that happens, stop, explain why Ghidra is needed, and wait for approval.
- [ ] Use only already verified offsets from `ROCK/src/physics-interaction/HavokOffsets.h` and the existing Havok notes under `libraries_and_tools/havok`.
- [ ] Keep this pass behavior-preserving: no gameplay tuning, no old keyframed held-object modes, no new INI options.

## Current Mapping

The plan is based on these repo facts from the current workspace:

- `ROCK/src/physics-interaction/HandState.h` currently defines 11 states: `Idle`, `SelectedClose`, `SelectedFar`, `SelectionLocked`, `HeldInit`, `HeldBody`, `Pulled`, `PreGrabItem`, `GrabFromOtherHand`, `SelectedTwoHand`, and `HeldTwoHanded`.
- Direct state writes currently exist in `Hand.cpp` and `HandGrab.cpp`, including selection, far-lock, pull, held-init, held-body, and release paths.
- `PreGrabItem` appears to be an enum-only HIGGS carryover in current ROCK code. It should be removed during the state machine task after a fresh `rg` confirms no live caller.
- `SelectedTwoHand` and `HeldTwoHanded` should not be removed without checking the active weapon support path first. Current two-handed weapon behavior is mostly owned by `TwoHandedGrip`/weapon support code, so the state machine task must either wire these states through one transition API or remove the unused enum values only after proving the active path does not use them.
- `PhysicsUtils.h`, `BodyCollisionControl.cpp`, `BethesdaPhysicsBody.cpp`, `GrabConstraint.cpp`, `PushAssist.cpp`, `WeaponCollision.cpp`, `HandGrab.cpp`, diagnostics, and registries all touch Havok runtime details directly or indirectly.
- Duplicated/fragile Havok work includes body array access, motion array access, body-slot validity checks, collision filter reads/writes, broadphase toggles, velocity/keyframe writes, body backpointer reads, modifier/filter pointer reads, and Havok heap allocation through TLS.
- `PhysicsInteraction::update()` repeatedly resolves the player `bhkWorld`, `hknpWorld`, frame delta, hand transforms, hand nodes, weapon source transforms, and contact/debug inputs. A per-frame context should own this once per frame.
- HIGGS reference files support the direction, not the exact code: `include/RE/havok.h`, `src/RE/havok.cpp`, and `analysis/subsystems/12_hand_class.md` show a dedicated Havok integration layer and explicit hand lifecycle, while also documenting that FO4VR offsets and wrapper layouts must be separately verified.

## Target File Structure

Create:

- `ROCK/src/physics-interaction/HavokRuntime.h`
- `ROCK/src/physics-interaction/HavokRuntime.cpp`
- `ROCK/src/physics-interaction/HandInteractionStateMachine.h`
- `ROCK/src/physics-interaction/HandInteractionStateMachine.cpp`
- `ROCK/src/physics-interaction/PhysicsFrameContext.h`
- `ROCK/tests/HavokRuntimePolicyTests.cpp`
- `ROCK/tests/HandStateMachineTests.cpp`

Modify:

- `ROCK/src/physics-interaction/PhysicsUtils.h`
- `ROCK/src/physics-interaction/BodyCollisionControl.h`
- `ROCK/src/physics-interaction/BodyCollisionControl.cpp`
- `ROCK/src/physics-interaction/BethesdaPhysicsBody.cpp`
- `ROCK/src/physics-interaction/GrabConstraint.cpp`
- `ROCK/src/physics-interaction/PushAssist.cpp`
- `ROCK/src/physics-interaction/WeaponCollision.cpp`
- `ROCK/src/physics-interaction/Hand.h`
- `ROCK/src/physics-interaction/Hand.cpp`
- `ROCK/src/physics-interaction/HandGrab.cpp`
- `ROCK/src/physics-interaction/PhysicsInteraction.h`
- `ROCK/src/physics-interaction/PhysicsInteraction.cpp`
- `ROCK/CMakeLists.txt`

Keep as data/reference only:

- `ROCK/src/physics-interaction/HavokOffsets.h`
- `ROCK/src/physics-interaction/PhysicsBodyFrame.h`
- `ROCK/src/physics-interaction/PhysicsScale.h`
- `ROCK/src/physics-interaction/TransformMath.h`

## New Internal Interfaces

### HavokRuntime

`HavokRuntime` becomes the only normal place for low-level hknp body and motion access. Files that build shapes, diagnostics, or verified offset documentation may still contain low-level details, but ordinary interaction logic should not.

Planned public surface:

```cpp
namespace frik::rock::havok_runtime
{
    struct BodySnapshot
    {
        bool valid = false;
        RE::hknpBodyId bodyId{ body_frame::kInvalidBodyId };
        std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
        std::uint32_t collisionFilterInfo = 0;
        RE::hknpBody* body = nullptr;
        RE::hknpMotion* motion = nullptr;
        RE::NiCollisionObject* collisionObject = nullptr;
        RE::NiAVObject* ownerNode = nullptr;
    };

    struct ResolvedBodyWorldTransform
    {
        bool valid = false;
        RE::NiTransform transform{};
        body_frame::BodyFrameSource source = body_frame::BodyFrameSource::Fallback;
        std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
    };

    bool isValidBodyId(RE::hknpBodyId bodyId);
    bool bodySlotLooksReadable(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    BodySnapshot snapshotBody(RE::hknpWorld* world, RE::hknpBodyId bodyId);

    RE::NiTransform bodyArrayWorldTransform(const RE::hknpBody& body);
    bool tryGetMotionWorldTransform(RE::hknpWorld* world, const RE::hknpBody& body, RE::NiTransform& outTransform);
    ResolvedBodyWorldTransform resolveLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId);

    bool tryReadFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t& outFilterInfo);
    bool setFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t filterInfo, std::uint32_t rebuildMode = 0);
    bool setBroadPhaseEnabled(RE::hknpWorld* world, RE::hknpBodyId bodyId, bool enabled);

    bool setBodyVelocityDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const float* linearVelocity, const float* angularVelocity);
    bool applyLinearVelocityDeltaDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiPoint3& deltaHavok);
    bool rebuildMotionMassProperties(RE::hknpWorld* world, std::uint32_t motionIndex, std::uint32_t mode);

    void* allocateHavok(std::size_t size);
    void freeHavok(void* ptr, std::size_t size);
    bool hkArrayReserveMore(void* array, int elementSize);
}
```

Rules:

- `HavokOffsets.h` remains the offset table. `HavokRuntime.cpp` becomes the normal relocation caller.
- `PhysicsUtils.h` remains temporarily as compatibility wrappers around `havok_runtime`, then should shrink to scale/transform helpers only.
- `BodyCollisionControl.*` remains temporarily as a compatibility namespace around `havok_runtime` so existing call sites can be migrated in small passes.
- No file outside `HavokRuntime.cpp`, `HavokOffsets.h`, shape construction, or diagnostic-only code should directly read `kHknpWorld_ModifierManager`, `kBody_CollisionObjectBackPointer`, `kFunc_SetBodyCollisionFilterInfo`, `kFunc_SetBodyBroadPhaseEnabled`, `kData_HavokTlsAllocKey`, or the raw motion array pointer.

### HandInteractionStateMachine

The state machine owns legal transitions and state classification. `Hand` still owns the physical side effects because those side effects need object handles, constraints, collision suppression, finger runtime, and telemetry.

Planned public surface:

```cpp
namespace frik::rock
{
    enum class HandInteractionEvent : std::uint8_t
    {
        Initialize,
        SelectionFoundClose,
        SelectionFoundFar,
        SelectionLost,
        LockFarSelection,
        BeginPull,
        PullArrivedClose,
        BeginGrabCommit,
        GrabCommitSucceeded,
        HeldFadeComplete,
        ReleaseRequested,
        ObjectInvalidated,
        WorldInvalidated,
        BeginOtherHandTransfer,
        CompleteOtherHandTransfer,
        BeginTwoHandSelection,
        BeginTwoHandHold,
        EndTwoHandHold
    };

    enum class HandTransitionEffect : std::uint32_t
    {
        None = 0,
        ClearSelection = 1u << 0,
        RememberDeselect = 1u << 1,
        LockSelection = 1u << 2,
        StartPull = 1u << 3,
        RestorePulledCollision = 1u << 4,
        CommitGrab = 1u << 5,
        ReleaseHeld = 1u << 6,
        ClearHeldRuntime = 1u << 7,
        ClearFingerPose = 1u << 8,
        EnterTwoHandAuthority = 1u << 9,
        ExitTwoHandAuthority = 1u << 10
    };

    struct HandTransitionRequest
    {
        HandState current = HandState::Idle;
        HandInteractionEvent event = HandInteractionEvent::Initialize;
        bool hasSelection = false;
        bool selectionIsFar = false;
        bool hasHeldBody = false;
        bool otherHandTransferPending = false;
        bool twoHandCandidateValid = false;
    };

    struct HandTransitionResult
    {
        bool accepted = false;
        HandState next = HandState::Idle;
        std::uint32_t effects = 0;
        const char* reason = "";
    };

    HandTransitionResult evaluateHandTransition(const HandTransitionRequest& request);
    bool isHoldingState(HandState state);
    bool canUpdateSelectionFromState(HandState state);
    bool hasExclusiveObjectSelection(HandState state);
}
```

Required first-pass transition table:

| Current state | Event | Next state | Effects |
| --- | --- | --- | --- |
| Any | `Initialize` | `Idle` | `ClearSelection`, `ClearHeldRuntime`, `ClearFingerPose` |
| `Idle`, `SelectedClose`, `SelectedFar` | `SelectionFoundClose` | `SelectedClose` | none |
| `Idle`, `SelectedClose`, `SelectedFar` | `SelectionFoundFar` | `SelectedFar` | none |
| `SelectedClose`, `SelectedFar` | `SelectionLost` | `Idle` | `ClearSelection`, `RememberDeselect` |
| `SelectedFar` | `LockFarSelection` | `SelectionLocked` | `LockSelection` |
| `SelectionLocked` | `BeginPull` | `Pulled` | `StartPull` |
| `Pulled` | `PullArrivedClose` | `SelectedClose` | `RestorePulledCollision` |
| `SelectedClose`, `SelectedFar`, `SelectionLocked` | `BeginGrabCommit` | current state | `CommitGrab` |
| `SelectedClose`, `SelectedFar`, `SelectionLocked` | `GrabCommitSucceeded` | `HeldInit` | none |
| `HeldInit` | `HeldFadeComplete` | `HeldBody` | none |
| `HeldInit`, `HeldBody`, `Pulled`, `SelectionLocked`, `SelectedClose`, `SelectedFar` | `ReleaseRequested` | `Idle` | `ReleaseHeld`, `ClearSelection`, `ClearHeldRuntime`, `ClearFingerPose` |
| `HeldInit`, `HeldBody`, `Pulled`, `SelectionLocked`, `SelectedClose`, `SelectedFar` | `ObjectInvalidated` | `Idle` | `ReleaseHeld`, `ClearSelection`, `ClearHeldRuntime`, `ClearFingerPose` |
| Any non-idle state | `WorldInvalidated` | `Idle` | `ReleaseHeld`, `ClearSelection`, `ClearHeldRuntime`, `ClearFingerPose`, `ExitTwoHandAuthority` |
| `SelectedClose`, `HeldBody` | `BeginOtherHandTransfer` | `GrabFromOtherHand` | none |
| `GrabFromOtherHand` | `CompleteOtherHandTransfer` | `HeldInit` | `CommitGrab` |
| `Idle`, `SelectedClose` | `BeginTwoHandSelection` | `SelectedTwoHand` | none |
| `SelectedTwoHand` | `BeginTwoHandHold` | `HeldTwoHanded` | `EnterTwoHandAuthority` |
| `HeldTwoHanded`, `SelectedTwoHand` | `EndTwoHandHold` | `Idle` | `ExitTwoHandAuthority`, `ClearFingerPose` |

Rules:

- Any transition not listed returns `accepted=false` and keeps the current state.
- The state machine does not call Havok, FRIK, CommonLib, or game APIs.
- `Hand::_prevState` is updated only by a single `Hand::applyTransition(...)` helper.
- `PreGrabItem` is removed if a fresh search proves no caller. If a caller appears because another agent changed code, the state is retained and given explicit transitions instead of remaining implicit.
- `SelectionStatePolicy.h` should become a compatibility include over the new state machine classification functions, then be removed when call sites are migrated.

### PhysicsFrameContext

`PhysicsFrameContext` is a read-only frame bundle created once at the top of `PhysicsInteraction::update()`.

Planned surface:

```cpp
namespace frik::rock
{
    struct HandFrameInput
    {
        bool isLeft = false;
        RE::NiTransform rawHandWorld{};
        RE::NiNode* handNode = nullptr;
        RE::NiPoint3 grabAnchorWorld{};
        RE::NiPoint3 palmNormalWorld{};
        RE::NiPoint3 pointingWorld{};
        bool disabled = false;
    };

    struct PhysicsFrameContext
    {
        RE::bhkWorld* bhkWorld = nullptr;
        RE::hknpWorld* hknpWorld = nullptr;
        float deltaSeconds = 1.0f / 90.0f;
        bool worldReady = false;
        bool menuBlocked = false;
        bool reloadBoundaryActive = false;
        HandFrameInput right{};
        HandFrameInput left{};
    };
}
```

Rules:

- Build this once per frame in `PhysicsInteraction::update()`.
- Clamp/normalize `deltaSeconds` once.
- Resolve both raw hand transforms and hand nodes once.
- Pass the context or the relevant `HandFrameInput` into selection, grab input, collision updates, debug overlay, weapon support, and contact handling.
- Reload remains a boundary flag or external-provider integration point in this plan; do not redesign reload internals here.

## Implementation Tasks

### Task 1: Preflight Inventory and Baseline Notes

**Files:**

- Read: `ROCK/src/physics-interaction/*.h`
- Read: `ROCK/src/physics-interaction/*.cpp`
- Read: `E:/fo4dev/skirymvr_mods/source_codes/higgs/include/RE/havok.h`
- Read: `E:/fo4dev/skirymvr_mods/source_codes/higgs/src/RE/havok.cpp`
- Read: `E:/fo4dev/skirymvr_mods/source_codes/higgs/analysis/subsystems/12_hand_class.md`

- [ ] Run state inventory:

```powershell
rg -n "\b_state\s*=|HandState::|clearSelectionState|startDynamicPull|updateDynamicPull" ROCK\src\physics-interaction\Hand.cpp ROCK\src\physics-interaction\HandGrab.cpp ROCK\src\physics-interaction\Hand.h
```

Expected: every direct state write is listed before migration begins.

- [ ] Run Havok direct-access inventory:

```powershell
rg -n "GetBodyArray\(|GetMotionArray\(|GetBodyMotion\(|kBody_CollisionObjectBackPointer|kHknpWorld_ModifierManager|kFunc_SetBodyCollisionFilterInfo|kFunc_SetBodyBroadPhaseEnabled|kFunc_SetBodyVelocityDeferred|kData_HavokTlsAllocKey|TlsGetValue|reinterpret_cast<.*0xE0" ROCK\src\physics-interaction
```

Expected: current duplicated access points are visible and can be checked off as they migrate.

- [ ] Run frame-context inventory:

```powershell
rg -n "getPlayerBhkWorld|getHknpWorld|getInteractionHandTransform|getInteractionHandNode|_deltaTime|updateSelection\(|updateGrabInput\(|resolveContacts\(|publishDebugBodyOverlay" ROCK\src\physics-interaction\PhysicsInteraction.cpp ROCK\src\physics-interaction\PhysicsInteraction.h
```

Expected: repeated per-frame world/hand/time lookup sites are visible.

- [ ] Re-read the HIGGS Havok and hand lifecycle reference.

Expected: use only structural patterns: dedicated Havok layer, state lifecycle, enter/exit responsibilities. Do not copy Skyrim offsets, hkp-only layouts, or HIGGS keyframed held object behavior.

### Task 2: Add State Machine Tests First

**Files:**

- Create: `ROCK/tests/HandStateMachineTests.cpp`
- Modify: `ROCK/CMakeLists.txt`

- [ ] Add a focused test executable under `BUILD_ROCK_TESTS` named `ROCKHandStateMachineTests`.

Expected CMake shape:

```cmake
add_executable(ROCKHandStateMachineTests
  "${ROOT_DIR}/tests/HandStateMachineTests.cpp"
)
target_compile_features(ROCKHandStateMachineTests PRIVATE cxx_std_23)
target_include_directories(ROCKHandStateMachineTests PRIVATE ${SOURCE_DIR})
target_link_libraries(ROCKHandStateMachineTests PRIVATE F4VRCommon::framework)
```

- [ ] Write tests for all required first-pass transitions listed in the table above.

Test style should match existing focused tests: `int main()`, local `expectBool`/`expectState` helpers, return `0` only when all checks pass.

- [ ] Include negative tests for illegal transitions:

```cpp
// Examples to encode:
// Idle + BeginPull -> rejected, stays Idle.
// SelectedClose + HeldFadeComplete -> rejected, stays SelectedClose.
// HeldBody + SelectionFoundFar -> rejected, stays HeldBody.
// Pulled + BeginTwoHandHold -> rejected, stays Pulled.
```

- [ ] Include a classification test proving:

```cpp
isHoldingState(HandState::HeldInit) == true
isHoldingState(HandState::HeldBody) == true
isHoldingState(HandState::Idle) == false
hasExclusiveObjectSelection(HandState::SelectionLocked) == true
hasExclusiveObjectSelection(HandState::Pulled) == true
hasExclusiveObjectSelection(HandState::HeldInit) == true
hasExclusiveObjectSelection(HandState::HeldBody) == true
hasExclusiveObjectSelection(HandState::SelectedClose) == false
hasExclusiveObjectSelection(HandState::SelectedFar) == false
```

- [ ] Run the new test target after implementation of Task 3.

Expected: before Task 3 it will not compile because the new state machine header does not exist. After Task 3 it must pass.

### Task 3: Implement Pure HandInteractionStateMachine

**Files:**

- Create: `ROCK/src/physics-interaction/HandInteractionStateMachine.h`
- Create: `ROCK/src/physics-interaction/HandInteractionStateMachine.cpp`
- Modify: `ROCK/src/physics-interaction/SelectionStatePolicy.h`
- Modify: `ROCK/src/physics-interaction/HandState.h`

- [ ] Add the enums and request/result structs from the interface section.
- [ ] Implement `evaluateHandTransition` as a table-like `switch` over event and current state.
- [ ] Implement bit helpers for `HandTransitionEffect` using `std::uint32_t` storage to keep the ABI boring and testable.
- [ ] Move `selection_state_policy` behavior behind state-machine classification functions:

```cpp
inline HandState stateForSelection(bool isFarSelection)
{
    return isFarSelection ? HandState::SelectedFar : HandState::SelectedClose;
}

inline bool canUpdateSelectionFromState(HandState state)
{
    return frik::rock::canUpdateSelectionFromState(state);
}
```

- [ ] Remove `PreGrabItem` from `HandState` if this command returns no references outside the enum:

```powershell
rg -n "PreGrabItem" ROCK\src ROCK\tests
```

Expected if removable: only the enum definition appears before the change.

- [ ] If `SelectedTwoHand` or `HeldTwoHanded` are unused by active code, keep them until the weapon support path is reviewed in the migration task. They are active-domain concepts even if the current enum is not yet wired through the main hand object.
- [ ] Run:

```powershell
cd ROCK
$env:VCPKG_ROOT="C:/vcpkg"
cmake --build build --config Release --target ROCKHandStateMachineTests
```

Expected: state machine tests pass, or the build is blocked by known reload extraction errors outside the test target. If blocked, record the exact non-state-machine blocker in the implementation notes.

### Task 4: Route Hand State Writes Through One Helper

**Files:**

- Modify: `ROCK/src/physics-interaction/Hand.h`
- Modify: `ROCK/src/physics-interaction/Hand.cpp`
- Modify: `ROCK/src/physics-interaction/HandGrab.cpp`

- [ ] Add private `Hand::applyTransition(const HandTransitionRequest& request)` returning `HandTransitionResult`.
- [ ] In `applyTransition`, update `_prevState` and `_state` only when the transition is accepted and the next state differs from the current state.
- [ ] Apply transition effects by calling existing side-effect functions. Do not move side-effect bodies into the pure state machine.
- [ ] Replace direct `_state = HandState::SelectionLocked` with `LockFarSelection`.
- [ ] Replace direct selection state assignment with `SelectionFoundClose` / `SelectionFoundFar`.
- [ ] Replace pull start state assignment with `BeginPull`.
- [ ] Replace pulled-arrival state assignment with `PullArrivedClose`.
- [ ] Replace grab success state assignment with `GrabCommitSucceeded`.
- [ ] Replace held fade completion state assignment with `HeldFadeComplete`.
- [ ] Replace release/cleanup state assignment with `ReleaseRequested`, `ObjectInvalidated`, or `WorldInvalidated` depending on the existing reason.
- [ ] Keep existing logs, but make transition logs consistent:

```cpp
ROCK_LOG_DEBUG(Hand, "{} hand state {} -> {} via {}", handName(), oldName, newName, eventName);
```

- [ ] After migration, run:

```powershell
rg -n "\b_state\s*=" ROCK\src\physics-interaction\Hand.cpp ROCK\src\physics-interaction\HandGrab.cpp ROCK\src\physics-interaction\Hand.h
```

Expected: direct writes only inside constructor/reset initialization and `Hand::applyTransition`.

### Task 5: Add HavokRuntime Tests and Facade Skeleton

**Files:**

- Create: `ROCK/src/physics-interaction/HavokRuntime.h`
- Create: `ROCK/src/physics-interaction/HavokRuntime.cpp`
- Create: `ROCK/tests/HavokRuntimePolicyTests.cpp`
- Modify: `ROCK/CMakeLists.txt`

- [ ] Add a focused test executable under `BUILD_ROCK_TESTS` named `ROCKHavokRuntimePolicyTests`.
- [ ] Write tests for pure policy pieces that do not require a live FO4VR process:

```cpp
isValidBodyId(RE::hknpBodyId{ body_frame::kInvalidBodyId }) == false
isValidBodyId(RE::hknpBodyId{ 42 }) == true
bodySlotCanBeRead through the facade rejects invalid ids
bodySlotCanBeRead through the facade rejects high-water values above body_frame::kMaxReadableBodyIndex
bodySlotCanBeRead through the facade accepts an id equal to a valid high-water mark
```

- [ ] Move body/motion constants used by facade code to one place by reusing `body_frame` constants and `HavokOffsets.h`.
- [ ] Implement facade functions by moving existing logic from `PhysicsUtils.h` and `BodyCollisionControl.cpp`, preserving exact behavior.
- [ ] Do not introduce new offsets.
- [ ] Do not reinterpret raw motion array stride in more than one function. If raw motion array access remains necessary, it must be inside `HavokRuntime.cpp` and use `offsets::kHknpWorld_MotionArrayPtr`.
- [ ] Run:

```powershell
cd ROCK
$env:VCPKG_ROOT="C:/vcpkg"
cmake --build build --config Release --target ROCKHavokRuntimePolicyTests
```

Expected: facade policy tests pass, or the build is blocked by known external compile failures.

### Task 6: Convert Compatibility Wrappers to HavokRuntime

**Files:**

- Modify: `ROCK/src/physics-interaction/PhysicsUtils.h`
- Modify: `ROCK/src/physics-interaction/BodyCollisionControl.h`
- Modify: `ROCK/src/physics-interaction/BodyCollisionControl.cpp`

- [ ] Convert `PhysicsUtils.h` functions that read world/body/motion data into inline calls to `havok_runtime`.
- [ ] Leave scale and transform helpers in `PhysicsUtils.h` only if they are broadly used and not low-level runtime access.
- [ ] Convert `body_collision::tryReadFilterInfo`, `setFilterInfo`, and `setBroadPhaseEnabled` into pass-through calls to `havok_runtime`.
- [ ] Run:

```powershell
rg -n "kFunc_SetBodyCollisionFilterInfo|kFunc_SetBodyBroadPhaseEnabled|kBody_CollisionObjectBackPointer|kHknpWorld_ModifierManager|reinterpret_cast<.*0xE0" ROCK\src\physics-interaction\PhysicsUtils.h ROCK\src\physics-interaction\BodyCollisionControl.cpp
```

Expected: zero direct offset hits in those compatibility files after migration.

### Task 7: Migrate Havok Allocation and Body Operations

**Files:**

- Modify: `ROCK/src/physics-interaction/BethesdaPhysicsBody.cpp`
- Modify: `ROCK/src/physics-interaction/GrabConstraint.cpp`
- Modify: `ROCK/src/physics-interaction/PushAssist.cpp`

- [ ] Replace duplicated `havokAlloc` / `havokFree` in `BethesdaPhysicsBody.cpp` with `havok_runtime::allocateHavok` / `freeHavok`.
- [ ] Replace duplicated `havokHeapAlloc` / heap free logic in `GrabConstraint.cpp` with the same facade calls.
- [ ] Move `HkArrayReserveMore` relocation use behind `havok_runtime::hkArrayReserveMore`.
- [ ] Replace direct `setVelocityDeferred` relocation in `PushAssist.cpp` with `havok_runtime::applyLinearVelocityDeltaDeferred`.
- [ ] Replace direct motion-array inertia normalization/restoration reads in `GrabConstraint.cpp` only after confirming the facade exposes the same saved/restored data. The saved state must still include body id, motion index, packed inverse inertia, and prior motion properties id.
- [ ] Run:

```powershell
rg -n "havokAlloc|havokFree|havokHeapAlloc|TlsGetValue|kData_HavokTlsAllocKey|kFunc_HkArray_ReserveMore|kFunc_SetBodyVelocityDeferred" ROCK\src\physics-interaction\BethesdaPhysicsBody.cpp ROCK\src\physics-interaction\GrabConstraint.cpp ROCK\src\physics-interaction\PushAssist.cpp
```

Expected: zero hits except references to `havok_runtime` function names.

### Task 8: Migrate Body/Motion Call Sites

**Files:**

- Modify: `ROCK/src/physics-interaction/HandGrab.cpp`
- Modify: `ROCK/src/physics-interaction/WeaponCollision.cpp`
- Modify: `ROCK/src/physics-interaction/Hand.cpp`
- Modify: `ROCK/src/physics-interaction/NearbyGrabDamping.cpp`
- Modify: `ROCK/src/physics-interaction/ObjectPhysicsBodySet.cpp`
- Modify: `ROCK/src/physics-interaction/HandBoneColliderSet.cpp`
- Modify: `ROCK/src/physics-interaction/HeldPlayerSpaceRegistry.cpp`
- Modify diagnostics last: `DebugBodyOverlay.cpp`, `PhysicsWorldOriginDiagnostics.cpp`

- [ ] Replace body validity plus `GetBodyArray()` sequences with `havok_runtime::snapshotBody`.
- [ ] Replace live transform resolution with `havok_runtime::resolveLiveBodyWorldTransform`.
- [ ] Replace filter reads/writes with `havok_runtime::tryReadFilterInfo` and `havok_runtime::setFilterInfo`.
- [ ] Replace broadphase toggles with `havok_runtime::setBroadPhaseEnabled`.
- [ ] Preserve diagnostic raw reads only when the diagnostic intentionally compares raw body-array and motion-array interpretations. Mark those as diagnostic exceptions in code comments.
- [ ] Run:

```powershell
rg -n "GetBodyArray\(|GetMotionArray\(|GetBodyMotion\(|kBody_CollisionObjectBackPointer|kHknpWorld_ModifierManager|kFunc_SetBodyCollisionFilterInfo|kFunc_SetBodyBroadPhaseEnabled|kFunc_SetBodyVelocityDeferred|reinterpret_cast<.*0xE0" ROCK\src\physics-interaction
```

Expected: normal runtime code has no direct hits. Remaining hits are limited to `HavokRuntime.cpp`, `HavokOffsets.h`, and diagnostic exceptions.

### Task 9: Add PhysicsFrameContext

**Files:**

- Create: `ROCK/src/physics-interaction/PhysicsFrameContext.h`
- Modify: `ROCK/src/physics-interaction/PhysicsInteraction.h`
- Modify: `ROCK/src/physics-interaction/PhysicsInteraction.cpp`

- [ ] Add `PhysicsFrameContext` and `HandFrameInput` from the interface section.
- [ ] Add `PhysicsInteraction::buildFrameContext()` as a private method.
- [ ] In `buildFrameContext`, resolve:

```cpp
bhkWorld
hknpWorld
deltaSeconds with the existing 1/90 fallback behavior
right and left raw hand transforms
right and left hand nodes
right and left grab anchor / palm normal / pointing vector
menu/world blocked flags already used by update()
```

- [ ] Change `updateSelection`, `updateGrabInput`, `updateHandCollisions`, `publishDebugBodyOverlay`, and `resolveContacts` to accept either `const PhysicsFrameContext&` or the specific sub-struct they need.
- [ ] Preserve existing update ordering while replacing repeated lookups.
- [ ] Keep reload as a boundary. Do not move reload state into `PhysicsFrameContext` beyond a boolean or provider snapshot needed by already active boundaries.
- [ ] Run:

```powershell
rg -n "getPlayerBhkWorld\(|getHknpWorld\(|getInteractionHandTransform\(|getInteractionHandNode\(|_deltaTime" ROCK\src\physics-interaction\PhysicsInteraction.cpp
```

Expected: repeated calls inside per-frame substeps are gone or justified. `buildFrameContext`, initialization, shutdown, and provider snapshot code may still call world accessors.

### Task 10: Remove Dead State and Wrapper Debris

**Files:**

- Modify: state/policy files touched above
- Modify: wrappers touched above
- Modify: `ROCK/docs/2026-05-02-havok-state-machine-refactor-plan.md` only if implementation discoveries require an addendum

- [ ] Remove `PreGrabItem` if Task 3 proved it has no live caller.
- [ ] Remove `SelectionStatePolicy.h` only if all call sites can include `HandInteractionStateMachine.h` without creating circular dependencies. If removing it would force wide churn, leave it as a small compatibility header with no behavior of its own.
- [ ] Remove `BodyCollisionControl.*` only if all collision suppression and body control call sites have moved directly to `havok_runtime`. If removal causes avoidable churn, leave it as a thin compatibility namespace and record it in implementation notes.
- [ ] Remove stale comments that mention old handspace or collision conventions no longer present in code.
- [ ] Run:

```powershell
rg -n "PreGrabItem|legacy|HIGGS XYZ|ROCK Y/Z/X|close-normal Y mirror|fGrabCloseThreshold|fGrabFarThreshold" ROCK\src ROCK\data ROCK\docs
```

Expected: no removed legacy config/convention text remains except deliberate historical notes.

## Verification Plan

Use these commands after each implementation stage where the build state allows it. Do not use git commands.

Focused tests:

```powershell
cd ROCK
$env:VCPKG_ROOT="C:/vcpkg"
cmake --build build --config Release --target ROCKHandStateMachineTests
cmake --build build --config Release --target ROCKHavokRuntimePolicyTests
cmake --build build --config Release --target ROCKBodyFramePolicyTests
cmake --build build --config Release --target ROCKProviderBoundaryTests
```

Full local build after reload extraction is stable:

```powershell
cd ROCK
$env:VCPKG_ROOT="C:/vcpkg"
cmake --build build --config Release
```

Static architecture checks:

```powershell
rg -n "\b_state\s*=" ROCK\src\physics-interaction\Hand.cpp ROCK\src\physics-interaction\HandGrab.cpp ROCK\src\physics-interaction\Hand.h
rg -n "GetBodyArray\(|GetMotionArray\(|GetBodyMotion\(|kBody_CollisionObjectBackPointer|kHknpWorld_ModifierManager|kFunc_SetBodyCollisionFilterInfo|kFunc_SetBodyBroadPhaseEnabled|kFunc_SetBodyVelocityDeferred|kData_HavokTlsAllocKey|TlsGetValue|reinterpret_cast<.*0xE0" ROCK\src\physics-interaction
rg -n "getPlayerBhkWorld\(|getHknpWorld\(|getInteractionHandTransform\(|getInteractionHandNode\(|_deltaTime" ROCK\src\physics-interaction\PhysicsInteraction.cpp
```

Expected final static state:

- `_state =` appears only in construction/reset and `Hand::applyTransition`.
- Direct low-level Havok runtime access appears only in `HavokRuntime.cpp`, `HavokOffsets.h`, shape construction, or marked diagnostics.
- Per-frame world/hand/time resolution is centralized in `PhysicsInteraction::buildFrameContext`.
- Removed legacy handspace/collision config comments do not return.

## Runtime Acceptance Scenarios

Run these in game after the build is stable and deployed:

- Hand colliders spawn, follow both skeleton hands, and still use direct skeleton bone data.
- Close selection still enters `SelectedClose`, curls fingers, and commits a dynamic constrained grab.
- Far selection still enters `SelectedFar`, locks on grab press, pulls, then transitions into close/grab behavior.
- Releasing a held object restores collision suppression, broadphase/filter state, finger pose authority, and provider ownership.
- Two-handed weapon support still works as it did before the refactor; no one-handed mesh wrap behavior returns.
- Weapon collision still follows the generated weapon package and uses the existing perfect collision basis with no rotation correction config.
- Menus, world invalidation, and object invalidation force clean return to idle without stale selection, stale held body ids, or stuck suppression.
- Debug overlays still render body/motion source information, with diagnostic-only raw reads explicitly isolated.

## Non-Goals

- Do not redesign reload internals in this plan.
- Do not reintroduce removed INI options.
- Do not add new grab modes.
- Do not tune physics constants.
- Do not port HIGGS keyframed held-object behavior.
- Do not create new FO4VR offset claims without approval for Ghidra verification.

## Implementation Notes to Keep During Execution

- Keep each task small enough that a failed build identifies one ownership area.
- Preserve logs during migration, then normalize names after behavior is proven.
- When a low-level operation cannot be moved behind `HavokRuntime` without changing behavior, leave a short comment explaining why that specific call site is an exception.
- If another agent changes reload-facing files during this work, do not revert those changes. Adapt only the boundary call signatures needed by this plan.
- Record any new verified runtime finding in the relevant ROCK docs or `libraries_and_tools/havok` notes, but only after the finding has evidence.
