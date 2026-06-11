# HIGGS-Inspired ROCK API Feature Roadmap

Date: 2026-06-11

Project: ROCK

Source used:

- Current local ROCK source in `F:\fo4dev\PROJECT_ROCK_V2\ROCK`
- Existing ROCK API docs and implementation notes
- Local HIGGS API map in `docs/research/2026-06-11-higgs-rock-api-map.md`
- Current hFRIK API context already mirrored in ROCK `src/api/FRIKApi.h`

Verification method: local source and local documentation review only. No web sources, Ghidra MCP, FO4 Mods MCP, deployed files, or runtime logs were used for this planning note.

Confidence:

- High for current API inventory and sequencing rationale.
- Medium for exact implementation costs until each phase maps the current source in detail.
- Low for any engine/layout claim that would require FO4VR binary verification. Those are explicitly marked as gated.

Affected systems:

- `src/api/ROCKProviderApi.h`
- `src/api/ROCKProviderApi.cpp`
- `src/api/ROCKApi.h`
- `src/api/ROCKApi.cpp`
- `src/physics-interaction/core`
- `src/physics-interaction/hand`
- `src/physics-interaction/grab`
- `src/physics-interaction/weapon`
- `src/physics-interaction/object`
- `src/physics-interaction/collision`
- `src/physics-interaction/input`
- `SDK/ROCK`
- `tests`
- Optional later: FO4VR Papyrus integration files

## Goal

Add the HIGGS-inspired API capabilities that would materially benefit ROCK while preserving ROCK's current FO4VR-native architecture:

1. Reason-coded hand/grab state queries.
2. Public two-hand grip state and events.
3. Held-object frame and grabbed-node/transform snapshots.
4. Tokenized interaction command queue.
5. Queued force release.
6. Queued force grab.
7. Owner-tokened weapon collision leases.
8. Narrow Papyrus bridge for quest/ESP integrations.
9. Safe runtime tuning/query API.
10. Declarative collision policy extensions.

The important constraint is that these should not become a HIGGS compatibility layer. They should be ROCK provider API extensions that use owner tokens, bounded queues, lifecycle guards, explicit result codes, and existing ROCK runtime paths.

## Baseline

Current ROCK already has the stronger foundation:

- Provider API v9.
- ROCK-issued owner tokens.
- Capability grants.
- Provider limits and feature bits.
- Frame callbacks.
- Lifecycle fields.
- Hand frames.
- Weapon evidence.
- Body contacts.
- External bodies and owner-filtered contacts.
- Diagnostic overlay and diagnostic input.
- Legacy/simple API v4.

The HIGGS API exposes some useful direct controls that ROCK does not yet expose publicly:

- `GrabObject`.
- `CanGrabObject`.
- `IsHandInGrabbableState`.
- weapon collision enable/disable/force-enable.
- `IsTwoHanding` plus start/stop callbacks.
- `GetGrabTransform` and `SetGrabTransform`.
- `GetGrabbedNodeName`.
- setting get/set.
- Papyrus functions and events.
- raw collision-filter and pre-physics callbacks.

ROCK should take the useful product surface, not the unsafe implementation style. Direct external mutation, raw body pointers, arbitrary hot callbacks, and broad setting writes are not acceptable as default public APIs.

## Operating Rules

Each implementation phase must follow these rules:

- Map the current local source before editing.
- Prefer existing ROCK provider patterns over new global state.
- Append API table entries only. Never insert into the middle of `RockProviderApi`.
- Bump provider API version when adding public table entries.
- Keep provider structs POD, standard-layout, trivially copyable, size-versioned, and alignment asserted.
- Public write/control APIs require ROCK-issued owner tokens.
- Public write/control APIs must be bounded, fail closed, and execute from ROCK-owned safe update points.
- Do not expose live Havok, CommonLib, or engine pointers unless there is a specific trusted-consumer need and a lifetime contract.
- Do not add raw collision-filter callbacks as the first solution.
- Do not add production INI mutation through Papyrus or public C API.
- Preserve compatibility with PAPER and SCISSORS.
- Add focused tests for every public API contract.

## Information Gates

Use these gates before each phase. Do not jump to Ghidra or external references first.

### Gate A: Local ROCK Source

Use for every phase:

- Current API headers and glue.
- Hand state machine.
- Grab lifecycle.
- Weapon collision lifecycle.
- Object claim ownership.
- External body registry.
- Provider frame snapshots.
- Existing tests.
- Existing logs and telemetry if locally available.

### Gate B: Sibling Consumers

Use when changing provider API semantics or owner tokens:

- PAPER provider client usage.
- SCISSORS provider client usage.
- Any SDK example or packaged header copy.

Questions:

- Will this change break a current sibling consumer?
- Does a sibling already need the new capability?
- Can the new field/function remain additive?

### Gate C: hFRIK

Use for hand pose, visual hand authority, skeleton readiness, and hand transform questions:

- `hFRIK/src/api/FRIKApi.h`
- hFRIK implementation where needed.

Questions:

- Is this actually a FRIK-owned pose/visual concern?
- Should ROCK expose only collision/contact facts and leave pose control to hFRIK?

### Gate D: Runtime Logs And Tests

Use when source behavior is ambiguous:

- Local build output.
- Existing policy/source tests.
- Runtime logs from a requested test session.

Questions:

- Does the current event dispatch happen?
- Are state transitions occurring in the expected order?
- Are lifecycle guards blocking writes correctly?

### Gate E: Approved Reverse Engineering

Use only after explaining the need and receiving user approval.

Possible reasons:

- Exact FO4VR binary behavior for Papyrus registration or function availability.
- Exact engine object lifetime that cannot be proven from source.
- Collision filter or physics-step hook semantics that depend on Bethesda or Havok internals.

Do not use Ghidra MCP or FO4 Mods MCP without approval for the current task.

## Roadmap Overview

Recommended order:

1. API state snapshots before commands.
2. Events before external write controls.
3. Release before grab.
4. Leases before collision toggles.
5. Papyrus after C++ provider semantics are stable.
6. Runtime tuning after mutable key safety is known.
7. Collision policy after external body/contact model is extended.

This order builds observability first. Commands and leases become safer when consumers can inspect state and when tests can assert state transitions.

## Phase 0: Baseline Audit And Test Harness

Purpose: establish the exact current public API and runtime surfaces before extending them.

Source to map:

- `src/api/ROCKProviderApi.h`
- `src/api/ROCKProviderApi.cpp`
- `src/api/ROCKApi.h`
- `src/api/ROCKApi.cpp`
- `tests/PublicApiLaunchSourceTests.ps1`
- `tests/FrikApiParitySourceTests.ps1`
- `SDK/ROCK/docs/PublicApi.md`
- `SDK/ROCK/docs/VersionMatrix.md`
- PAPER and SCISSORS provider client code if an API bump affects them.

Work:

- Confirm current provider API version, function order, and implemented feature bits.
- Confirm SDK header copies are byte-for-byte synced.
- Confirm which `GrabEventType` values are dispatched today and which are reserved.
- Confirm current owner-token registration cleanup behavior.
- Add or update source-boundary tests for the current v9 function table before making v10/v11 changes.

Info needed:

- Current dirty branch state.
- Whether PAPER and SCISSORS are expected to be migrated in the same branch or later.
- Whether runtime validation is expected after each phase or only after command phases.

Deliverable:

- Short baseline note or updated tests proving the public API table and docs are aligned.

Validation:

- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%`

Commit class:

- `test/api` if tests only.
- `docs/api` if note only.

## Phase 1: Reason-Coded Hand Interaction State

Purpose: replace guesswork around `CanGrabObject` and `IsHandInGrabbableState` with a ROCK-native read-only state snapshot.

HIGGS inspiration:

- `CanGrabObject(bool isLeft)`
- `IsHandInGrabbableState(bool isLeft)`

ROCK-native goal:

- Public consumers can ask why a hand can or cannot grab without mutating runtime state.
- The data must be a value snapshot, not live pointers with unclear lifetime.

Likely API version:

- `ROCK_PROVIDER_API_VERSION = 10` if this is the first extension.

New enums:

```cpp
enum class RockProviderHandInteractionStateV10 : std::uint32_t
{
    Unknown = 0,
    Idle = 1,
    Touching = 2,
    SelectedClose = 3,
    SelectedFar = 4,
    SelectionLocked = 5,
    Pulling = 6,
    PullArrived = 7,
    Held = 8,
    StashCandidate = 9,
    ConsumeCandidate = 10,
    Disabled = 11,
    ProviderBlocked = 12,
};

enum class RockProviderCanGrabReasonV10 : std::uint32_t
{
    CanGrab = 0,
    ProviderNotReady = 1,
    PhysicsWritesBlocked = 2,
    HandInvalid = 3,
    HandDisabled = 4,
    HandHolding = 5,
    HandPulling = 6,
    SelectionLocked = 7,
    OffhandReserved = 8,
    WeaponBlocking = 9,
    TargetUnavailable = 10,
    TargetPolicyBlocked = 11,
    ObjectAlreadyOwned = 12,
    NoSelection = 13,
};
```

New struct:

```cpp
struct RockProviderHandInteractionFrameV10
{
    std::uint32_t size{ sizeof(RockProviderHandInteractionFrameV10) };
    std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
    std::uint64_t frameIndex{ 0 };
    RockProviderHand hand{ RockProviderHand::None };
    RockProviderHandInteractionStateV10 state{ RockProviderHandInteractionStateV10::Unknown };
    RockProviderCanGrabReasonV10 canGrabReason{ RockProviderCanGrabReasonV10::ProviderNotReady };
    std::uint32_t flags{ 0 };
    std::uintptr_t heldRefr{ 0 };
    std::uint32_t heldFormId{ 0 };
    std::uint32_t heldPrimaryBodyId{ 0x7FFF'FFFF };
    std::uintptr_t selectedRefr{ 0 };
    std::uint32_t selectedFormId{ 0 };
    std::uint32_t selectedBodyId{ 0x7FFF'FFFF };
    std::uint32_t worldGeneration{ 0 };
    std::uint32_t skeletonGeneration{ 0 };
    std::uint32_t providerGeneration{ 0 };
    std::uint32_t reserved[8]{};
};
```

New provider function:

```cpp
bool(ROCK_PROVIDER_CALL* getHandInteractionFrameV10)(
    RockProviderHand hand,
    RockProviderHandInteractionFrameV10* outFrame);
```

Source to map first:

- `src/physics-interaction/hand/HandLifecycle.h`
- `src/physics-interaction/hand/HandInteractionStateMachine.h`
- `src/physics-interaction/hand/Hand.h`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
- provider frame snapshot code.

Implementation notes:

- Convert internal hand state to public state through a narrow helper.
- Derive `canGrabReason` from existing state and provider lifecycle flags.
- Do not call expensive selection scans from the query.
- Do not allocate or format strings.
- Do not expose full internal enum values if they are unstable.

Acceptance criteria:

- Consumers can tell idle, selected, pulling, held, disabled, stash candidate, and consume candidate states.
- Consumers can distinguish "hand busy" from "provider not ready".
- Query is read-only and returns false on invalid struct size or invalid hand.

Validation:

- Static/source test for appended function order.
- Policy test for public enum mapping from representative internal states.
- Source-boundary test that query does not enqueue or mutate command state.

Dependencies:

- None beyond current v9 provider API.

## Phase 2: Public Two-Hand Grip State And Events

Purpose: expose real two-hand state instead of leaving `TwoHandStarted` and `TwoHandStopped` as reserved ABI values.

HIGGS inspiration:

- `IsTwoHanding()`
- `AddStartTwoHandingCallback`
- `AddStopTwoHandingCallback`
- Papyrus `OnStartTwoHanding`
- Papyrus `OnStopTwoHanding`

ROCK-native goal:

- Expose more than a bool. Consumers need weapon identity, hand roles, body ids, and semantic grip data.

New enum:

```cpp
enum class RockProviderTwoHandGripKindV10 : std::uint32_t
{
    None = 0,
    WeaponSupport = 1,
    HeldObjectShared = 2,
    ExternalReserved = 3,
};
```

New struct:

```cpp
struct RockProviderTwoHandGripFrameV10
{
    std::uint32_t size{ sizeof(RockProviderTwoHandGripFrameV10) };
    std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
    std::uint64_t frameIndex{ 0 };
    RockProviderTwoHandGripKindV10 kind{ RockProviderTwoHandGripKindV10::None };
    std::uint32_t active{ 0 };
    RockProviderHand primaryHand{ RockProviderHand::None };
    RockProviderHand supportHand{ RockProviderHand::None };
    std::uintptr_t targetRefr{ 0 };
    std::uint32_t targetFormId{ 0 };
    std::uint32_t primaryBodyId{ 0x7FFF'FFFF };
    std::uint32_t supportBodyId{ 0x7FFF'FFFF };
    std::uint32_t weaponPartKind{ 0 };
    std::uint32_t supportRole{ 0 };
    std::uint64_t generationKey{ 0 };
    std::uint32_t worldGeneration{ 0 };
    std::uint32_t skeletonGeneration{ 0 };
    std::uint32_t providerGeneration{ 0 };
    std::uint32_t reserved[8]{};
};
```

New provider function:

```cpp
bool(ROCK_PROVIDER_CALL* getTwoHandGripFrameV10)(
    RockProviderTwoHandGripFrameV10* outFrame);
```

Event work:

- Dispatch `GrabEventType::TwoHandStarted` when ROCK enters a public two-hand state.
- Dispatch `GrabEventType::TwoHandStopped` when it exits.
- Set source kind to `Weapon` for weapon support and `HeldObject` for shared held-object grabs.
- Include body ids and form id where available.

Source to map first:

- `src/physics-interaction/weapon/TwoHandedGrip.h`
- `src/physics-interaction/weapon/TwoHandedGrip.cpp`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
- `src/physics-interaction/hand/HandGrab.cpp`
- tests around weapon support and shared held object grabs.

Info needed:

- Exact current ownership of weapon support grip state.
- Whether shared held-object two-hand state is represented centrally or only per hand.
- Which transitions are stable enough to fire events exactly once.

Acceptance criteria:

- Public query returns inactive state with valid generation fields when ready but not two-handing.
- Start/stop events fire once per transition.
- Events are not emitted every frame.
- Existing grab/release events remain unchanged.

Validation:

- Policy/source tests for event dispatch symbols.
- Runtime test with weapon support grip.
- Runtime test with shared held object if currently supported.

## Phase 3: Held-Object Frame And Grabbed Node Snapshot

Purpose: give consumers read-only access to held-object identity, body set, node name, and transform data.

HIGGS inspiration:

- `GetGrabbedObject`
- `GetGrabbedRigidBody`
- `GetGrabbedNodeName`
- `GetGrabTransform`
- `SetGrabTransform`

ROCK-native goal:

- Provide read-only structured snapshots first.
- Avoid public `SetGrabTransform` until command/lease ownership exists.
- Avoid live rigid body pointers.

New constants:

```cpp
inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HELD_BODIES_V10 = 16;
inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HELD_NODE_NAME_V10 = 64;
```

New struct:

```cpp
struct RockProviderHeldObjectFrameV10
{
    std::uint32_t size{ sizeof(RockProviderHeldObjectFrameV10) };
    std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
    std::uint64_t frameIndex{ 0 };
    RockProviderHand hand{ RockProviderHand::None };
    std::uint32_t held{ 0 };
    std::uintptr_t refr{ 0 };
    std::uint32_t formId{ 0 };
    std::uint32_t primaryBodyId{ 0x7FFF'FFFF };
    std::uint32_t bodyCount{ 0 };
    std::uint32_t bodyIds[ROCK_PROVIDER_MAX_HELD_BODIES_V10]{};
    RockProviderTransform handToHeldTransform{};
    RockProviderTransform heldBodyWorld{};
    RockProviderTransform heldNodeWorld{};
    float mass{ 0.0f };
    std::uint32_t motionMode{ 0 };
    std::uint32_t flags{ 0 };
    char grabbedNodeName[ROCK_PROVIDER_MAX_HELD_NODE_NAME_V10]{};
    std::uint32_t worldGeneration{ 0 };
    std::uint32_t skeletonGeneration{ 0 };
    std::uint32_t providerGeneration{ 0 };
    std::uint32_t reserved[8]{};
};
```

New provider function:

```cpp
bool(ROCK_PROVIDER_CALL* getHeldObjectFrameV10)(
    RockProviderHand hand,
    RockProviderHeldObjectFrameV10* outFrame);
```

Source to map first:

- `src/physics-interaction/hand/Hand.h`
- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/grab/GrabCore.h`
- `src/physics-interaction/grab/GrabHeldObject.h`
- grab transform telemetry in debug overlay.

Info needed:

- Current authoritative hand-to-held transform type and lifetime.
- Where grabbed node name is stored, if currently retained.
- Whether node name can be copied safely into a fixed char buffer.
- Which body set is the stable public one: primary only, connected set, or held drive body set.

Acceptance criteria:

- Query returns valid "not held" frame without exposing stale object data.
- Held frame is fixed-size and contains no live `std::string`, vectors, or private types.
- Node name copy is bounded and null-terminated.
- Body ids are copied primary-first and capped.

Validation:

- Struct size/static layout tests.
- Unit/policy test for bounded body-id and node-name copy.
- Runtime test while holding a loose object.

Deferred:

- Public transform write/control.
- External release velocity injection.
- Live body pointer access.

## Phase 4: Interaction Command Queue Foundation

Purpose: create the internal bounded queue and result model before exposing force grab or release.

HIGGS inspiration:

- HIGGS direct `GrabObject`.

ROCK-native goal:

- All public interaction writes become queued commands executed from ROCK's update path.

New internal owner:

- `src/physics-interaction/api/InteractionCommandQueue.h`
- `src/physics-interaction/api/InteractionCommandQueue.cpp`

Possible public enums:

```cpp
enum class RockProviderInteractionCommandKindV11 : std::uint32_t
{
    Unknown = 0,
    ForceGrab = 1,
    ForceRelease = 2,
};

enum class RockProviderInteractionCommandStateV11 : std::uint32_t
{
    Unknown = 0,
    Queued = 1,
    Executing = 2,
    Succeeded = 3,
    Rejected = 4,
    Cancelled = 5,
    Expired = 6,
};

enum class RockProviderInteractionFailureV11 : std::uint32_t
{
    None = 0,
    ProviderNotReady = 1,
    PhysicsWritesBlocked = 2,
    OwnerNotRegistered = 3,
    OwnerLacksCapability = 4,
    InvalidRequest = 5,
    StaleWorldGeneration = 6,
    StaleSkeletonGeneration = 7,
    StaleProviderGeneration = 8,
    TargetMissing = 9,
    TargetUnavailable = 10,
    TargetBodyMissing = 11,
    TargetPolicyBlocked = 12,
    TargetAlreadyOwned = 13,
    HandInvalid = 14,
    HandDisabled = 15,
    HandBusy = 16,
    HandNotHolding = 17,
    HeldObjectMismatch = 18,
    QueueFull = 19,
    Timeout = 20,
};
```

Result struct:

```cpp
struct RockProviderInteractionCommandResultV11
{
    std::uint32_t size{ sizeof(RockProviderInteractionCommandResultV11) };
    std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
    std::uint64_t ownerToken{ 0 };
    std::uint64_t commandId{ 0 };
    RockProviderInteractionCommandKindV11 kind{ RockProviderInteractionCommandKindV11::Unknown };
    RockProviderInteractionCommandStateV11 state{ RockProviderInteractionCommandStateV11::Unknown };
    RockProviderInteractionFailureV11 failure{ RockProviderInteractionFailureV11::None };
    RockProviderHand hand{ RockProviderHand::None };
    std::uintptr_t targetRefr{ 0 };
    std::uint32_t targetFormId{ 0 };
    std::uint32_t targetBodyId{ 0x7FFF'FFFF };
    std::uint64_t frameIndex{ 0 };
    std::uint32_t worldGeneration{ 0 };
    std::uint32_t skeletonGeneration{ 0 };
    std::uint32_t providerGeneration{ 0 };
    std::uint32_t reserved[8]{};
};
```

New provider function:

```cpp
RockProviderResultV9(ROCK_PROVIDER_CALL* getInteractionCommandResultV11)(
    std::uint64_t ownerToken,
    std::uint64_t commandId,
    RockProviderInteractionCommandResultV11* outResult);
```

Source to map first:

- provider consumer registry implementation.
- existing frame callback fault handling.
- `PhysicsInteraction::update`.
- lifecycle gate logic.

Implementation notes:

- Queue capacity should be explicit in provider limits.
- Completed result ring capacity should be explicit or documented.
- Commands should be removed on owner unregister.
- Commands should be cancelled on provider loss, world change, or skeleton destroying.
- Immediate API glue validates only shape, owner, capability, and queue capacity.
- Runtime execution validates all world/hand/object conditions again.

Acceptance criteria:

- Queue exists but no public force grab/release is active yet.
- Owner cleanup works.
- Result polling works for rejected/cancelled synthetic commands in tests.
- Provider limits report command queue capacity only when commands are actually supported.

Validation:

- Unit tests for queue capacity.
- Unit tests for owner cleanup.
- Source-boundary test proving provider glue does not call hand mutation directly.

## Phase 5: Queued Force Release

Purpose: implement the lower-risk command first, replacing legacy global `forceDropObject` for new consumers.

HIGGS inspiration:

- HIGGS does not have a direct drop command, but its drop callbacks and Papyrus events make release observable.
- ROCK already has legacy `forceDropObject`.

New public request:

```cpp
enum class RockProviderForceReleaseFlagsV11 : std::uint32_t
{
    None = 0,
    SuppressHaptics = 1u << 0,
    RequireMatchingTarget = 1u << 1,
    ClearSelectionOnly = 1u << 2,
};

struct RockProviderForceReleaseRequestV11
{
    std::uint32_t size{ sizeof(RockProviderForceReleaseRequestV11) };
    std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
    std::uint64_t commandId{ 0 };
    RockProviderHand hand{ RockProviderHand::None };
    std::uint32_t flags{ 0 };
    std::uintptr_t targetRefr{ 0 };
    std::uint32_t targetFormId{ 0 };
    std::uint32_t targetBodyId{ 0x7FFF'FFFF };
    std::uint32_t worldGeneration{ 0 };
    std::uint32_t skeletonGeneration{ 0 };
    std::uint32_t providerGeneration{ 0 };
    std::uint32_t reserved[8]{};
};
```

New provider function:

```cpp
RockProviderResultV9(ROCK_PROVIDER_CALL* requestForceReleaseV11)(
    std::uint64_t ownerToken,
    const RockProviderForceReleaseRequestV11* request);
```

Source to map first:

- `PhysicsInteraction::forceDropHeldObject`
- `PhysicsInteraction::makeGrabReleaseContext`
- `Hand::releaseGrabbedObject`
- claim release logic.
- release event dispatch.
- stash/consume transfer release behavior.

Info needed:

- Whether `forceDropHeldObject` already uses the full safe release path.
- Whether `SuppressHaptics` can be wired safely without changing unrelated event semantics.
- How to clear selection/pull state without releasing a held object.

Acceptance criteria:

- Request is queued and executed from ROCK update.
- Explicit `Left` or `Right` release works.
- `hand = None` can be deferred unless a safe target lookup exists.
- `RequireMatchingTarget` rejects mismatched held objects.
- Normal release events still dispatch on success.
- Legacy `ROCKApi::forceDropObject` remains unchanged but docs point new consumers to provider command.

Validation:

- Policy tests for request validation and result codes.
- Source-boundary test that provider API glue only enqueues.
- Runtime test: normal grab, queued release, release event, collision cleanup.

## Phase 6: Queued Force Grab

Purpose: provide the highest-value HIGGS-inspired command, implemented through ROCK's existing grab ownership paths.

HIGGS inspiration:

- `GrabObject(TESObjectREFR* object, bool isLeft)`.

ROCK-native goal:

- Force grab is a request, not direct mutation.
- It must use the same object selection, claim, collision, constraint, haptic, and cleanup invariants as normal grabs.

New public request:

```cpp
enum class RockProviderForceGrabFlagsV11 : std::uint32_t
{
    None = 0,
    AllowFarGrab = 1u << 0,
    RequireReachable = 1u << 1,
    DropExistingHeldObject = 1u << 2,
    SuppressHaptics = 1u << 3,
    UseCurrentSelectionIfMatching = 1u << 4,
    RequireLooseObjectPolicy = 1u << 5,
};

struct RockProviderForceGrabRequestV11
{
    std::uint32_t size{ sizeof(RockProviderForceGrabRequestV11) };
    std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
    std::uint64_t commandId{ 0 };
    RockProviderHand hand{ RockProviderHand::None };
    std::uint32_t flags{ 0 };
    std::uintptr_t targetRefr{ 0 };
    std::uint32_t targetFormId{ 0 };
    std::uint32_t targetBodyId{ 0x7FFF'FFFF };
    std::uint32_t worldGeneration{ 0 };
    std::uint32_t skeletonGeneration{ 0 };
    std::uint32_t providerGeneration{ 0 };
    float maxDistanceGame{ 0.0f };
    float preferredGrabPointGame[3]{};
    std::uint32_t reserved[8]{};
};
```

New provider function:

```cpp
RockProviderResultV9(ROCK_PROVIDER_CALL* requestForceGrabV11)(
    std::uint64_t ownerToken,
    const RockProviderForceGrabRequestV11* request);
```

Source to map first:

- close selection.
- far selection.
- mesh/skinned safety policy.
- loose pickup layer policy.
- object claim ownership.
- `Hand::grabSelectedObject` or equivalent current commit path.
- dynamic pull/catch path.
- grab event dispatch.
- hand collision lease and held body flag lease code.

Info needed:

- The narrowest existing helper that can commit a selected object into held state.
- Whether requested target can safely become a temporary selection.
- Whether target body id can be trusted only after revalidation.
- Whether far force grab should use pull/catch or immediate hold.

Recommended first version:

- Require explicit `Left` or `Right`.
- Require a live `targetRefr`.
- Use current selection if it matches and is valid.
- Otherwise prepare a temporary internal selection through existing target-acquisition logic.
- Reject if a reusable target-preparation path does not exist yet.
- Do not implement arbitrary no-selection direct constraint creation.

Acceptance criteria:

- Valid loose object can be queued and grabbed.
- Busy hand rejects unless `DropExistingHeldObject` is set.
- Already owned target rejects.
- Stale generation rejects.
- Policy-blocked targets reject.
- Normal grab events dispatch on success.

Validation:

- Policy tests for request validation.
- Source-boundary test that no second direct grab constraint path is introduced.
- Runtime tests for valid grab, busy reject, busy drop-then-grab, stale generation reject, and policy-blocked reject.

Deferred:

- Any-hand arbitration.
- External transfer of ownership between consumers.
- Release velocity injection.
- Forced actor grabs.

## Phase 7: Weapon Collision Leases

Purpose: expose HIGGS-like weapon collision control safely for PAPER, weapon mods, and diagnostics.

HIGGS inspiration:

- `DisableWeaponCollision`
- `EnableWeaponCollision`
- `IsWeaponCollisionDisabled`
- `ForceWeaponCollisionEnabled`

ROCK-native goal:

- Replace global toggles with owner-tokened leases.
- Preserve weapon collider lifecycle and restore behavior.

New concepts:

- Lease kind: suppress, force enable, reserve reload interaction, reserve support interaction.
- Owner token.
- Priority.
- Optional expiry frame.
- Reason enum.
- Current winner query.

Possible request:

```cpp
enum class RockProviderWeaponCollisionLeaseKindV12 : std::uint32_t
{
    None = 0,
    Suppress = 1,
    ForceEnable = 2,
    ReserveReload = 3,
    ReserveSupport = 4,
};

struct RockProviderWeaponCollisionLeaseRequestV12
{
    std::uint32_t size{ sizeof(RockProviderWeaponCollisionLeaseRequestV12) };
    std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
    RockProviderHand hand{ RockProviderHand::None };
    RockProviderWeaponCollisionLeaseKindV12 kind{ RockProviderWeaponCollisionLeaseKindV12::None };
    std::uint32_t priority{ 0 };
    std::uint32_t durationFrames{ 0 };
    std::uint32_t reason{ 0 };
    std::uint32_t reserved[8]{};
};
```

Source to map first:

- `src/physics-interaction/weapon/WeaponCollision.h`
- `src/physics-interaction/weapon/WeaponCollision.cpp`
- dominant weapon collision suppression.
- left weapon support collision suppression.
- offhand reservation logic.
- PAPER reload usage.

Info needed:

- Current collision layer/body restore path.
- Which hand owns weapon collision in left-handed mode.
- Which PAPER states need collision suppressed or forced.
- Whether a lease can be evaluated once per frame without hot-path churn.

Acceptance criteria:

- A consumer can request and release a lease.
- Lease auto-clears on unregister/provider loss.
- Highest-priority active lease wins.
- Query exposes current lease state.
- Existing internal weapon collision restore is not bypassed.

Validation:

- Unit tests for priority, expiry, owner cleanup.
- Runtime test with equipped weapon collision suppression and restore.
- PAPER smoke test if PAPER consumes it.

## Phase 8: Papyrus Bridge

Purpose: give quest/ESP integrations a narrow, safe ROCK-facing surface.

HIGGS inspiration:

- `HiggsVR` Papyrus native functions.
- object pulled/grabbed/dropped/stashed/consumed events.
- two-hand start/stop events.

ROCK-native goal:

- Expose read-only state and events first.
- Route writes through provider commands only after command queue exists.

Candidate Papyrus class:

- `ROCKVR`

Initial functions:

- `GetHeldObject(bool isLeft) -> ObjectReference`
- `GetSelectedObject(bool isLeft) -> ObjectReference`
- `IsHandHolding(bool isLeft) -> bool`
- `IsHandDisabled(bool isLeft) -> bool`
- `IsTwoHanding() -> bool`
- `GetLastTouchedObject(bool isLeft) -> ObjectReference`
- `RegisterForGrabEvent(Form receiver)`
- `UnregisterForGrabEvent(Form receiver)`
- `RegisterForReleaseEvent(Form receiver)`
- `UnregisterForReleaseEvent(Form receiver)`
- `RegisterForStashEvent(Form receiver)`
- `UnregisterForStashEvent(Form receiver)`
- `RegisterForConsumeEvent(Form receiver)`
- `UnregisterForConsumeEvent(Form receiver)`
- later: `RequestForceRelease(...)`
- later: `RequestForceGrab(...)`

Papyrus event candidates:

- `OnROCKGrabbed(ObjectReference akRef, bool abLeft)`
- `OnROCKReleased(ObjectReference akRef, bool abLeft)`
- `OnROCKStashed(Form akBaseForm, bool abLeft)`
- `OnROCKConsumed(Form akBaseForm, bool abLeft)`
- `OnROCKTwoHandStarted()`
- `OnROCKTwoHandStopped()`

Source to map first:

- CommonLibF4VR Papyrus registration patterns in local projects.
- Existing ROCK F4SE message dispatch.
- HIGGS Papyrus event registration pattern as conceptual reference only.
- FO4VR-specific Papyrus event APIs in local libraries.

Info needed:

- Exact FO4VR/CommonLibF4VR Papyrus registration API.
- Whether scripts/assets are needed in release package.
- Whether quest consumers actually need write commands in Papyrus.

Possible gated need:

- If local headers or examples are insufficient for FO4VR Papyrus registration, request approval before using Ghidra or external references.

Acceptance criteria:

- Papyrus layer cannot mutate hand/grab state directly.
- Event registration is bounded and unregisterable.
- Events are queued from safe runtime event paths.
- No per-frame Papyrus calls.

Validation:

- Build.
- Source-boundary test for no direct Papyrus force-grab mutation.
- Runtime test with a small local script only if requested.

## Phase 9: Runtime Tuning And Config Query API

Purpose: expose useful configuration status without allowing arbitrary production INI mutation.

HIGGS inspiration:

- `GetSettingDouble`
- `SetSettingDouble`
- Papyrus `GetSetting`
- Papyrus `SetSetting`

ROCK-native goal:

- Typed query for current runtime config.
- Optional whitelist of mutable hot-safe settings later.

Plan:

1. Add read-only config snapshot first.
2. Add typed setting IDs, not string names, for public use.
3. Add mutable tuning only for keys proven hot-reload safe.
4. Keep production INI writes out of runtime API.

Possible enum:

```cpp
enum class RockProviderRuntimeSettingIdV13 : std::uint32_t
{
    Unknown = 0,
    GrabEnabled = 1,
    HapticsEnabled = 2,
    DebugOverlayEnabled = 3,
};
```

Source to map first:

- `src/RockConfig.h`
- `src/RockConfig.cpp`
- runtime reads of config values.
- tests for config policy.

Info needed:

- Which values are read every frame.
- Which values are read only at startup or rebuild.
- Which values would require collider rebuilds or provider reset.

Acceptance criteria:

- Read-only query cannot fail by mutating state.
- Mutable API, if added, validates type, range, and hot-safety.
- Docs clearly distinguish runtime tuning from INI persistence.

Validation:

- Unit tests for setting ID mapping and bounds.
- Source-boundary tests that production INI is not modified by public API.

## Phase 10: Declarative Collision Policy Extensions

Purpose: capture useful parts of HIGGS collision-filter callbacks without arbitrary external code in hot collision paths.

HIGGS inspiration:

- `AddCollisionFilterComparisonCallback`.

ROCK-native goal:

- Consumers declare collision/contact policy for their external bodies.
- ROCK evaluates it internally using value data.

Possible extension:

- Add flags to external body registrations.
- Add owner-tokened collision policy records.
- Support target-kind filters and role filters.
- Support suppress dynamic push, report contacts, report all source kinds, allow/suppress specific role pairs.

Source to map first:

- `src/physics-interaction/object/ExternalBodyRegistry.h`
- `src/physics-interaction/object/ExternalBodyRegistry.cpp`
- contact handling in `PhysicsInteractionContacts.inl`
- dynamic push assist.
- collision suppression registry.

Info needed:

- Which collision decisions are currently available at external body registration time.
- Which are only known in hot contact callbacks.
- Whether any desired policy requires actual collision-filter hook changes.

Possible gated need:

- If we need exact collision-filter behavior or Bethesda/Havok callback ordering, request approval before Ghidra.

Acceptance criteria:

- Policies are declarative and bounded.
- No external callback is invoked from a collision-filter hot path.
- External contacts still report enough data for consumers.
- Existing PAPER/SCISSORS external body behavior remains compatible.

Validation:

- Unit tests for policy matching.
- Source-boundary tests for no arbitrary callback path.
- Runtime test with external body policy suppressing ROCK dynamic push.

## Cross-Phase API Version Strategy

Recommended version bumps:

- v10: read-only observability extensions:
  - hand interaction frame.
  - two-hand frame.
  - held-object frame.
- v11: interaction command queue:
  - command result polling.
  - request force release.
  - request force grab.
- v12: weapon collision leases.
- v13: runtime config query/tuning.
- v14: declarative collision policy extension, if needed.

If we want fewer version bumps, combine v10 and v11, but keep implementation commits separate.

Rules:

- Update `SDK/ROCK/include/ROCKProviderApi.h` with source header.
- Update `SDK/ROCK/docs/VersionMatrix.md`.
- Update `SDK/ROCK/docs/PublicApi.md`.
- Update `tests/PublicApiLaunchSourceTests.ps1` expected function order.
- Add static asserts for all new structs.

## Validation Matrix

For read-only API phases:

- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%`
- Fast build only if requested or if implementation touches compiled code and local workflow expects it.

For command/lease phases:

- Full policy/source tests.
- Fast Release build with custom-fast when build/deploy is requested or required by task.
- Runtime smoke tests after deploy if user requests runtime validation.

For Papyrus phase:

- Build.
- Packaging check for scripts if any are added.
- Runtime script smoke test if requested.

For provider changes affecting siblings:

- Build ROCK.
- Build PAPER if PAPER consumes new API.
- Build SCISSORS if SCISSORS consumes new API.

## Rollback Strategy

Each phase should be a focused commit:

- `docs/api`: planning and docs.
- `test/api`: source-boundary or policy tests.
- `feature/api`: public read-only API extensions.
- `feature/interaction`: command queue and force release/grab.
- `feature/weapon`: weapon collision leases.
- `feature/papyrus`: Papyrus bridge.
- `feature/collision`: declarative collision policy.

Do not keep hidden fallback implementations. Git commits are rollback.

Acceptable compatibility:

- Keep old public APIs while new provider APIs launch.
- Keep inert legacy ABI slots where already required.
- Keep existing sibling token paths only until PAPER and SCISSORS migration is explicitly completed.

Not acceptable:

- A second direct grab implementation for external commands.
- A raw public `ForceGrabNow` that mutates hand state immediately.
- A raw collision-filter external callback in hot paths.
- Arbitrary string setting mutation against production config.
- Public live body pointers without a lifetime contract.

## Decision Points Before Coding

Answer these before starting Phase 1:

- Should v10 combine hand interaction, two-hand, and held-object frames, or should each get a separate version bump?
- Do PAPER and SCISSORS need to consume any new read-only frames immediately?
- Should public docs continue to expose legacy `ROCKApi`, or move it to a compatibility appendix?

Answer these before Phase 5:

- Should `hand = None` be allowed for force release in first release?
- Should `SuppressHaptics` be supported immediately or deferred?
- Should force release support clear-selection-only in the first release?

Answer these before Phase 6:

- Should force grab require a live `TESObjectREFR*` in v11?
- Should force grab allow far-grab behavior in v11?
- Should force grab be allowed while another consumer owns the target?
- Should force grab with `DropExistingHeldObject` perform release and grab in the same frame or across two frames?

Answer these before Phase 8:

- Is Papyrus needed for actual quest/ESP consumers now, or should it wait until command APIs are stable?
- What script/class name should be public?
- Should Papyrus write commands exist at all in first release?

## Recommended Initial Decisions

Use these unless a concrete consumer need says otherwise:

- Put read-only frame additions in v10.
- Put command queue, force release, and force grab in v11.
- Require explicit `Left` or `Right` for first force grab.
- Defer `hand = None` arbitration.
- Defer release velocity injection.
- Defer public transform writes.
- Defer arbitrary config mutation.
- Defer raw collision-filter callbacks indefinitely.
- Use Papyrus read-only/events first.
- Let command polling ship before command callbacks.

## Near-Term Work Packages

### Package A: Observability

Includes:

- Phase 1 hand interaction frame.
- Phase 2 two-hand frame/events.
- Phase 3 held-object frame.

Why first:

- Lowers risk for future commands.
- Makes debugging and sibling integration easier.
- Adds value without external mutation.

### Package B: Interaction Commands

Includes:

- Phase 4 command queue.
- Phase 5 force release.
- Phase 6 force grab.

Why second:

- Requires observability and result codes.
- Highest runtime risk.
- Should be tested in isolation.

### Package C: Coordination APIs

Includes:

- Phase 7 weapon collision leases.
- Phase 10 declarative collision policies.

Why third:

- Needs clear ownership and lease patterns from command work.
- Affects cross-plugin interactions.

### Package D: Non-F4SE Consumers

Includes:

- Phase 8 Papyrus.
- Phase 9 runtime tuning/config query.

Why later:

- Better once provider command semantics are stable.
- Avoids creating a separate control model for Papyrus.

## First Concrete Task

Start with Phase 1:

1. Map internal hand states and public states.
2. Add `RockProviderHandInteractionFrameV10`.
3. Add `getHandInteractionFrameV10`.
4. Add enum/static assert tests.
5. Update SDK docs and version matrix.
6. Build tests and run source-boundary tests.

This gives us the first useful HIGGS-inspired capability, creates the public state vocabulary needed by later commands, and avoids the highest-risk mutation work until we have stronger observability.
