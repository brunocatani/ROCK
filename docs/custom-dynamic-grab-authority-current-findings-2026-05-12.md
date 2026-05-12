# Custom Dynamic Grab Authority - Current Findings

Date: 2026-05-12

This file supersedes `docs/custom-dynamic-grab-authority-tracker-2026-05-12.md` where the two conflict. The older tracker remains useful as historical context, but this file records the newer Ghidra/source audit findings from the current pass.

This is research only. No implementation has been made from these findings.

## Current Framing

HIGGS is the behavioral reference for polished dynamic grab feel, not a 1:1 implementation target. Fallout 4 VR is on a newer hknp/Havok stack than Skyrim VR, so the correct ROCK design should use FO4VR-native functions, wrappers, step phases, and constraint/action mechanisms where they give better control or stability.

The target remains ordinary dynamic one-hand grab for loose objects and loose non-equipped weapons. Equipped weapons, two-hand equipped weapon behavior, actor ragdoll grab, and HIGGS keyframed grab are outside this pass unless a shared infrastructure function must be understood.

Important FO4VR binary caveat:

- Many `hkp*` classes and helpers are still present in the FO4VR binary.
- Presence of an `hkp*` symbol is not proof that the live FO4VR runtime path uses it correctly or at all.
- `hkp*` findings are useful for atom layouts, vtable intent, motor field layouts, and compatibility evidence.
- A claim becomes production-relevant only after a live `hknp` world/constraint/action insertion or update path is traced to that code or to an equivalent wrapper.
- For this research, every `hkp*` finding must therefore be tagged as one of:
  - live `hknp` path confirmed;
  - layout witness only;
  - unresolved / possibly dead compatibility code.

## Locked Behavioral Rules

- COM is never grip pivot authority.
- COM is allowed only for mass, inertia, lever length, release swing, and weight effects.
- Grip authority must come from the captured contact/mesh/authored grip point and hand/palm reference frame.
- The current issue is not "the mouse spring cannot grab"; the current one-hand mouse-spring path mostly works but feels too strong, especially with heavy guns/objects.
- The desired next architecture is still being researched: custom authority should reproduce finite-force, mass, angular, collision, and deviation behavior without reintroducing jitter.
- One held body must have one drive authority. Mouse spring plus motors fighting the same body is not acceptable.

## Verified FO4VR Constraint Creation Path

### `0x1415469B0` - hknpWorld constraint creation path

Ghidra decompile shows this path:

- allocates/reserves a constraint id from the world constraint manager;
- creates/updates the internal constraint slot at world constraint array stride `0x38`;
- stores the incoming constraint data pointer;
- calls the constraint data vtable at:
  - `+0x20` for constraint type;
  - `+0x28` for constraint info / atom info;
  - `+0x90` for runtime info;
- allocates runtime memory based on returned runtime info.

Implication:

- ROCK's custom vtable slot choices for `getConstraintInfo` and `getRuntimeInfo` are structurally correct.
- The main danger is not the vtable slot mapping. The main danger is the runtime info values and atom runtime offsets returned/written by ROCK.

### `0x1417E39C0` - internal constraint slot initialization

This helper is called by `0x1415469B0`.

Confirmed behavior:

- refcounts/stores the constraint data pointer;
- calls vtable `+0x28` to fill atom/constraint info;
- calls vtable `+0x90` with a boolean-like want-runtime argument;
- allocates runtime memory using returned runtime size;
- stores runtime size/solver metadata into the internal constraint slot.

Implication:

- Custom constraint runtime layout errors are likely to manifest immediately as solver/runtime corruption, jitter, or unstable motor history.

## Verified FO4VR Native Constraint Runtime Witnesses

These were checked as independent witnesses, not from existing markdown.

### `0x1419B1350` - native prismatic constraint constructor

Relevant writes:

- linear motor atom is at native prismatic data offset around `+0xB0`;
- linear motor atom type is `0x0B`;
- packed write sets the linear motor initialized offset to `0x50`;
- packed write sets the previous target position offset to `0x54`.

### `0x1419B1B20` - native prismatic runtime info

Returns:

- runtime size `0x58`;
- solver results `10`.

Interpretation:

- `10 * sizeof(hkpSolverResults)` is `10 * 8 = 0x50`.
- The prismatic linear initialized byte at `0x50` and previous target float at `0x54` sit immediately after the solver result block.
- This strongly supports absolute runtime offsets from the runtime base, not per-atom-relative offsets.

### `0x1419ACA30` - native limited hinge constructor

Relevant writes:

- motor-related offsets include `0x50` and `0x54`.

### `0x1419AD260` - native limited hinge runtime info

Returns:

- runtime size `0x60`;
- solver results `10`.

Interpretation:

- This is a second native witness that motor runtime offsets are absolute from the runtime base and follow the solver result block.

### `0x1419B1D50` - native ragdoll constraint constructor

Relevant writes:

- transform atom data occupies the expected transform area;
- ragdoll motor enabled byte is around the ragdoll motor atom;
- packed runtime offsets include `0x90` and `0x94` for native ragdoll motor runtime fields;
- target matrix and motor pointers are initialized in the ragdoll motor atom area.

### `0x1419B2900` - native ragdoll runtime info

Returns:

- runtime size `0xB0`;
- solver results `0x12` / `18`.

Interpretation:

- `18 * 8 = 0x90`, so native ragdoll's initialized/previous-angle data at `0x90/0x94` again sits after the solver results.
- This does not mean ROCK's custom grab should use native ragdoll's `18` solver results. It means offsets must match the custom runtime layout's own solver result count.

## Major New Finding: ROCK Custom Runtime Layout Is High Risk

2026-05-12 later audit correction:

- This section is partially superseded by the later "constraint-info utility and switch-table audit" section.
- The original concern that `RUNTIME_SOLVER_RESULTS = 12` was probably wrong is no longer supported.
- FO4VR's own constraint-info utility reports the ROCK atom stream shape as 12 solver results:
  - ragdoll motor atom type `0x13` contributes 6;
  - each linear motor atom type `0x0B` contributes 2;
  - three linear motors therefore contribute 6;
  - total = 12.
- The remaining runtime-layout risk is narrower:
  - focused range disassembly now confirms atom type `0x0B` and `0x13` use current-runtime-cursor offset semantics;
  - the earlier "absolute from runtime base" conclusion is incorrect for ROCK's linear atom placement;
  - ROCK's odd-looking linear offsets are coherent relative-to-current-runtime-cursor values, not immediate overlap bugs;
  - the unresolved part is runtime size/padding and any helper-side warm-start behavior, not solver-result overlap.

Current ROCK constants:

- `RUNTIME_SOLVER_RESULTS = 12`
- `RUNTIME_REPORTED_SIZE = 0x100`
- `RT_RAGDOLL_INIT_OFFSET = 0x60`
- `RT_RAGDOLL_PREV_ANG_OFFSET = 0x64`
- linear init offsets: `{0x40, 0x31, 0x22}`
- linear previous target offsets: `{0x44, 0x38, 0x2C}`

Why this is dangerous:

- If ROCK reports `12` solver results, the solver result block is `12 * 8 = 0x60`.
- ROCK's ragdoll offsets `0x60/0x64` match that 12-result assumption.
- But ROCK's linear offsets `{0x40, 0x31, 0x22}` and `{0x44, 0x38, 0x2C}` fall inside or across the solver result block.
- That means linear motor runtime bytes/floats may overlap solver results if the reported count is 12.
- The current log text saying the linear offsets are "relative to per-atom ptrs" is not supported by native witnesses. Native atom offsets appear absolute relative to runtime base.

HIGGS source layout for its custom grab constraint:

- `SOLVER_RESULT_MAX = 6`;
- solver results occupy `0x00..0x2F`;
- angular initialized bytes at `0x30`;
- previous target angles at `0x34`;
- linear initialized bytes at `0x40`;
- previous target positions at `0x44`;
- `sizeof(Runtime) = 0x50`;
- external runtime reports `sizeof(Runtime) * 2`, i.e. `0xA0`;
- linear init offsets are expected to be `0x40, 0x41, 0x42`;
- linear previous target offsets are expected to be `0x44, 0x48, 0x4C`.

Current audit verdict:

- The custom atom order/size looks structurally plausible.
- The custom vtable slots look structurally plausible.
- The custom runtime layout constants are not currently trustworthy.
- This must be resolved before custom authority becomes the primary one-hand dynamic grab path.

### 2026-05-12 follow-up: live hknp path versus hkp witnesses

Confirmed from Ghidra:

- `0x1417E39C0` is reached by the live hknp world constraint creation path.
- It calls the constraint data vtable for:
  - type at `+0x20`;
  - atom/constraint info at `+0x28`;
  - runtime info at `+0x90`;
  - wrapper unwrapping at `+0xC0` when type is `0x0C` or `0x0D`.
- It allocates runtime memory from the returned runtime byte size and zeroes that runtime directly.
- No normal insertion-path call to vtable `+0xA0` was visible in this function.

Confirmed solver path:

- `hknpPrepareConstraintsTask::vfunction5` groups active constraints before solve.
- `hknpBuildConstraintJacobiansTask::vfunction5` builds solver jacobians for grouped constraints.
- The build task calls `0x141A55550`, a large atom-stream interpreter, using the stored atom stream pointer/size from the live constraint slot.
- This means hkp-style atom streams are not merely dead leftovers: the live hknp solver path consumes atom streams through the hknp task pipeline.

Confirmed wrapper path:

- `hknpBreakableConstraintData` wraps an inner `hkpWrappedConstraintData` and returns type `0x0C`.
- `hknpMalleableConstraintData` wraps an inner `hkpWrappedConstraintData` and returns type `0x0D`.
- Both wrappers delegate inner atom/constraint info and runtime info to the wrapped constraint data.
- `hknpMalleableConstraintData::vfunction6` delegates inner info, then appends a 0x18-byte wrapper atom at `this + 0x18`.
- `hknpMalleableConstraintData` stores a default float `0x3c23d70a` at object offset `+0x3C`, and clone/copy preserves that value.
- Constructor/copy xrefs found so far are class/clone/setup paths, not a confirmed Bethesda gameplay use of malleable constraints.

Implications:

- The user's warning is correct: an `hkp*` symbol alone is not enough.
- However, the live FO4VR hknp constraint solver is explicitly built to consume hkp-style constraint data/atoms through hknp world tasks.
- For ROCK custom grab, native hkp constructors are layout witnesses; live hknp world insertion plus the build-jacobians atom interpreter are the production-relevant paths.
- `hknpMalleableConstraintData` remains a research candidate, not a selected design. It is live-compatible in structure, but not proven useful for per-frame grab feel.

### 2026-05-12 follow-up: linear motor runtime offset witnesses

Additional native witnesses:

- `hkpPrismaticConstraintData::hkpPrismaticConstraintData` at `0x1419B1350`:
  - atom stream starts at object `+0x18`;
  - type `2` set-local-transforms atom starts at atom-stream offset `0x00`;
  - type `0x0B` linear motor atom appears at object offset around `+0xA8`, i.e. atom-stream relative offset `0x90`;
  - that linear motor atom writes initialized offset `0x50`;
  - that linear motor atom writes previous-target-position offset `0x54`.
- `hkpPrismaticConstraintData::vfunction19` at `0x1419B1B20`:
  - runtime size `0x58`;
  - solver results `10`;
  - `10 * 8 = 0x50`, so the linear motor initialized byte at `0x50` begins immediately after the solver-result block.
- `hkpLimitedHingeConstraintData::hkpLimitedHingeConstraintData` at `0x1419ACA30`:
  - motor-related packed runtime offsets include `0x50` and `0x54`;
  - this is a second independent native witness that motor runtime offsets are absolute from the runtime base.
- `hkpPositionConstraintMotor::vfunction5` at `0x141F61120`:
  - clones a 0x30-byte position motor;
  - copies type byte at `+0x10`;
  - copies min/max force at `+0x18/+0x1C`;
  - copies tau/damping at `+0x20/+0x24`;
  - copies proportional/constant recovery velocity at `+0x28/+0x2C`.

Implications for ROCK:

- The current source log saying linear motor offsets are relative to per-atom runtime pointers is not supported by the native witnesses or by `0x1417E39C0`.
- FO4VR's normal insertion path allocates one runtime block for the constraint and passes that block to the solver; no per-atom runtime base was found in the insertion path.
- Under ROCK's current reported runtime info:
  - solver results = `12`;
  - solver-result block = `0x00..0x5F`;
  - ragdoll runtime offsets `0x60/0x64` are coherent with that report;
  - linear offsets `0x40,0x31,0x22` and `0x44,0x38,0x2C` overlap solver-result storage.
- Under the HIGGS-shaped custom runtime:
  - solver results = `6`;
  - solver-result block = `0x00..0x2F`;
  - angular state starts at `0x30`;
  - linear offsets should be `0x40/0x41/0x42` and `0x44/0x48/0x4C`.
- Therefore, the current ROCK linear motor runtime offsets are incompatible with both:
  - ROCK's own current `12` solver-result report;
  - the original HIGGS custom grab runtime shape.
- This is a strong explanation candidate for the old custom-motor stutter/fighting behavior, but not the only candidate. Timing phase, proxy drive snapping, body-A authority, and target continuity are still open contributors.

Likely correction direction, pending final verification:

- Use a coherent custom runtime layout, probably HIGGS-shaped:
  - 6 solver results;
  - runtime size `0xA0` if FO4VR needs the same doubled external runtime safety margin;
  - ragdoll offsets `0x30/0x34`;
  - linear init offsets `0x40/0x41/0x42`;
  - linear previous target offsets `0x44/0x48/0x4C`.
- Do not implement this until the remaining runtime/addInstance audit is done.

## Verified FO4VR Physics Step Phase Order

### `0x141DF73A0` - `bhkWorld::Update` / physics world update

Ghidra timer labels and virtual calls confirm the step-listener order:

1. `TtPhysicsStepListeners-BeforeWholePhysicsUpdate`
   - listener vtable `+0x08`;
   - ROCK currently maps this to `onBeforeWholePhysicsUpdate`;
   - ROCK currently flushes native mouse spring here.

2. Per substep: `TtPhysicsStepListeners-BeforeAnyPhysicsStep`
   - listener vtable `+0x18`;
   - ROCK currently maps this to `onBeforeAnyPhysicsStep`;
   - ROCK currently flushes generated hand/body/weapon colliders here.

3. `TtCollide`
   - collision detection runs.

4. `TtPhysicsStepListeners-BetweenPhysicsCollideAndSolve`
   - listener vtable `+0x28`;
   - this happens after collision and before solver;
   - ROCK currently has this slot wired as a noop.

5. `TtSolve`
   - solver consumes bodies/constraints.

6. `TtPhysicsStepListeners-AfterAnyPhysicsStep`
   - listener vtable `+0x38`.

7. `TtPhysicsStepListeners-AfterWholePhysicsUpdate`
   - listener vtable `+0x48`.

Implication:

- For custom grab authority, `between collide and solve` is the strongest candidate for updating:
  - grab authority proxy body target;
  - constraint transform A/B;
  - ragdoll angular target;
  - motor force/tau fields.
- Updating these in normal game-frame `Hand::updateHeldObject` risks stale or mixed-phase targets.
- Updating proxy in one phase and constraint fields in another risks solver-visible disagreement.

Required future architecture:

- Extend `PhysicsStepDriveCoordinator` to support a real `betweenCollideAndSolve` callback.
- Drive proxy and constraint target/motor updates in one verified physics phase before solve.

## Current ROCK Timing/Authority Map

Current ROCK constructor setup:

- `PhysicsInteraction::_generatedBodyStepDrive.setDriveCallbacks(...)` accepts only:
  - whole-pre-step callback;
  - substep-pre-collide callback.

Current flush ownership:

- `driveNativeGrabFromPhysicsStep` runs whole-pre-step.
- `driveGeneratedCollidersFromPhysicsSubstep` runs substep-pre-collide.
- `Hand::flushPendingHeldNativeGrab` flushes `_nativeGrab`.
- `Hand::updateHeldObject` currently writes shared constraint target/motor state from game/update flow, not from the verified `between collide and solve` phase.

Implication:

- The old custom motor stutter may not have been caused only by motor math.
- It may have been caused by:
  - wrong runtime offsets;
  - target writes outside the solver-safe phase;
  - body A and constraint target updated in different phases;
  - using contact palm body as solver authority;
  - rebuilding/toggling runtime history.

## Dedicated Proxy Body Audit

Confirmed source facts:

- `HandBoneColliderSet::captureBoneLookup` samples `DebugSkeletonBoneSource::GameRootFlattenedBoneTree`.
- The generated palm anchor body is already rooted in the correct flattened hand frame.
- `HandBoneColliderSet::makeRoleFrame` derives `PalmAnchor` from the hand bone plus finger bases/back direction.
- `HandBoneColliderSet::create` creates the palm anchor as a keyframed `BethesdaPhysicsBody` using:
  - `kHandFilterInfo = (0x000B << 16) | (ROCK_HAND_LAYER & 0x7F)`;
  - `BethesdaMotionType::Keyframed`;
  - a generated convex palm shape.
- `HandBoneColliderSet::update` explicitly defers rebuilds while `palmAnchorBody.isConstrained()`.

Conclusion:

- The existing palm anchor is not wrong because of root-frame source.
- It is wrong as a primary one-hand grab body A because it is also a contact/collider lifecycle body.
- Constraining it contaminates contact evidence and can block/reorder hand collider rebuild logic.
- A dedicated no-contact per-hand grab authority proxy is still the cleaner design.

Proxy requirements still unresolved:

- exact no-contact filter/layer;
- whether to use `FO4_LAYER_NONCOLLIDABLE` or a new ROCK-owned no-contact layer/matrix policy;
- minimal shape strategy, since `BethesdaPhysicsBody::create` requires a non-null shape;
- whether proxy should be keyframed through existing `GeneratedKeyframedBodyDrive` or a specialized drive that writes in `between collide and solve`.

## Generated Body Drive Findings

Current generated collider drive:

- Queues render/skeleton sampled targets.
- Flushes during substep-pre-collide.
- Interpolates between previous and pending target across substeps.
- Uses Bethesda `CollisionObject_DriveToKeyFrame`.
- Does not predict or hard-sync by default.

Relevant implication:

- This drive is suitable for contact colliders.
- A grab authority proxy may need a specialized variant because its target and the constraint target should be committed together immediately before solve.
- If using the existing drive unchanged, proxy movement may be visible to collision before the held-object constraint target is updated, which can reintroduce phase mismatch.

## FO4VR-Native Extra Mechanisms To Research

These are not implementation decisions yet. They are now approved research candidates because FO4VR may offer better tools than Skyrim/HIGGS.

### `0x141E56820` - `hknpSafeEaseConstraintsAction`

Confirmed:

- Constructor calls `hknpEaseConstraintsAction`.
- Registers itself with a world signal/slot-like structure at world offset around `+0x558`.
- Destructor unsubscribes from the same structure.

Possible relevance:

- FO4VR has a native constraint-easing action that may provide smooth transition/ease behavior for constraints.
- It may be useful for startup fade or safe constraint insertion/removal.

Need more research:

- action update behavior;
- what constraint types it supports;
- how it modifies constraint force/strength;
- whether it is only for native constraint classes or can wrap custom hkp constraint data through hknp bridge;
- lifecycle hazards.

### `0x1419083C0` - `hknpEaseConstraintsAction`

Confirmed:

- Constructor receives world, constraint-id array, and count.
- Collects constraint ids and body ids for constraints of type `7` or `2`.
- Stores internal arrays of constraints/bodies.

Possible relevance:

- Native hknp action may ease constraints over time instead of forcing all authority instantly.

Need more research:

- virtual update/apply function;
- fields controlling duration/strength;
- whether it is safe for a continuously held grab or only for transition windows.

### `0x141F5B600` - `hknpMalleableConstraintData`

Confirmed:

- Wraps an existing constraint data object.
- Calls the wrapped constraint's runtime info through vtable `+0x90`.
- Stores wrapper fields including a float around `+0x34`.

Possible relevance:

- Could provide a FO4VR-native softness/scaling wrapper around an hkp constraint.
- Could be useful for weight/authority modulation if it scales impulses/forces cleanly.

Need more research:

- exact field meaning;
- wrapper behavior during solve;
- whether it interacts cleanly with motor max force and tau;
- whether it can wrap the custom grab data.

### `0x141960980` - `hknpBreakableConstraintData`

Confirmed:

- Wraps an existing constraint data object.
- Has a break threshold-like float initialized to `10.0f`.

Possible relevance:

- Could support future overforce release behavior.
- Not part of immediate dynamic grab authority until understood.

## Follow-Up Ghidra Audit - Slots, Native Easing, Wrappers

This section records the follow-up audit from the same 2026-05-12 research pass. It supersedes any older note that treats the custom add-instance vtable slot itself as the main suspected issue.

### Custom `addInstance` vtable slot correction

Source fact:

- ROCK currently defines `VTABLE_SLOT_ADD_INSTANCE = 20` in `src/physics-interaction/grab/GrabConstraint.h`.
- ROCK installs `addInstanceCallback` into that slot in `src/physics-interaction/grab/GrabConstraint.cpp`.

Ghidra facts:

- `hkpConstraintData::vfunction20` at `0x141A4ACD0` returns the runtime pointer-like second parameter unchanged.
- Xrefs for `0x141A4ACD0` include ragdoll vtable data at `0x142E18330`.
- `hkpConstraintData::vfunction21` at `0x141A4ACE0` zeroes a runtime pointer for a supplied byte count.
- Xrefs for `0x141A4ACE0` include ragdoll vtable data at `0x142E18338`.
- Earlier ragdoll vtable evidence places `hkpRagdollConstraintData::vfunction19` runtime-info at `0x1419B2900`, with xref at `0x142E18328`.

Interpretation:

- Raw vtable offsets line up as:
  - runtime info at vtable `+0x90`;
  - base `vfunction20` at vtable `+0x98`;
  - base `vfunction21` at vtable `+0xA0`.
- ROCK's zero-based slot `20` maps to vtable `+0xA0`, not to `+0x98`.
- Therefore the slot offset is likely correct for the inherited runtime-zeroing/add-instance-like function.

Remaining risk:

- The exact FO4VR call signature for this slot is still not fully proven.
- The base function decompiles as `(this, runtime, size)` because it only uses those parameters.
- HIGGS-style code often models add-instance as `(this, constraint, runtime, size)`.
- Current ROCK callback is declared as `(thisPtr, constraint, runtime, sizeOfRuntime)`.
- If FO4VR ever calls the base form with only runtime and size, a custom callback with the longer signature would see shifted arguments.
- However, the verified `0x1417E39C0` insertion path allocates/zeros runtime itself and may not call this slot in the common creation path.

Current verdict:

- Do not blame the vtable slot number without more evidence.
- Keep the runtime-layout risk as the higher-confidence custom motor problem.
- Before implementation, verify whether custom add-instance is ever called during ROCK constraint creation, either by binary callsite trace or instrumented logging on a research branch.

### Native ease action is not a generic custom-grab smoother

#### `0x1419086F0` - `hknpEaseConstraintsAction::vfunction7`

Confirmed behavior:

- Iterates tracked bodies/constraint ids and removes invalid body entries.
- Reads duration at object offset around `+0x20` and elapsed time at `+0x24`.
- If duration is positive, advances elapsed time by `dt`.
- Calls `FUN_141908C30` with interpolation factor `dt / (duration - previousElapsed)`.
- When the ease finishes, calls `FUN_1419086B0(this, world, 0.0)` and returns completion.

#### `0x141908C30` - native ease field interpolator

Confirmed behavior:

- Reads the wrapped constraint data and calls its type function at vtable `+0x20`.
- Handles only type `2` and type `7`.
- Type `2` interpolates two fields on native limited-hinge-style data.
- Type `7` interpolates five fields on native ragdoll-style data.
- Any other type emits a Havok error from `Extensions\Actions\EaseConstraints\hknpEaseConstraintsAction.cpp` saying the action does not handle this type.

Interpretation:

- This action is not a generic ease wrapper for arbitrary custom constraint data.
- It does not know how to ease ROCK's custom grab type.
- It may only help if ROCK deliberately uses native limited hinge/ragdoll data classes for a future feature.
- It should not be planned as the smoothing layer for the custom dynamic grab constraint.

#### `0x141E568C0` - safe-ease event callback

Confirmed behavior:

- Searches its tracked constraint ids for an event-supplied id.
- If matched, calls `FUN_1419086B0(..., 0.0)`.
- Removes the action from the world through a world/action removal helper.

Interpretation:

- Safe-ease is a lifecycle guard around native ease state.
- It does not change the above type limitation.

### `hknpMalleableConstraintData` is a real wrapper, but semantics need one more pass

#### Constructor `0x141F5B600`

Confirmed behavior:

- Calls `hkpWrappedConstraintData` constructor with the wrapped data pointer.
- Sets vtable to `hknpMalleableConstraintData`.
- Initializes an internal atom-like block at object offset around `+0x18`.
- Stores a boolean-like constructor argument at `+0x30`.
- Stores float bits `0x3C23D70A` at `+0x34`, which is approximately `0.01f`.
- Calls a helper that queries the wrapped constraint runtime info via wrapped vtable `+0x90`.

#### `0x141F5B760` - type function

Confirmed behavior:

- Returns type `0x0D`.

#### `0x141F5B770` - constraint info function

Confirmed behavior:

- Calls the wrapped constraint's info function at vtable `+0x28`.
- Adds/points wrapper atom info at object offset around `+0x18` with size `0x18`.

#### `0x141F5B7E0` - runtime info function

Confirmed behavior:

- Delegates runtime-info query to the wrapped data's vtable `+0x90`.

#### `0x141F5B800` - solver/build function

Confirmed behavior:

- Copies a solver/build input structure.
- Multiplies a float at input offset around `+0x24` by wrapper field `+0x34`.
- Calls the wrapped constraint's info function.
- Calls a solver helper with the modified input.

#### `0x141A4B450` - wrapped-data accessor

Confirmed behavior:

- Returns the wrapped data pointer stored at object offset `+0x10`.

Interpretation:

- Malleable is not a second drive authority. It wraps an existing constraint and scales at least one solver/build input value before delegating.
- The default scale of roughly `0.01f` suggests softness/strength modulation, but the exact solver meaning of input offset `+0x24` is not yet proven.
- It may be useful for FO4VR-native grab authority shaping if it can wrap ROCK's custom motorized constraint without disturbing motor force/tau semantics.
- It must not be used until its scale field and solver input semantics are independently mapped.

### `hknpBreakableConstraintData` remains future-only

#### Constructor `0x141960980`

Confirmed behavior:

- Calls `hkpWrappedConstraintData` constructor with the wrapped data pointer.
- Sets vtable to `hknpBreakableConstraintData`.
- Writes float bits `0x41200000` at object offset around `+0x18`, which is `10.0f`.

Interpretation:

- This is plausibly a threshold field, but its break condition and callback behavior are not mapped yet.
- It is not needed for ordinary one-hand dynamic grab parity.
- It may become useful later for overforce release, weapon snag release, or safety detachment, after separate research.

## Proxy Collision/Layer Audit Update

Additional source facts:

- `CollisionLayerPolicy.h` defines `FO4_LAYER_NONCOLLIDABLE = 15`.
- `PhysicsBodyClassifier.h` rejects active grab candidates on layer `FO4_LAYER_NONCOLLIDABLE`, so a future proxy on that layer should not become a selectable loose-object candidate.
- Current ROCK generated hand and weapon masks explicitly remove `FO4_LAYER_NONCOLLIDABLE`.
- ROCK currently owns generated interaction layers:
  - `ROCK_LAYER_HAND = 43`;
  - `ROCK_LAYER_WEAPON = 44`;
  - `ROCK_LAYER_BODY = 47`.
- The comments state vanilla configured layers stop at `46`, while the hknp matrix is 64 rows wide and ROCK currently owns `47` as extended matrix capacity.
- `isRockOwnedReusableLayer` currently returns true only for `43`, `44`, and `47`.
- `applyRockGeneratedLayerPolicies` currently normalizes only hand, weapon/reload, and body policies.

Generated body creation facts:

- `BethesdaPhysicsBody::create` requires a non-null hknp shape.
- It creates an `hknpPhysicsSystemData`, appends body/motion/material/shape entries, creates a `bhkNPCollisionObject`, adds it to the world, obtains the runtime body id, assigns material, applies motion type, sets filter info, enables body flags `0x08020000`, and activates the body.
- A future authority proxy therefore cannot be a pure transform handle in current ROCK infrastructure. It must be represented by at least a tiny or inert shape unless a new direct hknp body creation path is researched.

Proxy layer implications:

- `FO4_LAYER_NONCOLLIDABLE` is the lowest-scaffolding candidate because it is already rejected by active grab classification and excluded by ROCK hand/weapon masks.
- A dedicated extended ROCK proxy layer, for example in the unused `48..63` matrix range, would be cleaner architecturally because ROCK could explicitly set its row to zero and log/watchdog it.
- However, adding a new extended layer would require expanding layer policy ownership and verification beyond the current `ROCK_LAYER_BODY = 47` row.

Current verdict:

- The authority proxy should be a dedicated no-contact per-hand body, not the existing palm anchor.
- The proxy should be rooted from the same flattened hand frame source as the generated palm anchor, because that source is already the convention the user confirmed works.
- The no-contact layer choice is not final:
  - layer `15` is likely safe but relies on existing noncollidable behavior;
  - a new extended ROCK proxy layer is architecturally cleaner but needs explicit matrix policy work and verification.
- The proxy drive likely needs a proxy-specific physics-step path, because the existing generated keyframed body drive flushes pre-collide for contact colliders, while grab authority should update proxy and constraint fields together after collide and before solve.

## Follow-Up Ghidra Audit - Runtime Insertion, Velocity Drive, Weight Data

This section adds a correction from user testing context: native mouse spring is smoother and stutters less than the old custom motor path, but it still stutters and is not the quality target. The goal is not to preserve mouse spring as "good enough"; the goal is to understand why it is less bad, then reproduce/surpass that stability with proper custom dynamic authority.

### Constraint insertion does not call the custom add-instance slot in the normal path

#### `0x1415469B0` - constraint creation wrapper

Confirmed call flow:

- reserves/allocates a constraint id;
- calls `0x1417E39C0` to initialize the world constraint slot;
- dispatches create/constraint notifications;
- if needed, notifies body constraint graph changes.

#### `0x1417E39C0` - world constraint slot initialization

Disassembly confirms:

- calls constraint data vtable `+0x28` for constraint/atom info;
- calls vtable `+0x20` for type;
- if type is wrapper `0x0C` or `0x0D`, calls vtable `+0xC0` to get wrapped inner data and then calls the wrapped type function;
- calls vtable `+0x90` for runtime info;
- allocates the runtime buffer using the returned byte size;
- zeros the allocated runtime directly with `REP STOSD`;
- stores runtime byte size at constraint slot offset `+0x26`;
- stores runtime pointer at constraint slot offset `+0x28`.

Important implication:

- Normal FO4VR constraint insertion does not visibly call vtable `+0xA0` / ROCK's custom add-instance callback.
- The previous concern about add-instance callback signature is lower priority than the runtime layout itself.
- The strongest current custom motor stutter/corruption suspect remains mismatched solver-result count and runtime offsets.

### `DriveToKeyFrame` has a built-in transform snap fallback

#### `0x141E086E0` - `bhkNPCollisionObject` drive-to-keyframe wrapper

Confirmed behavior:

- Resolves the collision object's world and body id.
- Calls `0x14153A6A0` to compute linear and angular velocity needed to reach the target transform over `dt`.
- Looks up the body's motion-properties record from the motion properties library.
- Reads two float fields from that record:
  - record `+0x10` is used as a max linear velocity magnitude;
  - record `+0x14` is used as a max angular velocity magnitude.
- If the required linear or angular velocity exceeds the corresponding limit:
  - calls `0x141DF55F0` to set body transform;
  - calls `0x141DF56F0` with zero vectors to zero velocity.
- Otherwise:
  - calls `0x141DF56F0` to set the computed velocity.

Interpretation:

- `DriveToKeyFrame` is smooth only while the required follow velocity stays below motion-property caps.
- When the target jump exceeds caps, it deliberately snaps transform and zeroes velocity.
- This is a plausible source of visible stutter in generated body/proxy-style drives.
- A future grab-authority proxy should not blindly reuse the current generated collider `driveToKeyFrame` path without accounting for this fallback.

### `ApplyHardKeyFrame` computes and writes velocity directly

#### `0x14153ABD0`

Confirmed behavior:

- Calls `0x14153A6A0` to compute target linear/angular velocity from target position/orientation and `dt`.
- Calls `0x141539F30` to write the computed velocities.
- Does not perform the `DriveToKeyFrame` max-velocity snap fallback.

Interpretation:

- For a no-contact authority proxy, this direct hard-keyframe velocity path may be a better research candidate than `CollisionObject_DriveToKeyFrame`.
- It still needs phase correctness, deadband/filtering, max velocity policy, and safe activation handling.
- It should be studied as a proxy drive primitive, not as held-object authority.

### `SetBodyVelocity` / deferred velocity path

#### `0x141DF56F0`

Confirmed behavior:

- If not inside the command-buffer/TLS mode, calls `0x141539F30` directly.
- If inside command-buffer mode, queues an API command with body id and linear/angular velocity.
- After a non-zero velocity write, calls `0x141DF60C0`, which handles activation/island bookkeeping for the body.

#### `0x141539F30`

Confirmed behavior:

- Resolves body -> motion.
- Writes motion linear velocity at motion offset `+0x40`.
- Converts/writes angular velocity into the motion orientation frame and stores at motion offset `+0x50`.
- Skips tiny changes through epsilon/deadband comparisons.
- Triggers motion/body changed notifications.

Interpretation:

- Direct velocity writes are a native FO4VR primitive and have their own deadband.
- They can avoid transform snap if used correctly.
- They are not a grip model and must not write the held object as a second authority beside the constraint.

### Mouse spring update is an action-style finite drive, not just smoothing

#### Constructor `0x141E4A850`

Confirmed fields copied from cinfo-like input include:

- body id around action offset `+0x18`;
- target transform block written later at `+0x60..+0x88`;
- target position written later at `+0x90..+0x98`;
- local/body-space grab point around `+0xA0`;
- prior error/deadband state around `+0xB0`;
- tuning fields around `+0xC0..+0xCC`;
- body flag field around `+0xD0`.

#### Setters

- `0x141E4A960` writes target position to action offsets `+0x90..+0x98`.
- `0x141E4A980` writes target transform to action offsets `+0x60..+0x88`.

#### Update helper `0x141E4AA30`

Confirmed behavior:

- validates target body id and live simulation state;
- computes current grabbed-point world position from the body and stored local grab point;
- compares grabbed-point error to previous error state for deadband/early-out;
- calls `0x14153A6A0` to compute hard-keyframe-style velocities toward the target;
- scales/carries existing linear/angular velocity with the field around `+0xCC`;
- computes position-error correction using tuning fields around `+0xC4/+0xC8`;
- computes angular correction and clamps it by a field around `+0xC0 * dt`;
- writes angular velocity through `0x141539D30`;
- applies a linear impulse-like correction through `0x14153A250`;
- optionally enables body flags from the field around `+0xD0`;
- stores previous error back to `+0xB0..+0xBC`.

Interpretation:

- Mouse spring is smoother than the old motor path because it is one per-body hknp action with:
  - physics-step timing;
  - deadband;
  - previous-error state;
  - velocity carry;
  - finite angular clamp;
  - direct native velocity/impulse helpers.
- Mouse spring is still not HIGGS-quality because its tuning is mostly static and does not implement the full HIGGS-style finite-force mass, collision, deviation, and weapon/lever behavior.
- Future custom motor authority should reproduce the useful action-style stability traits without adopting mouse spring as pivot/orientation authority.

### Motion properties are a verified temporary-weight-feel surface

#### `0x14153B2F0` - set body motion properties

Confirmed behavior:

- resolves body id to motion id;
- writes the supplied motion-properties id into the motion at offset `+0x38`;
- dispatches a body/motion changed notification through the world signal/list around `+0x538`.

ROCK source cross-check:

- `NearbyGrabDamping.cpp` already uses this native function through a lease/restore system.
- It clones motion-properties records, modifies damping fields, adds/reuses the cloned record, sets the body motion-properties id, and restores the original id when the lease ends.

#### `0x141767A70` - motion properties library add/reuse

Confirmed behavior:

- compares 16 dwords of a candidate record against existing library entries for reuse;
- if no matching record exists, inserts a new 0x40-byte motion-properties record;
- returns the motion-properties id.

Current known record fields:

- `+0x10`: used by `DriveToKeyFrame` as max linear velocity.
- `+0x14`: used by `DriveToKeyFrame` as max angular velocity.
- `+0x18`: currently used by ROCK nearby damping as linear damping.
- `+0x1C`: currently used by ROCK nearby damping as angular damping.

Interpretation:

- Motion-properties leases are already a proven ROCK pattern for temporary physics feel changes.
- Future held-weight research should consider a held-object motion-properties lease that controls damping and possibly velocity caps, but only if it does not create snap fallback through `DriveToKeyFrame`.
- This is weight/response data only; it must not affect grip pivot authority.

### Mass and inertia functions are powerful but dangerous

#### `0x141E08C00` - `bhkNPCollisionObject` set mass

Confirmed behavior:

- resolves the collision object's body id and motion;
- computes inverse mass from the supplied mass;
- rewrites packed inverse inertia / inverse mass data at motion offset `+0x20`;
- uses the current packed inertia ratios while scaling the mass component.

Interpretation:

- FO4VR exposes a native wrapper that can mutate mass/inertia data.
- This is not a pivot function and does not choose grip.
- Mutating held-object mass/inertia during grab could produce stronger weight feel but has high restore risk, especially for shared-motion/multipart objects.
- If this is ever used, it needs the same kind of lease/restore discipline as motion-properties damping, plus multipart/shared-motion handling.
- Safer first design direction is still: use actual mass/inertia as input to finite motor force, damping, angular force ratio, deviation, haptics, and release calculations; do not rewrite mass unless research proves it is needed.

#### `0x141E08EF0` - get COM world

Confirmed behavior:

- resolves body -> motion;
- returns the motion position at motion offset `+0x00`.

Interpretation:

- COM is easy to read and should be treated as weight/lever data only.
- It must not become pivot, target frame, or grip authority.

### Direct hknp body creation exists but is not yet production-ready for the proxy

#### `0x141543FF0` - direct body creation path

Confirmed behavior:

- allocates or uses a body id;
- initializes body data through `0x141764C60` or `0x141764CF0` depending on motion/static state;
- can add the body to broadphase/world systems;
- invokes the shape vtable during creation/registration.
- locks the world while allocating/reusing the body id and initializing the body slot;
- calls `0x141561CA0` or `0x1415486A0` for motion/system setup depending on the incoming body description;
- when adding immediately, calls `0x1415441F0`;
- when a shape/mass payload is present and body flags do not include `0x80000`, registers shape/mass properties through the world structure around `+0x610`;
- fires body-created listeners through the world listener array around `+0x4D0`.

Related functions:

- `0x141764C60` initializes one body mode and sets body flags including static-style bit `0x1`.
- `0x141764CF0` initializes another body mode and sets body flags including dynamic-style bit `0x2`.
- `0x141546350` allocates/initializes a motion.
- `0x1415464C0` removes/frees motion ids from the world motion manager.

Interpretation:

- FO4VR has a direct hknp body path that could avoid Bethesda `bhkNPCollisionObject` wrapper overhead for a no-contact authority proxy.
- This path still requires a valid shape; a pure no-shape transform handle is not proven and likely unsafe.
- Direct body lifetime/destruction and broadphase detach are now partially mapped, but still too lifecycle-heavy to treat as a trivial helper.
- Current production-safe proxy research should assume a valid tiny shape is required, whether through the existing `BethesdaPhysicsBody` wrapper or a later direct hknp proxy helper.

### Body lifetime commands

Ghidra confirms the hknp API command processor dispatch table at `hknpApiCommandProcessor::vfunction5`:

- command `1` -> `0x1415441F0`: add bodies to world/broadphase/island state;
- command `2` -> `0x141544E80`: destroy body;
- command `3` -> `0x141544B00`: remove body;
- command `6` -> `0x1415395E0`: set body transform;
- command `9` -> `0x141539F30`: set body velocity;
- command `0x15` -> `0x14153B2F0`: set body motion-properties id;
- command `0x1A` -> `0x14153C090` / `0x14153C150`: enable/disable body flags.

Confirmed lifecycle details:

- `0x1415441F0` adds bodies to active world/broadphase/island state:
  - has timer regions including `LtAddBodies`, `StSetup`, `StAddToBroadPhase`, and `StFireCallbacks`;
  - registers body ids into world mappings and active structures;
  - invokes body/shape virtuals while preparing insertion;
  - computes broadphase AABBs from shape, motion, and world extent data;
  - inserts into broadphase through the world broadphase interface around `world + 0x178`, vtable slot `+0x08`;
  - fires body-added callbacks through world listener state around `+0x4D8` via helper `0x14153EE00`.
- `0x141544B00` removes a body from active world/broadphase state:
  - fires `TtFireBodyRemoved`;
  - invokes world body-removed listeners at world `+0x4E0`;
  - detaches broadphase/island/collision cache state;
  - removes from broadphase through the world broadphase interface around `world + 0x178`, vtable slot `+0x10`;
  - sets body `+0x6C` to `-1`;
  - marks body flags at `+0x40` with `& 0xFFFBFFFF | 0x400`;
  - calls motion cleanup around `0x14154AA50`.
- `0x141544E80` calls `0x141544B00` first, then fully destroys/releases:
  - fires `TtFireBodyDestroyed`;
  - invokes world body-destroyed listeners at world `+0x4E8`;
  - releases shape references;
  - removes shape/mass property registration from the world structure around `+0x610`;
  - if the body owns motion state, removes the motion through `0x1417E2940`;
  - calls body cleanup helper `0x141548940`;
  - clears body flags;
  - appends the body slot to the free list;
  - increments world body/destruction bookkeeping;
  - calls `0x1417D8820` after the batch.

Interpretation:

- Existing `BethesdaPhysicsBody` wrapper creation is still the safest known production lifecycle because it creates a real physics-system instance, adds it through Bethesda's bhk world path, assigns filter/material, enables flags, activates the body, and destroys through `BhkWorld_RemovePhysicsSystemInstance`.
- Direct hknp body creation remains possible research, but production use would need a verified add/remove/destroy pairing, listener behavior, broadphase behavior, shape/mass-property lifetime, and motion-owner cleanup. This is not complete enough to select for first implementation.

## ROCK Layer And Generated Body Lifecycle

ROCK source confirms the current project layer namespace:

- layer `43`: ROCK hand/reload bodies;
- layer `44`: ROCK weapon/gun bodies;
- layer `47`: ROCK body/fullbody generated bodies;
- layers `48..63`: matrix-addressable extended rows, but not currently named/owned by ROCK policy.

Important source files:

- `src/physics-interaction/collision/CollisionLayerPolicy.h`
  - `ROCK_LAYER_HAND = 43`;
  - `ROCK_LAYER_WEAPON = 44`;
  - `ROCK_LAYER_BODY = 47`;
  - `ROCK_LAYER_EXTENDED_FIRST = ROCK_LAYER_BODY`;
  - `ROCK_LAYER_EXTENDED_LAST = 63`;
  - `isRockOwnedReusableLayer()` currently returns only `43`, `44`, and `47`;
  - `applyRockGeneratedLayerPolicies()` writes the hand, weapon, reload, and body matrix rows and normalizes extended-layer symmetry.
- `src/physics-interaction/core/PhysicsInteraction.cpp`
  - `registerCollisionLayer()` obtains the native collision matrix, applies `applyRockGeneratedLayerPolicies()`, stores expected masks, and logs pair state;
  - the watchdog re-registers if hand/weapon/reload/body masks or critical actor/body pair symmetry drift.
- `src/physics-interaction/body/BodyBoneColliderSet.cpp`
  - generated fullbody bodies use filter info `(0x000B << 16) | 47`;
  - root flattened skeleton capture is through `DirectSkeletonBoneReader::capture(..., GameRootFlattenedBoneTree)`;
  - bodies are created with `BethesdaPhysicsBody::create(..., Keyframed)`;
  - creation calls `placeGeneratedKeyframedBodyImmediately()` and `initializeGeneratedKeyframedBodyDriveState()`;
  - per-frame target capture uses `queueGeneratedKeyframedBodyTarget(..., teleportDistance=1000.0f)`;
  - physics-step flush uses `driveGeneratedKeyframedBody()`.
- `src/physics-interaction/hand/HandBoneColliderSet.cpp`
  - generated hand bodies use filter info `(0x000B << 16) | 43`;
  - palm anchor and non-anchor colliders follow the same generated keyframed body lifecycle.
- `src/physics-interaction/weapon/WeaponCollision.cpp`
  - generated weapon bodies use filter info `(0x000B << 16) | 44`;
  - collision can be suppressed by OR-ing the suppression no-collide bit;
  - bodies start collision-disabled according to options, then are published/enabled later.
- `src/physics-interaction/native/BethesdaPhysicsBody.cpp`
  - wrapper creation builds `hknpPhysicsSystemData`, body cinfo, motion cinfo, material, shape refs, `bhkPhysicsSystem`, `bhkNPCollisionObject`, and a linked `NiNode`;
  - `AddToWorld` creates the runtime hknp physics-system instance and body id;
  - post-add setup validates motion, assigns global material, applies motion type, sets filter info, enables flags `0x08020000`, activates the body, and verifies the body `+0x88` collision-object back pointer;
  - destroy removes the native physics-system instance from bhkWorld, destroys the NiNode, releases the collision object, and resets state.

Design implication:

- A future hidden authority proxy can reuse the proven generated-body lifecycle and activation discipline, but it should not automatically reuse the generated collider drive method.
- If a dedicated proxy layer is chosen from `48..63`, policy, watchdog, classifier, debug, suppression, and detection rejection all need to be explicitly extended. Current source does not yet own those rows.
- A no-contact proxy layer would be for the hidden solver anchor only. Contact-capable hand/weapon/body generated colliders should remain on layers `43`, `44`, and `47`.

## FO4VR Physics-Step Listener Timing

Ghidra-confirmed update loop: `bhkWorld::vfunction43` at `0x141DF73D6`.

The native update order is:

1. `TtPhysicsStepListeners-BeforeWholePhysicsUpdate`
   - listener vtable `+0x08`;
   - engine companion callback after it at `+0x10`.
2. Per substep: `TtPhysicsStepListeners-BeforeAnyPhysicsStep`
   - listener vtable `+0x18`;
   - engine companion callback after it at `+0x20`.
3. `TtCollide`.
4. Per substep: `TtPhysicsStepListeners-BetweenPhysicsCollideAndSolve`
   - listener vtable `+0x28`;
   - engine companion callback after it at `+0x30`.
5. `TtSolve`.
6. Per substep: `TtPhysicsStepListeners-AfterAnyPhysicsStep`
   - listener vtable `+0x38`;
   - engine companion callback after it at `+0x40`.
7. `TtPhysicsStepListeners-AfterWholePhysicsUpdate`
   - listener vtable `+0x48`;
   - engine companion callback after it at `+0x50`.

`0x141DFA7B0` is the step-listener registration helper. It appends a listener pointer to the bhkWorld listener array if not already present. The bhkWorld update clears the listener array after the world update, matching ROCK's current re-register-for-next-step design.

ROCK source state:

- `src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp` already matches the confirmed vtable offsets.
- Current callbacks forward only:
  - `beforeWhole`;
  - `beforeAny`.
- Current `betweenCollideAndSolve`, `afterAny`, and `afterWhole` callbacks are no-op.
- `src/physics-interaction/core/PhysicsInteraction.cpp` flushes held native grab and generated colliders from the current substep pre-collide callback.

Design implication:

- The old custom motor stutter risk is consistent with game-frame target writes or pre-collide-only writes being out of phase with collide/solve.
- A future proxy + constraint authority path should be researched around `betweenCollideAndSolve`, because it is the native slot after contact generation and before constraint solving.
- This is not an implementation instruction yet; it is the confirmed timing map needed before designing the next custom authority path.

## Direct Velocity Path Versus DriveToKeyFrame

Confirmed functions:

- `0x14153A6A0`: computes hard-keyframe linear and angular velocity needed to move a body from current motion to a target transform over `dt`;
- `0x14153ABD0`: direct hard-keyframe velocity helper;
  - calls `0x14153A6A0`;
  - calls `0x141539F30` with the computed velocities;
  - does not apply the DriveToKeyFrame cap-triggered transform snap in this wrapper.
- `0x141539F30`: set body velocity;
  - writes linear velocity to motion `+0x40`;
  - transforms/writes angular velocity into motion orientation/body-local convention at motion `+0x50`;
  - deadbands tiny linear/angular changes;
  - activates/marks the body when needed;
  - notifies world listeners through world `+0x538`.
- `0x141DF56F0`: deferred/direct set-velocity wrapper;
  - direct-calls `0x141539F30` when not inside the command queue context;
  - otherwise queues command `9`;
  - calls `0x141DF60C0` when either linear or angular velocity is meaningfully nonzero.
- `0x141DF55F0`: deferred/direct set-transform wrapper;
  - direct-calls `0x1415395E0` when not inside command queue context;
  - otherwise queues command `6`.
- `0x141E086E0`: collision-object DriveToKeyFrame wrapper;
  - computes hard-keyframe velocities through `0x14153A6A0`;
  - reads current motion-properties record from world `+0x5D0`;
  - compares computed linear velocity magnitude against record `+0x10`;
  - compares computed angular velocity magnitude against record `+0x14`;
  - if either cap is exceeded, calls set-transform and then zeroes velocity;
  - otherwise calls set-velocity with the computed velocities.

Design implication:

- DriveToKeyFrame is convenient for generated collider following, but it is not a clean proxy-drive primitive for grab authority because a fast hand movement can trigger a transform snap and velocity zero.
- The direct hard-keyframe velocity path is the more relevant research target for a no-contact authority proxy: it preserves the native compute-velocity math and velocity writer while avoiding the cap-triggered snap branch.
- This does not prove direct velocity is sufficient; it still needs phase, target continuity, and deactivation behavior mapped.

## Motion-Properties Record Map

Confirmed by Ghidra:

- world `+0x5D0` points to the motion-properties library;
- library `+0x28` points to 0x40-byte records;
- library `+0x30` is the live record count;
- `0x141767A70` adds/reuses a 0x40-byte record:
  - if input record starts with zero at `+0x00`, it searches existing entries for a byte-identical 0x40-byte record;
  - otherwise it allocates from the free list;
  - writes the input record to `entries + id * 0x40`;
  - notifies library listeners;
  - returns the new/reused short id.
- `0x14153B2F0` sets a body's current motion `motionPropertiesId`:
  - resolves body `+0x68` to motion index;
  - writes the short id at motion `+0x38`;
  - notifies world listeners through world `+0x538`.

Record field evidence:

- `+0x10`: max linear velocity used by DriveToKeyFrame cap test;
- `+0x14`: max angular velocity used by DriveToKeyFrame cap test;
- `+0x18`: damping-like float, currently used by ROCK nearby grab damping as linear damping;
- `+0x1C`: damping-like float, currently used by ROCK nearby grab damping as angular damping;
- `+0x20` / `+0x24`: written by `0x141841E60` based on mode and input scale; not fully named;
- `+0x28`: written as `param2 * param2` by `0x141841F30`; not fully named;
- `+0x2C`: written as clamped squared value from `param3 * DAT_142E01440`; not fully named;
- `+0x30`: written as reciprocal of a scaled `param2`; not fully named;
- `+0x34` / `+0x36`: short fields written by `0x141842020`; not fully named;
- `+0x38`: byte written by `0x141841F30`; not fully named;
- `+0x39` / `+0x3A` / `+0x3B` / `+0x3C`: compact byte fields written by `0x141841F30` and `0x141842020`; not fully named.

Preset builder evidence:

- `0x141841D30` zeroes record bytes from `+0x04` through `+0x3F`, then fills fields for preset modes `0..4`.
- Mode `1` and mode `2` write high velocity caps (`+0x14 = 100.0f`) and zero damping at `+0x18/+0x1C`.
- Mode `3` writes `+0x18 = 30.0f` and `+0x1C = 30.0f`, then zeroes velocity caps.
- Mode `4` writes small linear damping-like value at `+0x18`, angular damping-like value `0.5f` at `+0x1C`, and lower angular cap-like value `10.0f` at `+0x14`.

Design implication:

- ROCK's existing nearby damping lease is using a Ghidra-confirmed native field pair, but that is only damping. It does not by itself reproduce HIGGS-style finite force or angular lever feel.
- Velocity caps are confirmed to influence DriveToKeyFrame snap behavior. Changing caps for a proxy or held body could change snap thresholds, but this is risky unless the drive primitive is also chosen deliberately.
- Motion properties are useful for held-state polish and damping, but they are not a replacement for finite-force constraint authority.

## Mouse Spring Current Role

Current native mouse spring findings remain valid:

- It is an hknp action-style per-body drive, not a conceptual grab model.
- It has built-in dt-aware correction, damping/cap/deadband/previous-error behavior.
- It is smoother than the old custom path because it is a native moving-target action.

Current issue:

- The native path is too static/strong for heavy objects and loose weapons.
- It does not expose the HIGGS-style finite-force, mass cap, angular-to-linear force ratio, collision tau, inertia, and deviation behavior we need.

Future research implication:

- When replacing it, ROCK must preserve the good action-style timing/smoothing properties using FO4VR-native step timing and possibly native constraint-easing helpers.
- The replacement must not copy native mouse-spring pivot/orientation conventions as design authority.

## Contact Versus No-Contact Authority Proxy Clarification

Important correction:

- HIGGS does not implement dynamic grab as a globally no-contact interaction.
- HIGGS uses contact heavily for detection, haptics, held-state collision state, collision-aware tau changes, and release cleanup.
- The "no-contact proxy" idea in this document is a ROCK/FO4VR architecture proposal for a hidden motor-authority anchor only. It should not be read as "make the whole grab no-contact."

Confirmed HIGGS behavior:

- `Hand::CreateHandCollision` in `src/hand.cpp:560-614` creates a real keyframed `handBody`:
  - real `bhkBoxShape`;
  - layer `56` plus player collision group;
  - bit `15` set for same-group behavior;
  - bit `14` initially set to disable collision until update enables it;
  - added to the world with `hkpWorld_AddEntity`.
- `Hand::UpdateHandCollision` in `src/hand.cpp:655-666` disables hand collision while `state == State::HeldBody`:
  - this prevents the hand collider from directly colliding against the object it is currently constraining;
  - it is not a general removal of contact from the dynamic grab system.
- `Hand::TransitionHeld` in `src/hand.cpp:1592-1619` constrains the real `handBody->hkBody` to the selected object body:
  - `pivotA` is computed from `palmPos` transformed into hand body local space;
  - `pivotB` is computed from selected `ptPos` transformed into object body local space;
  - `CreateGrabConstraint(bodyA, bodyB, handTransformHandSpace, handTransformObjSpace)` creates the motorized constraint.
- `Hand::TransitionHeld` in `src/hand.cpp:1636-1646` adds entity contact listeners to connected held bodies and forces `setContactPointCallbackDelay(0)`.
- `PhysicsListener::contactPointCallback` in `src/physics.cpp:660-719`:
  - ignores already disabled contacts;
  - treats HIGGS hand, weapon, and held bodies as special;
  - disables only specific held-object contacts against the player's collision group;
  - calls `HandleIgnoredContact` for temporary pair-specific release/throw ignore windows;
  - records used contact points and separating velocity for collision state and haptics.
- `EntityCollisionListener::contactPointCallback` in `src/physics.cpp:798-808` records used contact points for held connected bodies.
- `Hand::Update` held-body logic in `src/hand.cpp:3971-3978` uses `EntityCollisionListener::IsColliding()` to select colliding versus non-colliding tau targets.

Design implication for ROCK:

- If ROCK creates a dedicated custom motor authority body, that body should likely be no-contact because it is not a physical hand collider; it is a hidden solver anchor.
- ROCK should still keep separate contact-capable hand/collider systems for detection, haptics, world touch, held-object collision state, and collision-aware motor response.
- The clean architecture is therefore two roles, not one:
  - visible/physical hand collision bodies: contact evidence and world interaction;
  - hidden grab authority proxy: stable no-contact body A for the custom constraint.
- This separation may avoid a HIGGS-specific compromise where the same `handBody` is both the contact collider and the constraint anchor, requiring collision disable while physically holding.

## HIGGS Dynamic Grab Behavior Map - Source Audit

This section is behavioral mapping from the approved HIGGS source tree at `E:\fo4dev\skirymvr_mods\source_codes\higgs`. It is not a FO4VR implementation decision. It is used to define which behavior ROCK must reproduce or surpass with FO4VR-native mechanisms.

### Dynamic state machine boundary

Confirmed HIGGS states in `include/hand.h`:

- `Idle`;
- `SelectedFar`;
- `SelectedClose`;
- `SelectionLocked`;
- `PreGrabItem`;
- `PrePullItem`;
- `Pulled`;
- `HeldInit`;
- `Held`;
- `HeldBody`;
- `GrabFromOtherHand`;
- `GrabExternal`;
- `LootOtherHand`;
- `SelectedTwoHand`;
- `HeldTwoHanded`.

Dynamic scope for this ROCK research:

- `SelectedClose`, `SelectedFar`, `SelectionLocked`, `Pulled`, `HeldBody`, and release from `HeldBody` are relevant.
- `HeldInit` and `Held` are keyframed-object states and are only boundary context.
- `SelectedTwoHand` and `HeldTwoHanded` are out of scope except where they share body/constraint cleanup.
- Actor/ragdoll-specific branches inside `HeldBody` are out of scope except where they share the same custom constraint and connected-body functions.

### Detection pipeline

Confirmed `Hand::FindCloseObject(...)` behavior:

- Temporarily reconfigures a sphere phantom:
  - collision filter info set to `0x2C`;
  - radius set to `nearCastRadius`;
  - transform moved to the hand/palm start.
- Performs `hkpWorld_LinearCast` from palm start to `start + castDirection * nearCastDistance`.
- Iterates hit collidables and requires:
  - valid rigid body;
  - valid user data wrapper;
  - reference not equal to player;
  - selectable object, or the other hand's selected object when the other-hand grab path is valid.
- Projectiles are filtered so only impacted/stuck projectiles are grabbable.
- Candidate priority is the hit distance from the cast ray, not distance to COM.
- A special offhand two-handed weapon branch can select the other hand's weapon body, but this is outside current loose-object scope.
- If no normal close candidate is found and the hand has a pulled object, it performs a wider penetration check against that pulled object and returns the pulled body if present.
- It restores phantom filter, radius, and transform after the query.

Confirmed `Hand::FindFarObject(...)` behavior:

- Computes a far target along the pointing direction.
- Performs a raycast first; if the ray hits world geometry, the later linear cast is clipped to the ray hit position.
- Temporarily reconfigures the same sphere phantom:
  - filter `0x2C`;
  - radius `farCastRadius`;
  - transform at ray start.
- Performs a sphere linear cast to the clipped target.
- Filters candidates similarly to close detection.
- Requires the candidate direction from HMD to hit point to satisfy `requiredCastDotProduct` against HMD forward.
- Candidate priority is again distance from hit point to the pointing ray.

Behavioral meaning:

- HIGGS detection is contact/geometry based from the beginning.
- The selected point is the cast/contact hit point carried forward into selection and grab state.
- COM is not part of candidate priority.
- Pull fallback intentionally broadens only for the already-pulled object so a pulled object can be caught as it arrives.

### Selection, lock, and pull entry

Confirmed behavior in the main `Hand::Update` state logic:

- In `SelectedFar`, pressing grab within the trigger leeway locks the selection:
  - stores `pulledPointOffset = selectedObject.point - hkObjPos`;
  - enters `SelectionLocked`;
  - keeps the selected contact point relative to the object motion position for later pull/arrival.
- In `SelectedClose`, pressing grab goes directly toward `TransitionHeld(...)` after geometry-based rigid-body selection:
  - `GetRigidBodyToGrabBasedOnGeometry(...)` may replace the originally hit rigid body with the body that best matches graphics/contact geometry;
  - `ComputeInitialObjectTransformFromUpdatedCollidableNode(...)` may provide an initial transform for authored/standard adjustments;
  - `TransitionHeld(...)` receives the selected object point and palm frame.
- In `SelectionLocked`, if the locked object becomes close and the detected close body is the selected body, HIGGS calls `TransitionHeld(...)` with the newly detected close contact point.
- If the locked object is not close, HIGGS checks controller velocity projected toward the selected point:
  - if speed toward the hand exceeds `pullSpeedThreshold`;
  - and `IsObjectPullable()` allows it;
  - it enters the pulled path.

Behavioral meaning:

- HIGGS separates far lock from pull from close grab.
- A far selection does not immediately create a hand/object constraint.
- Touch/close grab can refresh the actual close contact point when the object arrives.
- Pulling is velocity-driven by a deliberate hand motion, not only by holding selection.

### Pull/converge behavior

Confirmed `TransitionPulled` behavior:

- Cancels/restores an existing pulled object if pulling a new one.
- Stores pulled object handle/body.
- Saves the pulled body's angular damping and replaces it with `pulledAngularDamping`.
- Converts keyframed pulled bodies to dynamic.
- Converts debris quality to moving quality when configured.
- Handles impacted projectiles by forcing a collision filter/motion-type change so they become physically movable.
- Computes pull duration with `SetPulledDuration(...)`:
  - distance = length from palm to pulled object point;
  - `pullDuration = A + B * exp(-C * distance)`;
  - `pulledExpireTime = pullDuration + 1.0`.
- Triggers haptic/sound/API pulled callbacks and enters `Pulled`.

Confirmed `State::Pulled` update:

- Uses object motion position plus `pulledPointOffset` as the pull point.
- For `pullApplyVelocityTime`, computes a predicted velocity to reach `pullTarget`.
- During `pullTrackHandTime`, updates `pullTarget` to palm plus a small vertical destination offset.
- Horizontal velocity is `horizontalDelta / remainingDuration`.
- Vertical velocity includes a gravity compensation term: `0.5 * 9.81 * duration + verticalDelta / duration`.
- Collects connected rigid bodies every frame.
- Updates pulled collision-ignored bodies to the connected set.
- If the grabbed connected set is not attached to fixed bodies, applies the same velocity to every moveable connected body.
- If attached to a fixed body, only the selected body receives the velocity.
- After the velocity application window expires, HIGGS deselects and returns to idle.

Behavioral meaning:

- Pull/converge is not a spring-to-COM constraint.
- It is a short finite predicted velocity phase toward the hand with gravity compensation.
- Connected/multipart bodies are moved coherently when safe.
- Pull is intentionally time-limited and hands off to close grab when contact arrives.

### Dynamic release behavior

Confirmed release from `HeldBody`:

- Release consumes the release request and sets `idleDesired`.
- If another hand is still holding the same body, drop events and consume/stash are suppressed for this hand.
- It computes hand release velocity:
  - base hand velocity from controller/hand velocity;
  - optional boost if above `throwVelocityThreshold`;
  - player/world velocity component;
  - optional tangential velocity from hand angular velocity.
- Tangential velocity uses COM only as release lever data:
  - axis = normalized hand angular velocity;
  - hand-to-COM vector = object COM world - palm;
  - project hand-to-COM onto the rotation plane;
  - tangential direction = cross(axis, projected vector);
  - tangential magnitude = lever distance * angular speed;
  - clamp to `tangentialVelocityLimit`;
  - if clamped, angular velocity is reduced to match the clamp.
- If release velocity is above threshold, HIGGS temporarily disables hand/object contacts to avoid immediate throw collision.
- For `HeldBody`, it collects connected rigid bodies and computes a release object velocity from recent local object linear velocities.
- The held-body release path writes the selected body's linear velocity from that object velocity plus player-space smoothing.
- It removes the grab constraint from world and hand body.
- It removes entity contact listeners from connected bodies.
- It restores saved contact callback delays unless the other hand still owns that connected body.
- It restores saved inverse inertia unless the other hand still owns that connected body.
- It clears saved contact-delay and inertia maps.
- It queues haptics, sounds, drop callbacks, and dropped tracking when appropriate.

Behavioral meaning:

- COM is used on release for realistic tangential throw contribution, not pivot authority.
- Release velocity comes from both hand motion and actual recent object motion, not from an arbitrary final controller target.
- Cleanup is multipart-aware and two-hand-aware.
- Collision ignore on throw is temporary and conditional on release velocity.

### Dynamic transition into held state

Relevant HIGGS functions/files:

- `src/hand.cpp`
  - `Hand::TransitionHeld(...)`;
  - `Hand::Update(...)` / held-body branch;
  - nearby selection and pull/converge state logic around the same state machine.
- `include/constraint.h`, `src/constraint.cpp`
  - `GrabConstraintData`;
  - `CreateGrabConstraint(...)`;
  - `GrabConstraintData::setTargetRelativeOrientationOfBodies(...)`.

Confirmed `TransitionHeld` behavior:

- The selected object mass at grab time is captured from inverse mass for haptics/weight context.
- A selected point is established from the closest/contact point and may be improved by graphics-triangle evidence.
- The desired object transform preserves the object's current rotation.
- The desired object transform is translated so the selected object point seats into the hand/palm point.
- The object is not rotated around COM to fit the hand.
- COM is not used as grip pivot.
- For dynamic physics grab, HIGGS creates:
  - `pivotA` from the palm/hand-side point transformed into the hand body frame;
  - `pivotB` from the selected object contact point transformed into the object body frame;
  - a desired hand/object transform relation from the seated desired transform;
  - a custom grab constraint between hand body and object body.
- Connected bodies are collected and contact listeners are attached for held collision state.
- Inertia is captured and clamped/normalized on the grabbed body set, then restored later by lifecycle cleanup.

### HIGGS custom grab constraint shape

Confirmed HIGGS custom constraint fields:

- It uses a custom `hkpConstraintData` with:
  - a set-local-transforms atom;
  - setup-stabilization atom;
  - one ragdoll motor atom for angular authority;
  - three linear motor atoms for X/Y/Z linear authority.
- Runtime layout:
  - six solver results at `0x00..0x2F`;
  - angular initialized bytes at `0x30`;
  - angular previous target angles at `0x34`;
  - linear initialized bytes at `0x40,0x41,0x42`;
  - linear previous target positions at `0x44,0x48,0x4C`;
  - source runtime struct size `0x50`;
  - external runtime reported as `sizeof(Runtime) * 2`, i.e. `0xA0`.
- `setMotorsActive(false)` clears the six solver results, which explicitly resets motor history when disabling motors.
- `setTargetRelativeOrientationOfBodies(bRa)` writes angular target as `bRa * transformA.rotation`.

Research implication for ROCK:

- ROCK's custom motor path must treat linear motor runtime offsets as absolute runtime offsets, matching the native FO4VR witnesses.
- ROCK's current axis 1/2 linear runtime offsets are a high-risk mismatch against both HIGGS custom runtime and FO4VR native motor witnesses.
- The FO4VR live hknp insertion path confirms vtable/runtime allocation is reachable, but the exact safe runtime size and solver-result count still need final verification.

### Held-state dynamic update loop

Confirmed HIGGS held-body behavior:

- HIGGS updates visual hand pose from the held object's current transform, not by forcing the held object to visually match the controller perfectly.
- It computes:
  - `heldTransform` from the current object/collidable node transform;
  - inverse of the desired object-in-hand relation captured during grab;
  - adjusted hand transform = held object transform * inverse desired relation.
- At grab start, the visual hand can lerp from the current hand transform to the adjusted hand transform over a short duration derived from the grab distance.
- It tracks hand deviation as distance between adjusted/held visual hand and the physical/controller hand.
- Deviation is accumulated in a rolling buffer.
- If average deviation exceeds configured allowed hand distance after ignore windows, HIGGS releases/drops instead of infinitely forcing the object.

Behavioral meaning:

- Heavy or blocked objects are allowed to lag behind the physical hand.
- The player's visual hand follows the actual held object relation during dynamic hold.
- The system does not make the player hand an infinitely strong kinematic authority.
- Deviation is both polish and safety: it allows mass/constraint lag, then releases when the hand/object relationship becomes physically unreasonable.

### Per-frame pivot and target refresh

Confirmed HIGGS held-body constraint update:

- Rebuilds the desired hand/object relation each frame from the captured desired node transform and the rigid-body local transform.
- Updates angular motor target through `GrabConstraintData::setTargetRelativeOrientationOfBodies(...)`.
- Recomputes object-side pivot B from the desired hand transform in object space applied to the palm point.
- Writes that new transform-B translation into the constraint transform atom.

Behavioral meaning:

- Pivot remains hand/contact-frame based.
- The solver target can stay stable even when body-local rigid transforms and graphical object transforms differ.
- The object-side pivot is not recaptured from COM or from current closest point during held state.
- COM remains weight data only.

### Mass, force, angular authority, and collision response

Confirmed HIGGS non-actor held-body motor policy:

- It distinguishes non-actor ordinary objects from weapons using the selected object's base form type.
- Loose weapons use a different linear max force constant than ordinary objects.
- Angular max force is derived from linear max force through `angularToLinearForceRatio`.
- During fade-in, `angularToLinearForceRatio` can interpolate from a fade-in ratio to the normal ratio.
- Collision state comes from `EntityCollisionListener::IsColliding()`.
- Collision state selects different target tau values for linear and angular motors.
- Tau values are advanced gradually toward target values with a configured lerp speed.
- For non-actor objects, actual mass caps the linear max force:
  - mass is derived from inverse mass;
  - linear max force is limited by `mass * grabConstraintMaxForceToMassRatio`;
  - angular max force is limited again from the capped linear force divided by the angular-to-linear ratio.
- Motor min force is set to negative max force.
- Motor tau, damping, recovery, and force values are written each frame.
- The grabbed rigid body is activated after updates.

Behavioral meaning:

- Heavy objects do not become unmovable, but they are not controlled with infinite authority.
- The heavier the body, the more finite the acceleration feels because motor force is capped against body mass.
- Angular authority is not an independent magic value; it is linked to linear authority through a ratio.
- Collision changes responsiveness, so the held object behaves differently when colliding than when freely moving.
- Loose weapons get explicit dynamic-grab force policy, but still use the loose-object dynamic path, not equipped weapon logic.

### Inertia and connected-body handling

Confirmed HIGGS behavior:

- At transition into held state, HIGGS saves inverse inertia for connected grabbed bodies.
- It clamps per-axis inverse inertia so no axis can become too extreme relative to the minimum inverse inertia axis.
- It also clamps against a configured minimum inertia.
- Connected bodies are collected through constraint relationships.
- Non-moveable connected bodies are detected and marked.
- Connected/contained bodies feed object mass registration and player-space body registration.

Behavioral meaning:

- Long/heavy/multipart objects are not treated as a single point at the palm.
- Inertia normalization prevents pathological rotation without replacing grip pivot with COM.
- Connected body collection is part of weight, contact, cleanup, and player-space compensation, not just grab creation.

### Dynamic hand pose and finger math

Confirmed HIGGS dynamic pose path in `Hand::TransitionHeld`:

- Finger pose is computed during dynamic grab from geometry/contact context, not from special per-object `HIGGS_R/L` grip nodes.
- Graphics triangles near the palm/grab line are gathered and transformed to world space.
- The selected/contact point can be improved using closest graphics geometry intersection evidence.
- A hand scale is derived from the configured hand size.
- For each finger curve:
  - a start position is transformed from hand space to world space;
  - a finger normal is transformed to world space;
  - a zero-angle vector is transformed to world space;
  - the finger start is offset by the palm-to-selected-point vector so the test is performed as if the palm is seated onto the object.
- HIGGS calls `GetIntersections(...)` against nearby triangles to find where each finger curve contacts the object.
- If an intersection exists, the intersection angle drives the finger curl.
- If a finger does not intersect, the fallback behavior differs from full geometry contact and can result in open/default pose values.
- Thumb has alternate curve handling when the standard thumb curve is invalid/misses.
- Finger values are clamped to avoid overcurl.
- During held state, `fingerAnimator.SetFingerValues(...)` blends toward the captured dynamic values with speed increasing over the early grab window.

Behavioral meaning:

- The natural hand blend comes from solving fingers against nearby object triangles after seating the palm/contact relation.
- Special authored grip nodes are not the core hand-pose solution for ordinary dynamic grab.
- The system computes a plausible static target curl from local object geometry and then animates toward it.
- ROCK can reproduce this in FO4VR if it can gather/render or collision-triangle evidence around the seated palm/grip relation, without requiring per-object `ROCK_R/L` nodes.

### Loose weapon dynamic behavior

Confirmed HIGGS loose weapon behavior in the dynamic held loop:

- Loose weapons are detected from base form type in the ordinary dynamic held-object path.
- They use weapon-specific linear max force.
- Angular max force still derives from linear force through the angular-to-linear ratio.
- Mass cap still applies to non-actor loose weapons.
- They are not treated as equipped weapon grips in this dynamic path.
- The available source evidence does not show loose dynamic weapons using hardcoded equipped weapon hand offsets as the authority model.

Research implication for ROCK:

- ROCK loose non-equipped weapon dynamic grab should be researched and implemented as a loose-object dynamic variant with weapon force/lever policy.
- Equipped weapon handling and two-hand equipped weapon handling remain separate.
- For loose weapons, grip-to-COM lever length should be considered only as weight/torque/inertia data, not as pivot selection.

## ROCK Current Dynamic Grab Source Map - Replacement-Relevant Findings

This section was added after re-reading this whole file in the resumed 2026-05-12 research pass. It records source-confirmed ROCK behavior only; it is not an implementation plan.

### Current one-hand dynamic authority split

Source-confirmed files:

- `src/physics-interaction/hand/Hand.h`
- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/grab/GrabConstraint.h`
- `src/physics-interaction/grab/GrabConstraint.cpp`
- `src/physics-interaction/grab/GrabMotionController.h`
- `src/physics-interaction/grab/GrabHeldObject.h`

Current drive enum:

- `HeldObjectDriveMode::NativeMouseSpring`
- `HeldObjectDriveMode::SharedConstraint`

Current behavior:

- Ordinary one-hand loose-object and loose non-equipped weapon grabs use `HeldObjectDriveMode::NativeMouseSpring`.
- `SharedConstraint` is currently used for the shared/two-hand loose-object path, especially when joining a peer-held loose object.
- The HIGGS-style finite-force controller exists in ROCK as `grab_motion_controller::solveMotorTargets(...)`, but `Hand::updateHeldObject(...)` calls `updateConstraintGrabDriveMotors(...)` only when `_heldDriveMode == SharedConstraint`.
- Therefore ordinary one-hand dynamic grab currently has:
  - contact/palm/non-COM pivot capture;
  - native mouse-spring BODY-frame target following;
  - adaptive lead;
  - deviation release;
  - visual held-hand relation;
  - release velocity composition.
- Ordinary one-hand dynamic grab currently does not have the custom motor path's:
  - per-frame mass-capped max force;
  - angular max force derived from linear force;
  - collision-tau response;
  - motor fade-in force shaping;
  - loose-weapon shared-constraint force/tau multipliers.

Replacement implication:

- The replacement target is not "improve mouse spring."
- The replacement target is to move ordinary one-hand dynamic grab onto a corrected custom linear/angular authority path while preserving the current working frame conventions and acquisition/release behavior.
- Mouse spring remains a stability witness only: it shows which FO4VR BODY-frame and action/update properties avoided some old custom-motor stutter, but it is not the desired authority model.

### Current grab commit and frame capture

`Hand::grabSelectedObject(...)` source-confirmed behavior:

- Rejects non-ROCK active grab targets before dynamic object work.
- Detects `joiningPeerHeldObject`, `grabbedFromPullCatch`, and `looseWeaponGrab`.
- Scans/prepares the selected object's physics body set, converts accepted bodies to dynamic, enables collision, and chooses a primary body near the selected/grip evidence.
- Uses mesh/contact/authored-node evidence as evidence, not as competing authorities.
- Builds a three-phase grab relation through:
  - `grab_three_phase::buildGrabPocketFrame(...)`;
  - `grab_three_phase::buildObjectGripArea(...)`;
  - `grab_three_phase::classifyAcquisitionPhase(...)`.
- The normal dynamic relation is `rockPointToPalm`:
  - palm/pocket point becomes the hand-side seat;
  - selected/contact/grip seed becomes the object-side grip point;
  - object rotation is preserved unless an authored grab node explicitly supplies a frame;
  - object translation is shifted so the selected grip point seats into the palm pocket.
- `_grabFrame.pivotBBodyLocalGame` is frozen from `grabBodyWorldAtGrab` and the chosen grip point.
- `_grabFrame.pivotAHandBodyLocalGame` is generated by `grab_frame_math::buildSplitGrabFrameFromDesiredObject(...)`.
- Current source explicitly sets `_grabFrame.bodyLocal` to identity for the native BODY-frame path, because current logs showed the visible held node follows the hknp BODY frame directly. This is a working convention to preserve or consciously replace, not a COM pivot.

Replacement implication:

- The future motor authority should reuse the same contact/palm frame capture unless new research proves a better FO4VR-native reference frame.
- COM must not replace `_grabFrame.pivotBBodyLocalGame`.
- The hidden proxy/body-A design must take `pivotA` from the generated root-flattened palm/proxy frame, while `pivotB` remains the captured contact/grip point in object BODY space.

### Current pull/converge path

`Hand::startDynamicPull(...)` and `Hand::updateDynamicPull(...)` source-confirmed behavior:

- Pull starts only from `SelectionLocked` far selection.
- Pull preparation converts object bodies to dynamic and enables collision before pull motion.
- Pull point is derived from the selected point/body relation and stored as `_pullPointOffsetHavok`.
- Pull duration uses `pull_motion_math::computePullDurationSeconds(...)`.
- `updateDynamicPull(...)` computes distance from the live object pull point to the palm/pocket point.
- When the object reaches the configured arrival/touch envelope, the state returns to close selection and the close grab path reuses the arrived hit point.
- During active pull, `pull_motion_math::computePullMotion(...)` supplies finite velocity toward a predicted hand target.
- Velocity is applied to the primary and held body set through `setHeldLinearVelocity(...)`.

Replacement implication:

- Pull/converge should remain a finite velocity acquisition phase, not become a COM-centered spring.
- The custom motor replacement should take over at grab commit/held authority, not collapse far pull directly into a held constraint before close/touch evidence exists.

### Current held update split

`Hand::updateHeldObject(...)` source-confirmed behavior:

- Validates the active drive mode:
  - native mode requires `_nativeGrab.isValid()`;
  - shared constraint mode requires `_activeConstraint.isValid()`.
- Builds desired object/body world transforms from the captured hand-space relation each frame.
- Computes live grip error by transforming `_grabFrame.pivotBBodyLocalGame` through the live grab-authority body transform.
- Native path:
  - computes adaptive lead from controller/object relative velocity and angular velocity;
  - writes the led target into `_nativeGrab.queueTarget(...)`.
- Shared constraint path:
  - calls `updateConstraintGrabDriveTarget(...)`;
  - later calls `updateConstraintGrabDriveMotors(...)`.
- Deviation release is shared across both paths.
- Visual hand relation is visual-only and derived from held-object transform times inverse frozen object-hand relation.
- Three-phase convergence can delay full touch-held pose/fade behavior until the grip reaches touch distance or times out inside the pocket.
- Mesh finger pose updates are independent of native versus shared drive after pose capture.

Replacement implication:

- Custom one-hand authority must move target, pivot, angular target, and motor updates out of the normal game-frame `updateHeldObject(...)` timing or at least synchronize them with the verified physics phase.
- The visual hand relation/deviation logic is already conceptually aligned with HIGGS and should be preserved.
- The replacement must avoid two active writers: no `_nativeGrab.queueTarget(...)` on the same body while custom motors drive it.

### Current custom constraint source shape

`GrabConstraint.h/.cpp` source-confirmed behavior:

- Constraint data allocation size: `GRAB_CONSTRAINT_SIZE = 0x168`.
- Atom stream:
  - set-local-transforms atom at `0x20`;
  - setup-stabilization atom at `0xB0`;
  - ragdoll motor atom at `0xC0`;
  - linear motor atoms at `0x120`, `0x138`, `0x150`.
- Runtime report:
  - `RUNTIME_SOLVER_RESULTS = 12`;
  - `RUNTIME_REPORTED_SIZE = 0x100`.
- Ragdoll motor runtime offsets:
  - init `0x60`;
  - previous angle `0x64`.
- Linear motor runtime offsets:
  - init `{0x40, 0x31, 0x22}`;
  - previous target `{0x44, 0x38, 0x2C}`.
- Source log currently claims the linear offsets are relative to per-atom runtime pointers.

Replacement implication:

- This source state matches the previously documented Ghidra risk:
  - FO4VR native witnesses indicate absolute runtime offsets from one runtime block;
  - normal hknp insertion allocates one runtime buffer;
  - current linear offsets overlap solver-result storage under the current 12-solver-result report.
- This is still the strongest concrete old-custom-motor stutter/fighting suspect.
- The next Ghidra work should focus on the live atom interpreter's linear motor atom (`0x0B`) and ragdoll motor atom (`0x13`) cases to confirm exactly how enabled byte, axis, runtime init offset, previous-target offset, target value, and motor pointer are consumed.

### 2026-05-12 Ghidra follow-up: live atom interpreter runtime base

Binary-confirmed functions:

- `hknpBuildConstraintJacobiansTask::vfunction5` at `0x14197F9A0`;
- helper `0x141979EF0`;
- atom-stream interpreter `0x141A55550`.

Confirmed call chain:

- `hknpBuildConstraintJacobiansTask::vfunction5` calls `0x141979EF0` for constraint groups before solve.
- `0x141979EF0` reads the live hknp constraint slot and calls:
  - atom stream pointer from slot offset `+0x18`;
  - atom stream byte size from slot offset `+0x20`;
  - runtime pointer from slot offset `+0x28`;
  - atom interpreter `0x141A55550(atomStream, atomSize, context, &writerState)`.
- Immediately before calling `0x141A55550`, `0x141979EF0` initializes a writer-state block where:
  - writer-state `[0]` is the schema/jacobian output pointer;
  - writer-state `[1]` is the constraint runtime pointer from slot `+0x28`;
  - writer-state `[2]` is another copy/related pointer initialized from the same runtime pointer.
- Inside `0x141A55550`, motor-like atom cases compute runtime addresses by adding atom-provided offsets to writer-state `[1]` and/or `[2]`.

Concrete evidence from the atom interpreter:

- The interpreter is reached from the live hknp build-jacobians task, not only from a dead hkp path.
- The runtime pointer passed into the interpreter is one pointer from the hknp constraint slot, not a separate per-atom runtime pointer.
- At least one motor-like case in `0x141A55550` uses atom bytes as offsets added to writer-state runtime pointers, then advances writer-state runtime pointers by solver-result-sized increments.

Replacement implication:

- This confirms the previous native-witness conclusion more strongly: custom linear/ragdoll motor runtime offsets must be coherent offsets into the single runtime block allocated for the constraint.
- ROCK's current source comment/log claiming linear offsets are relative to per-atom runtime pointers is not supported by the live hknp build-jacobians path.
- A corrected custom one-hand authority must choose one coherent runtime shape and make:
  - runtime byte size;
  - solver result count;
  - ragdoll init/previous-angle offsets;
  - linear init/previous-target offsets;
  - atom stream order;
  - atom stream size;
  agree with what the live interpreter consumes.

Still unresolved:

- The exact decompiled case bodies for atom type `0x0B` linear motor and atom type `0x13` ragdoll motor are still being narrowed inside `0x141A55550`.
- The Ghidra MCP decompile/disassembly output for the full interpreter is very large and truncates the middle of the function in normal output, so the next pass must identify the relevant switch table entries and inspect those case bodies without relying on truncated output.

### 2026-05-12 Ghidra follow-up: constraint-info utility and switch-table audit

Binary-confirmed functions/data:

- `0x141A4AD20` - native constraint-info utility used by ROCK through `offsets::kFunc_GetConstraintInfoUtil`.
- `0x141A4AEE8` - constraint-info utility switch table for atom types `0x00..0x22`.
- `0x141A55550` - live hknp build-jacobians atom interpreter.
- `0x141A5A32C` - live atom-interpreter switch table for atom types `0x00..0x1A`.

Constraint-info utility findings from `0x141A4AD20`:

- Type `0x02` / set-local-transforms:
  - switch-table entry points to `0x141A4AED1`;
  - advances the atom cursor by `0x90` bytes;
  - does not add solver results.
- Type `0x17` / setup-stabilization:
  - switch-table entry points to `0x141A4ADEF`;
  - matches native ragdoll/limited-hinge helper placement before motor atoms;
  - still needs exact case-level disassembly, but it is not the source of the 12-result count.
- Type `0x13` / ragdoll motor:
  - switch-table entry points to `0x141A4AE8D`;
  - adds `0xC0` bytes of schema/jacobian output;
  - adds 6 solver results;
  - advances the atom cursor by `0x60` bytes.
- Type `0x0B` / linear motor:
  - switch-table entry points to `0x141A4AE16`;
  - adds `0x50` bytes of schema/jacobian output;
  - adds 2 solver results;
  - advances the atom cursor by `0x18` bytes.

Implication for ROCK's atom stream:

- ROCK stream:
  - set-local-transforms type `0x02` at atom offset `0x00`, size `0x90`;
  - setup-stabilization type `0x17` at atom offset `0x90`, size `0x10`;
  - ragdoll motor type `0x13` at atom offset `0xA0`, size `0x60`;
  - three linear motor type `0x0B` atoms at atom offsets `0x100/0x118/0x130`, size `0x18` each.
- That shape exactly matches the source layout:
  - `ATOM_TRANSFORMS = 0x20`;
  - `ATOM_STABILIZE = 0xB0`;
  - `ATOM_RAGDOLL_MOT = 0xC0`;
  - `ATOM_LIN_MOTOR_0/1/2 = 0x120/0x138/0x150`;
  - `ATOMS_SIZE = 0x148`.
- The native info utility's solver-result count for this stream is:
  - ragdoll motor: 6;
  - linear motor 0: 2;
  - linear motor 1: 2;
  - linear motor 2: 2;
  - total: 12.
- Therefore `RUNTIME_SOLVER_RESULTS = 12` is coherent with FO4VR's native info utility for the current atom stream.

Native constructor cross-checks:

- `hkpRagdollConstraintData` helper `0x1419B2910` constructs:
  - type `0x02` at stream offset `0x00`;
  - type `0x17` at stream offset `0x90`;
  - type `0x13` at stream offset `0xA0`;
  - later angular/limit atoms after the ragdoll motor.
- The ragdoll constructor packs runtime offsets `0x90/0x94` into the type `0x13` atom, and native runtime info reports size `0xB0`, solver results `18`.
- `hkpPrismaticConstraintData` constructor `0x1419B1350` writes a type `0x0B` linear motor atom at its own stream offset `0x90`, packs runtime offsets `0x50/0x54`, and native runtime info reports size `0x58`, solver results `10`.
- `hkpLimitedHingeConstraintData` helper `0x1419AD2B0` constructs type `0x12` angular motor layout and also packs motor runtime offsets around `0x50/0x54`.

Corrected runtime-offset interpretation:

- The earlier "all motor runtime offsets are absolute from runtime base" conclusion is no longer strong enough.
- The live atom interpreter maintains runtime writer/cursor pointers in `param_4[1]` and `param_4[2]`.
- A visible live case, type `0x16` at `0x141A59AF8`, reads atom-provided runtime offsets by adding them to the current runtime writer/cursor, then advances that cursor by one solver result.
- Focused range disassembly of the approved unpacked FO4VR executable confirmed the type `0x0B` linear motor case at `0x141A57162` also adds runtime offsets to the current runtime writer/cursor.
- FO4VR's constraint-info utility proves that each type `0x0B` linear motor advances the runtime solver-result cursor by 2 solver results (`0x10` bytes), and the live case does exactly that at `0x141A5727E/0x141A5728C`.
- ROCK's linear offsets therefore resolve as:
  - linear 0 current cursor after ragdoll = `0x30`; init `0x40` -> runtime `0x70`; previous target `0x44` -> runtime `0x74`;
  - linear 1 current cursor = `0x40`; init `0x31` -> runtime `0x71`; previous target `0x38` -> runtime `0x78`;
  - linear 2 current cursor = `0x50`; init `0x22` -> runtime `0x72`; previous target `0x2C` -> runtime `0x7C`.
- Under that interpretation:
  - total solver-result block is `12 * 8 = 0x60`;
  - ragdoll initialized/previous-angle data at `0x60/0x64` starts immediately after solver results;
  - linear initialized bytes at `0x70/0x71/0x72` and previous targets at `0x74/0x78/0x7C` do not overlap solver results.

Current verdict:

- ROCK's atom stream order, atom sizes, and reported solver-result count now look structurally coherent for FO4VR.
- ROCK's current linear motor runtime offsets are not the obvious corruption source previously suspected.
- The previous custom-motor stutter/fighting explanation should not be reduced to "solver count/runtime offsets are corrupt."
- The remaining custom-authority risks are now more likely a combination of:
  - target writes occurring outside the verified physics phase;
  - using a contact palm body as body A rather than a dedicated authority proxy;
  - `DriveToKeyFrame` snap fallback if used for the proxy;
  - more than one authority writing the held body;
  - motor tuning/finite-force policy not matching HIGGS-style mass/collision/deviation behavior.

Still worth verifying, but no longer the main blocker:

- Whether `RUNTIME_REPORTED_SIZE = 0x100` is required padding or could be reduced.
- Whether zeroing the whole runtime block remains sufficient on every insertion path.
- Whether helper calls inside type `0x0B` and type `0x13` have side effects that matter for motor tuning or warm-start behavior.

#### Focused range disassembly: type `0x0B` linear motor case

Reason for focused local disassembly:

- Ghidra MCP decompiles `0x141A55550` only as one huge function and truncates the middle of the output.
- The switch table and case addresses were first confirmed in Ghidra MCP.
- The byte-range disassembly used only the user-provided binary path:
  - `E:\fo4dev\reverse_engineering\Fallout4VR.exe.unpacked.exe`;
  - PE image base verified as `0x140000000`;
  - `.text` section verified to cover the case addresses.

Case `0x141A57162`, atom type `0x0B`, confirmed field reads:

- `0x141A57162`: checks enabled byte at atom `+0x02`.
- `0x141A5716D`: checks motor pointer at atom `+0x10`.
- `0x141A57178`: reads signed 16-bit initialized offset from atom `+0x04`.
- `0x141A5717D`: reads signed 16-bit previous-target offset from atom `+0x06`.
- `0x141A57186`: adds initialized offset to current runtime cursor at writer state `[+0x08]`.
- `0x141A5718A`: adds previous-target offset to current runtime cursor at writer state `[+0x08]`.
- `0x141A5718E`: reads initialized byte at the computed runtime address.
- `0x141A57195`: if uninitialized, copies atom target float from atom `+0x08` into the computed previous-target runtime address.
- `0x141A571A9`: reads axis byte at atom `+0x03`.
- `0x141A57202`: reads current target float from atom `+0x08`.
- `0x141A57275/0x141A57279`: updates previous target and sets initialized byte when needed.
- `0x141A5727E`: advances runtime cursor `[+0x08]` by `0x10`.
- `0x141A5728C`: advances secondary runtime/schema cursor `[+0x10]` by `0x10`.
- `0x141A57288`: advances atom cursor by `0x18`.

Case `0x141A57162`, interpretation:

- Linear motor runtime offsets are relative to the current runtime cursor.
- Each linear atom owns 2 solver-result slots (`0x10` bytes), matching `0x141A4AD20`.
- ROCK's `{0x40,0x31,0x22}` initialized offsets and `{0x44,0x38,0x2C}` previous-target offsets intentionally collapse to contiguous post-solver runtime storage when added to each linear atom's current cursor.

#### Focused range disassembly: type `0x13` ragdoll motor case

Case `0x141A5873C`, atom type `0x13`, confirmed field reads:

- `0x141A5873C`: checks enabled byte at atom `+0x02`.
- `0x141A58747`: reads signed 16-bit initialized offset from atom `+0x04`.
- `0x141A5874C`: reads signed 16-bit previous-angle offset from atom `+0x06`.
- `0x141A58755`: adds initialized offset to current runtime cursor at writer state `[+0x08]`.
- `0x141A58759`: adds previous-angle offset to current runtime cursor at writer state `[+0x08]`.
- `0x141A5875D`: passes target rotation matrix at atom `+0x10` into helper `0x1417D05F0`.
- `0x141A5877B`: treats atom `+0x40` as the start of the three angular motor pointer slots.
- The loop processes three angular axes; null motor pointers produce inactive/fallback schema output.
- `0x141A58CE8/0x141A58CEE`: initializes previous-angle/runtime state for an axis and sets the initialized byte.
- `0x141A58EDB`: updates previous angle when the solved target differs.
- `0x141A58F35`: advances runtime cursor `[+0x08]` by `0x30`.
- `0x141A58F55`: advances secondary runtime/schema cursor `[+0x10]` by `0x30`.
- `0x141A58F51`: advances atom cursor by `0x60`.
- Disabled path at `0x141A58F5F` also advances atom cursor by `0x60` and runtime cursors by `0x30`.

Case `0x141A5873C`, interpretation:

- Ragdoll motor runtime offsets are also relative to the current runtime cursor.
- In ROCK's stream the ragdoll motor is the first solver-producing atom after set-local-transforms/setup-stabilization, so its current runtime cursor is the runtime base.
- ROCK's ragdoll initialized/previous-angle offsets `0x60/0x64` therefore land immediately after the 12-result solver-result block.

### 2026-05-12 Ghidra follow-up: motor helper semantics

Binary-confirmed helper functions from live atom interpreter xrefs:

- `0x141AFD820` - linear motor constraint row preparation helper.
- `0x141AFD600` - constraint motor policy evaluator.
- `0x141AFD970` - linear motor schema/jacobian output writer.
- `0x141F61120` - `hkpPositionConstraintMotor::vfunction5` clone/copy helper.

Xrefs:

- `0x141AFD820` is called by live atom interpreter type `0x0B` at `0x141A571CF`.
- `0x141AFD600` is called by:
  - type `0x0B` at `0x141A57249`;
  - type `0x13` at `0x141A58E45`;
  - other motorized atom paths.
- `0x141AFD970` is called by live type `0x0B` at `0x141A5725E`.

`0x141AFD600` position motor field use:

- Reads motor type byte at motor `+0x10`.
- ROCK creates `HkPositionMotor::type = 1`, so the type-1 path is the relevant one.
- For type 1:
  - reads proportional recovery velocity at motor `+0x28`;
  - reads constant recovery velocity at motor `+0x2C`;
  - reads max force at motor `+0x1C`;
  - derives min force as the sign-flipped value from motor `+0x1C` in the decompiled path, while other paths also read motor `+0x18`;
  - reads tau at motor `+0x20`;
  - reads damping at motor `+0x24`;
  - uses timestep/inverse timestep from the solver context carried in the motor input block.
- It writes a compact output block used by the row writer:
  - target/error-like value;
  - previous/velocity correction contribution;
  - force limits;
  - tau;
  - damping.

Interpretation:

- FO4VR's live motor helper consumes the same fields ROCK writes:
  - min/max force;
  - tau;
  - damping;
  - proportional recovery velocity;
  - constant recovery velocity.
- That means the custom motor path can express finite-force mass feel without inventing a new low-level motor class.
- HIGGS-style mass/weight polish should be implemented as per-frame policy feeding these fields, not by changing pivot authority or using COM as target authority.

`0x141AFD820` linear row preparation helper:

- Uses body pointers from the build context at `param_3 + 0x30` and `param_3 + 0x38`.
- Builds Jacobian/effective-mass data from:
  - current body transforms;
  - pivot/axis vectors;
  - body inverse mass/inertia fields around motion offsets `+0x30..+0x3C`;
  - the target/axis state from the type `0x0B` atom.
- Writes prepared data into the schema block consumed by `0x141AFD600` and `0x141AFD970`.

Interpretation:

- Linear motor strength is not a raw teleport target. It goes through effective-mass/Jacobian preparation and motor force limits.
- This supports the replacement direction: corrected custom motors can produce weight and lag if fed sane finite-force parameters in the correct physics phase.

`0x141AFD970` linear row writer:

- Writes schema row type byte `10`.
- Scales output by the solver context values at `param_2 + 0x04` and `param_2 + 0x08`.
- Advances the schema output pointer by `0x50`, matching the constraint-info utility's type `0x0B` schema size.

Interpretation:

- The type `0x0B` linear atom is a real finite-force solver row path, not a dead hkp leftover.
- The custom replacement should use this existing position-motor path and fix authority timing/body setup rather than bypassing it with direct velocity writes to the held object.

ROCK source connection:

- `createPositionMotor(...)` writes exactly the fields consumed by `0x141AFD600`.
- `updateConstraintGrabDriveMotors(...)` already rewrites those fields each held update for the shared-constraint path:
  - linear/angular tau;
  - damping;
  - proportional and constant recovery;
  - min/max force.
- `grab_motion_controller::solveMotorTargets(...)` already contains the HIGGS-like policy shape:
  - error factor from position/rotation deviation;
  - collision tau;
  - tau lerp;
  - force fade-in;
  - mass-capped max force;
  - angular force derived from linear force by ratio;
  - loose-weapon multipliers.

Current interpretation:

- ROCK does not lack native motor capability.
- The likely old-custom-motor failure mode has moved away from "bad runtime offsets" toward:
  - ordinary one-hand path not using this controller;
  - custom targets/motor writes occurring from game update instead of the verified pre-solve physics phase;
  - body A being a contact palm body rather than a dedicated authority proxy;
  - proxy drive/body-A discontinuity;
  - possible double authority during promote/destroy transitions;
  - tuning values that make the finite-force path feel too stiff or too weak when moved to ordinary one-hand grab.

### 2026-05-12 Ghidra follow-up: malleable wrapper semantics

Binary-confirmed functions:

- `hknpMalleableConstraintData::hknpMalleableConstraintData` at `0x141F5B600`;
- setup helper `0x141F5B5A0`;
- runtime-info helper `0x141F5B5C0`;
- type function `0x141F5B760`;
- constraint-info function `0x141F5B770`;
- runtime-info function `0x141F5B7E0`;
- solver/build delegate `0x141F5B800`;
- wrapped-data accessor `0x141A4B450`.

Confirmed behavior:

- Constructor wraps an inner constraint data pointer through `hkpWrappedConstraintData`.
- Type function returns `0x0D`.
- The wrapper owns a small atom/block at object offset `+0x18`.
- Constructor initializes wrapper state:
  - word at wrapper block `+0x18` is set to `1`;
  - pointer at wrapper block `+0x20` is set to null by setup helper;
  - object byte `+0x30` stores the constructor flag;
  - object float/value at `+0x34` is initialized to `0x3C23D70A`;
  - helper `0x141F5B5C0` queries wrapped constraint runtime info through wrapped vtable `+0x90`.
- Constraint-info function calls the wrapped constraint's info function at vtable `+0x28`, then exposes the wrapper atom/block at `this + 0x18` with size `0x18`.
- Runtime-info function delegates to the wrapped constraint's vtable `+0x90`.
- Solver/build delegate `0x141F5B800`:
  - copies the incoming build context to a local context;
  - multiplies the incoming float at context offset `+0x24` by the wrapper value at object offset `+0x34`;
  - queries the wrapped constraint's atom info;
  - calls the same live atom interpreter `0x141A55550(...)` on the wrapped atom stream with the modified local context and the same writer/runtime state.
- `hkpWrappedConstraintData::vfunction25` at `0x141A4B450` returns the wrapped inner constraint data pointer.

Interpretation:

- `hknpMalleableConstraintData` is not a second motor authority.
- It does not expose separate linear/angular/pivot/deviation policy.
- It broadly scales one solver/build context float before delegating the wrapped atom stream to the normal interpreter.
- It may be useful as a global softness/strength wrapper only if the context `+0x24` field is proven to correspond to the desired impulse/constraint-strength term in the live solver path.
- It is not a substitute for the HIGGS-style per-frame policy:
  - mass-capped linear max force;
  - angular max force derived from linear force;
  - collision tau;
  - fade-in angular ratio;
  - loose-weapon force policy;
  - deviation/lag behavior.

Replacement implication:

- Do not design the custom one-hand grab replacement around malleable as the primary quality mechanism.
- Keep malleable as an optional later wrapper only after the core custom motor runtime, proxy, phase timing, and per-frame motor tuning are correct.
- If malleable is ever used, it should be treated as a coarse wrapper layered around a correct custom motor constraint, not as the source of physical grab feel.

### Current release path

`Hand::releaseGrabbedObject(...)` source-confirmed behavior:

- Restores nearby damping, hand collision, grabbed inertia, lifecycle/body flags, FRIK hand pose, visual hand override, and active drive state.
- Destroys `_activeConstraint` and `_nativeGrab` depending on current state.
- Release velocity uses:
  - hand/controller local velocity history;
  - held-object local velocity history;
  - player-space velocity;
  - tangential angular swing.
- Tangential swing uses COM only as release lever data:
  - lever origin is the frozen/live grip point when available;
  - COM is read for center-of-mass world position;
  - angular hand swing generates tangential release velocity around that lever relation.

Replacement implication:

- Release logic is already aligned with the COM rule: COM is release/weight data, not pivot.
- Custom motor replacement should preserve this release path unless runtime testing later proves a specific issue.

## Current Parity Gap Summary

What ROCK already has:

- contact/mesh/palm grip capture;
- non-COM pivot storage;
- hand-relative object relation;
- held visual relation/deviation scaffolding;
- custom motor constraint path;
- live-confirmed FO4VR position motor helper support for tau, damping, recovery, and finite min/max force;
- mass cap policy in `GrabMotionController`;
- loose weapon tuning surfaces;
- generated root-flattened hand bodies;
- player-space compensation and release logic.

What is missing or suspect:

- exact custom constraint runtime size/padding decision after the type `0x0B` and type `0x13` field-read correction;
- physics-phase-safe custom target writes;
- dedicated grab proxy body;
- no-contact proxy filter/layer policy for a hidden authority anchor only, not for the whole grab interaction;
- unified authority for proxy + constraint update before solve;
- proxy drive primitive that avoids `DriveToKeyFrame` snap fallback under fast hand motion;
- ordinary one-hand dynamic path still bypasses the custom finite-force motor controller;
- source tests that enforce custom one-hand authority instead of native one-hand authority;
- possible FO4VR-native malleable wrapper integration, if verified useful.
- native `hknpEaseConstraintsAction` is no longer a likely custom-grab smoothing candidate because it handles only native type `2` and type `7`.

## No-Code Implementation Implications

Before any custom one-hand replacement can be implemented:

1. Finish the custom runtime layout audit.
   - Solver result count `12` is now coherent with FO4VR's native constraint-info utility for the current ROCK atom stream.
   - Live type `0x0B` and type `0x13` field reads now confirm current-runtime-cursor offset semantics.
   - Confirm final runtime size and whether `0x100` is required or simply padded.
   - Confirm addInstance slot behavior and whether zeroing full runtime is sufficient.

2. Finish step-phase integration design.
   - Add planned architecture for a `betweenCollideAndSolve` callback.
   - Ensure proxy and constraint writes happen in the same phase.
   - Do not assume `CollisionObject_DriveToKeyFrame` is acceptable for the proxy; it has a verified cap-triggered transform snap fallback.

3. Finish proxy design.
   - Choose no-contact filter/layer for hidden authority proxy only.
   - Choose tiny/minimal shape.
   - Decide whether proxy should use direct hard-keyframe velocity (`0x14153ABD0`/`0x141539F30`) instead of generated-collider `DriveToKeyFrame`.

4. Continue FO4VR-native feature research.
   - `hknpMalleableConstraintData` solver input semantics;
   - `hknpBreakableConstraintData`.
   - motion-properties max velocity/damping fields for held weight feel;
   - mass/inertia lease feasibility only if force/motor input is insufficient.

5. Only then create implementation branch from `develop`.
   - Suggested branch remains `feature/custom-dynamic-grab-authority`.

## Open Questions

- Does FO4VR's hknp bridge require doubled runtime memory for custom hkp constraints, as HIGGS did, or can it use exact runtime size safely?
- Is ROCK's current `RUNTIME_REPORTED_SIZE = 0x100` intentionally padded, or can/should it be reduced after exact field-read verification?
- Is ROCK's current custom add-instance callback called from any alternate path outside normal constraint insertion, or is runtime zeroing entirely owned by `0x1417E39C0`?
- Can `hknpMalleableConstraintData` scale a custom motorized hkp constraint in a useful way, or is it only generic Havok compatibility plumbing in this binary?
- What exactly does the malleable wrapper scale at solver/build input offset around `+0x24`?
- Is `FO4_LAYER_NONCOLLIDABLE` safe for a constrained keyframed proxy body, or should ROCK add a dedicated no-contact layer/matrix row?
- Should proxy drive use direct hard-keyframe velocity, direct deferred velocity, or a specialized between-collide-and-solve keyframe update?
- Can a direct hknp proxy body be safely created/destroyed without Bethesda `bhkNPCollisionObject`, or is the wrapper path required for production safety?
- Which motion-properties fields beyond `+0x10/+0x14/+0x18/+0x1C` affect hknp held-object response?
- Should loose weapon dynamic grab add lever-length-aware angular force scaling beyond mass cap, using grip-to-COM only as weight data?
- Since ROCK's current linear motor runtime offsets now look correct relative to the runtime cursor, was old custom-motor stutter dominated by phase timing, body-A choice, proxy drive snapping, or competing authorities?

## Immediate Next Research Actions

- Inspect custom constraint runtime usage deeper in FO4VR, now focused on runtime size/padding, add-instance behavior, and atom helper side effects rather than solver-result count.
- Inspect `hknpMalleableConstraintData` solve behavior and fields.
- Inspect no-contact layer behavior and generated body constraints.
- Inspect direct hknp body creation or no-shape alternatives for a non-contact authority proxy.
- Inspect native APIs for changing/limiting dynamic body motion properties, velocity caps, and inertia safely during held state.
- Inspect native destruction/removal path for direct hknp-created proxy bodies.
- Inspect HIGGS dynamic release and pull/converge transitions only for dynamic behavior mapping, not as FO4VR implementation authority.
- Update the old tracker only to point at this file as superseding if needed.

## 2026-05-12 Continuation: HIGGS Dynamic Grab Source Mapping

This section is source-mapped from:

- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\hand.h`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\constraint.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\constraint.h`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\RE\havok.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\math_utils.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\finger_animator.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\finger_curves.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\main.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\physics.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\utils.cpp`

This is still research only. No ROCK implementation change is implied by this section.

### HIGGS dynamic contact model: not "no contact"

Confirmed source fact:

- HIGGS does not avoid contact as a design principle.
- HIGGS uses contact/casts/geometry heavily for:
  - close object detection;
  - far object detection;
  - pulled-object catch;
  - choosing the selected object point;
  - choosing the actual rigid body for skinned/multipart objects;
  - selecting finger curl values from object triangles;
  - held-object collision-state feedback through contact listeners.
- During `State::HeldBody`, HIGGS disables the generated hand collision body against the world/object by setting collision-disabled bit 14 in `Hand::UpdateHandCollision`.
- The same generated keyframed hand body still remains the constraint body A.

Interpretation:

- "No-contact authority proxy" should not mean "ignore contact evidence".
- The HIGGS-like rule is closer to:
  - contact is used before and around grab to understand the object;
  - the held drive anchor itself should not keep colliding with the held object while the constraint is driving it.
- In HIGGS, body A is the keyframed hand collision body. Its source comment is explicit: `handTransform must be where the hand is in real life, as the hand collision is what drives the grab constraint`.
- For ROCK, this reopens the body-A design question:
  - a dedicated no-contact authority proxy is still plausible in FO4VR if it avoids generated-collider lifecycle contamination;
  - but HIGGS does not prove a separate proxy is required. HIGGS proves that the body-A drive frame must be real-hand/root-frame aligned and non-colliding while held.

### HIGGS detection pipeline

`Hand::FindCloseObject(...)`:

- Temporarily configures the hand sphere phantom:
  - collision filter info `0x2C`;
  - radius `nearCastRadius`;
  - position at the hand/palm start.
- Performs `hkpWorld_LinearCast` from start to `start + castDirection * nearCastDistance`.
- Filters hits:
  - requires a rigid body and `m_userData`;
  - requires a non-player reference or the other hand's weapon when in offhand two-hand mode;
  - ignores disabled-collision bodies via bit 14;
  - only accepts selectable refs or the other hand's grabbable selected object;
  - ignores mid-flight projectiles by requiring impact data;
  - for the other hand's weapon path, requires weapon collision to be enabled.
- Prioritizes by perpendicular distance from hit point to the cast ray, not by COM distance.
- If no normal close object is found and a pulled object exists, it widens the sphere to `widePullGrabRadius` and uses `hkpWorld_GetPenetrations` against the pulled object as a catch fallback.

`Hand::FindFarObject(...)`:

- First raycasts to the far target to clip the linear cast at walls/occluders.
- Then linear-casts the sphere to the clipped point with filter `0x2C` and radius `farCastRadius`.
- Filters:
  - excludes the other hand's exclusive object;
  - requires selectable refs;
  - ignores mid-flight projectiles;
  - gates actor/loot targets through looting config;
  - requires the hit point to be within the HMD forward cone using `requiredCastDotProduct`;
  - prioritizes perpendicular ray distance, same as close selection.

`Hand::GetRigidBodyToGrabBasedOnGeometry(...)`:

- Builds current skinned and static object triangles.
- Applies any adjusted/initial object transform before geometry testing.
- Finds the closest point on graphics geometry to the palm line.
- For static triangle hits, walks to the closest parent with moveable collision and returns that rigid body.
- For skinned triangle hits:
  - maps triangle vertices back through partition vertex maps;
  - weights each contributing bone by skin weight divided by distance to the grabbed triangle point;
  - chooses the highest accumulated bone;
  - walks to closest parent with moveable collision and returns that rigid body.

Interpretation:

- HIGGS body choice is contact/mesh/skin driven.
- It is not COM based.
- This function is important for multipart objects/weapons because it avoids constraining a generic root when the actual contacted part belongs to a different body.

### HIGGS state transitions: dynamic scope only

Relevant dynamic states found in `Hand::Update(...)` and `Hand::TransitionHeld(...)`:

- `Idle`
- selected close/far object
- `SelectionLocked`
- `Pulled`
- `PreGrabItem` / `PrePullItem` only as spawned-item handoff scaffolding
- `HeldInit`
- `HeldBody`
- release/drop back to `Idle`

Pull/converge behavior:

- When selected far/locked and the controller moves toward the selected point faster than `pullSpeedThreshold`, HIGGS enters `Pulled`.
- Pull setup:
  - records pulled object handle/body;
  - saves angular damping;
  - sets angular damping to `pulledAngularDamping` default `8.0f` to prevent spin-out;
  - converts keyframed/projectile/fixed-ish bodies to dynamic where needed;
  - computes a pull duration;
  - stores `pulledPointOffset` from COM to the selected point for pull targeting only.
- During early `Pulled`, HIGGS applies predicted velocity for `pullApplyVelocityTime`, and tracks hand target only during `pullTrackHandTime`.
- Close catch uses `FindCloseObject`; when the pulled object is near enough, `TransitionHeld(...)` takes over.

Held transition:

- `TransitionHeld(...)` prepares object state, mesh/triangle state, desired transform, finger pose, connected bodies, constraint, inertia, listeners, and state.
- Dynamic behavior enters `HeldBody` when `ShouldUsePhysicsBasedGrab(...)` returns true.
- In the production HIGGS config under this task, `ForcePhysicsGrab=1`, so ordinary grabs resolve to this dynamic path.

Release:

- On release from `HeldBody`, HIGGS removes the grab constraint from the world and from the hand body.
- It restores contact listener callback delay and saved inverse inertia for every connected body unless the other hand still owns that connected component.
- It sets release velocity using held-object velocity history and controller-derived components.
- COM is used for tangential throw velocity only, as release lever data, not as grip pivot.

### HIGGS pivot and reference-frame pipeline

Confirmed in `TransitionHeld(...)`:

- `closestPoint` arrives in Havok meters and becomes `ptPos = closestPoint / havokWorldScale`.
- If graphics geometry is available, HIGGS replaces the collision point with the closest graphics triangle point `triPos`.
- `ptPos` is the object-side grip/contact point.
- `palmPos` is the hand-side palm point.
- `desiredNodeTransform` starts as the current/adjusted object transform, preserving object rotation.
- Translation is applied as:
  - `desiredNodeTransform.pos += palmPos - ptPos`
- This seats the selected object point into the palm without rotating the object around COM.
- `desiredNodeTransformHandSpace = inverseHand * desiredNodeTransform`.

Constraint construction:

- `hkPivotA = palmPos * havokWorldScale`.
- `hkPivotB = ptPos * havokWorldScale`.
- `pivotA` is transformed into body A local space using the hand body transform.
- `pivotB` is transformed into body B local space using the grabbed body transform.
- `handTransformHandSpace.pos = pivotA`.
- `desiredHavokTransformHandSpace = desiredNodeTransformHandSpace * GetRigidBodyTLocalTransform(selectedObject.rigidBody)`.
- `handTransformObjSpace = InverseTransform(desiredHavokTransformHandSpace)`.
- `handTransformObjSpace.pos = pivotB`.
- `CreateGrabConstraint(bodyA, bodyB, handTransformHandSpace, handTransformObjSpace)` creates the actual dynamic drive.

Held update:

- HIGGS computes `heldTransform = collidableNode->m_worldTransform`.
- Visual hand target is `m_adjustedHandTransform = heldTransform * inverse(desiredNodeTransformHandSpace)`.
- For the constraint:
  - `desiredTransformHandSpace = desiredNodeTransformHandSpace * GetRigidBodyTLocalTransform(selectedObject.rigidBody)`;
  - `desiredHandTransformHavokObjSpace = InverseTransform(desiredTransformHandSpace)`;
  - angular target is `desiredHandTransformHavokObjSpace.rot`;
  - transform B translation is recomputed from:
    - `(desiredHandTransformHavokObjSpace * palmPosHandspace) * collidableNode->m_worldTransform.scale * havokWorldScale`.

Interpretation:

- The initial pivot B starts from the selected object contact point.
- The per-frame transform-B translation is not blindly frozen to COM or to a static body-local point. It is recomputed from the frozen hand/object reference relation so the linear and angular goals remain coherent as the desired hand/object frame changes.
- COM is absent from this pivot/target/orientation pipeline.
- COM appears later for release tangential velocity and mass/inertia behavior only.

### HIGGS custom constraint and motor pipeline

`CreateGrabConstraint(...)` in `src\RE\havok.cpp`:

- Creates `hkConstraintCinfo`.
- Sets priority to `PRIORITY_TOI`.
- Sets rigid body A to the hand body and body B to the grabbed body.
- Allocates `GrabConstraintData`.
- Calls `setInBodySpace(transformA, transformB)`.
- Constructs `bhkGroupConstraint`.
- Sets collision group from body B.
- Calls `setTargetRelativeOrientationOfBodies(...)`.
- Calls `setMotorsActive(..., true)`.

`GrabConstraintData` in `constraint.cpp/.h`:

- Atom chain:
  - set local transforms;
  - setup stabilization;
  - ragdoll motor atom;
  - three linear motor atoms.
- Solver results:
  - 3 angular motor solver results;
  - 3 linear motor solver results;
  - `SOLVER_RESULT_MAX = 6` in Skyrim/HIGGS.
- Runtime:
  - angular initialized bytes;
  - angular previous target angles;
  - linear initialized bytes;
  - linear previous target positions.
- `getRuntimeInfo` reports `Runtime::getSizeOfExternalRuntime()`, which is `sizeof(Runtime) * 2` because HIGGS observed crashes with exact runtime size/aligned runtime size.
- `addInstance` zeroes runtime.
- `setMotorsActive(false)` clears solver result history.

Important FO4VR comparison:

- HIGGS runtime count/offsets are hkp/Skyrim-specific and are not directly portable.
- FO4VR Ghidra already corrected ROCK's current atom runtime interpretation: ROCK's 12 solver results are coherent for its current FO4VR atom stream, and the current-runtime-cursor offsets avoid overlap.
- HIGGS still matters as behavior architecture: custom linear + angular motors with finite max force and per-frame motor field updates.

### HIGGS mass, inertia, finite-force, collision, and deviation behavior

HIGGS dynamic grab is not "strong spring to hand". It uses several layers together.

Per-frame non-actor motor values in `HeldBody`:

- `isWeapon = baseForm->formType == kFormType_Weapon`.
- Linear max force:
  - normal object: `grabConstraintLinearMaxForce`, default `2000`.
  - loose weapon: `grabConstraintLinearMaxForceWeapon`, default `9000`.
- Angular max force:
  - `linearMotor->m_maxForce / angularToLinearForceRatio`.
  - default ratio `12.5`.
- Startup angular fade:
  - if `fadeInGrabConstraint`, angular ratio lerps from `grabConstraintFadeInStartAngularMaxForceRatio` default `100` down to `12.5` over `grabConstraintFadeInTime` default `0.1`.
  - This deliberately makes angular authority weak at startup while the object seats into the hand.
- Collision state:
  - contact listeners on connected bodies feed `EntityCollisionListener::IsColliding()`.
  - if colliding, angular and linear tau targets become `grabConstraintCollidingAngularTau` and `grabConstraintCollidingLinearTau`, both default `0.01`.
  - otherwise tau targets are normal `grabConstraintAngularTau` and `grabConstraintLinearTau`, both default `0.03`.
  - tau changes through `AdvanceFloat(..., grabConstraintTauLerpSpeed)`, default speed `0.5`, so it does not snap.
- Mass cap:
  - final linear force is capped to `mass * grabConstraintMaxForceToMassRatio`, default ratio `500`.
  - angular force is capped again from the capped linear force and angular ratio.
- Min forces are mirrored negative max forces.
- Recovery/damping are set every frame:
  - angular proportional recovery default `2`;
  - angular constant recovery default `1`;
  - angular damping default `0.8`;
  - linear proportional recovery default `2`;
  - linear constant recovery default `1`;
  - linear damping default `0.8`.

Loose weapon interpretation:

- HIGGS loose weapons are still ordinary dynamic held objects in this branch.
- Loose weapon special treatment is not equipped weapon logic.
- The only confirmed loose-weapon dynamic motor distinction in this source pass is a higher base linear max force before mass cap.
- Because the mass cap still applies, weapon force is not infinite:
  - with default ratio `500`, a weapon under 18 mass caps below the `9000` weapon force.
  - angular force then derives from that capped linear force.
- Long-object lever effects are not an explicit separate weapon branch in the inspected HIGGS code. They emerge from:
  - finite angular max force;
  - inertia tensor normalization;
  - grip-to-COM geometry in release tangential velocity;
  - solver effective mass around the selected pivot.

Inertia normalization:

- On grab, HIGGS collects all connected grabbed rigid bodies.
- For each connected body:
  - saves contact point callback delay and sets it to 0 for immediate collision listener feedback;
  - saves inverse inertia;
  - clamps each inverse-inertia axis so it is no more than `grabbedObjectMaxInertiaRatio` default `10` times the minimum inverse-inertia axis;
  - enforces minimum inertia through `grabbedObjectMinInertia` default `0.01`.
- On release, HIGGS restores saved inverse inertia unless the other hand still owns the connected component.

Deviation and visual lag:

- HIGGS does not force the real hand to perfectly match the held object.
- It computes a visual/adjusted hand transform from the live held object:
  - `adjustedHand = heldObjectWorld * inverse(desiredNodeTransformHandSpace)`.
- At grab start, it lerps from real hand transform to adjusted hand transform. Duration is distance-based between:
  - `physicsGrabLerpHandTimeMin` default `0.1`;
  - `physicsGrabLerpHandTimeMax` default `0.2`.
- It tracks five frames of hand deviation:
  - current deviation = distance between adjusted/visual hand and real/controller hand.
  - average deviation over five samples is used.
- It ignores hand distance briefly after grab/sneak transition:
  - `physicsGrabIgnoreHandDistanceTime` default `0.2`;
  - `sneakUnsneakIgnoreHandDistanceTime` default `0.1`.
- If average deviation exceeds `maxHandDistance` after the ignore windows, HIGGS releases/drops instead of applying unlimited force.

Mass effects outside the constraint:

- `RegisterObjectMass(...)` accumulates unique held masses each frame.
- `UpdateSpeedReduction()` slows player movement from total held mass using:
  - `slowMovementMassProportion`;
  - `slowMovementMassExponent`;
  - `slowMovementMaxReduction`;
  - fade-out time.
- Jump height is also reduced from total held mass in the jump hook.
- Player-space compensation registers held/connected/contained bodies so player locomotion is added consistently to them instead of making the object lag behind room movement.

Interpretation:

- HIGGS weight feel is a system, not a single COM trick.
- The core held-object "not Superman" behavior comes from finite force caps, mass cap, angular force ratio, weak angular startup, collision tau softening, inertia normalization, deviation/visual lag, and movement/jump penalties.
- COM is not used as grip target. COM is only weight/release data.

### HIGGS hand posing and finger blend

Confirmed dynamic hand pose setup in `TransitionHeld(...)`:

- HIGGS builds/skinning object triangles at grab time:
  - skinned triangles from partitions and bone transforms;
  - static triangles from geometry;
  - filtered by blacklists, hair, soft/decal/blood flags, and optional vertex-alpha cutoff.
- It finds the closest point on graphics geometry to the palm line.
- It filters nearby triangles around the selected point.
- It computes `palmToPoint = ptPos - palmPos`.
- For each finger:
  - reads hand-space normal, zero-angle vector, and start position from generated finger curve data;
  - flips left-hand axes as needed;
  - transforms those vectors/points through the live hand transform;
  - offsets `startFingerPos += palmToPoint` so the test is performed as if the hand has already seated onto the object point.
- `GetIntersections(...)` tests each finger against nearby triangles using precomputed inner/outer/tip finger curves.
- The intersection logic:
  - slices triangle edges against the finger plane;
  - converts edge intersections into angular/radial finger-curve space;
  - tests inner, outer, and tip curves;
  - handles negative/behind-finger hits;
  - chooses the largest curve value, i.e. the least-curling valid hit.
- Thumb has an alternate curve (`index 5`) if the standard thumb curve misses or produces a negative angle.
- Result rules:
  - negative angle opens the finger;
  - positive curve value is used directly;
  - no intersection closes the finger;
  - minimum finger value is clamped to `0.2` to avoid overcurl;
  - no graphics point falls back to `0.9` for all fingers.

`FingerAnimator` behavior:

- Finger animation stores target values in `grabbedFingerValues[5]`.
- It uses precomputed open/closed local positions and rotations from `finger_curves.cpp`.
- It lerps/slerps local transforms for three joints per finger.
- It advances toward target at configured linear/angular speeds:
  - grab default `fingerAnimateGrabLinearSpeed = 4`;
  - grab default `fingerAnimateGrabAngularSpeed = 630`;
  - start/end speeds are separate.
- It restores fingers when they stop being set.

Important conclusion:

- HIGGS hand posing is math/geometry driven.
- It does not require HIGGS_R/L mesh-specific grip nodes for ordinary dynamic grab.
- Optional HIGGS grab nodes and attach transforms can override initial object transform for specific objects, but the generic pose-quality path is:
  - live hand frame;
  - object graphics triangles;
  - generated finger curves;
  - finger-plane/curve intersection.

### ROCK parity checkpoint from source after this HIGGS pass

ROCK currently has equivalents for many HIGGS dynamic pieces:

- contact/mesh/palm evidence and a three-phase pocket model;
- non-COM captured grip point;
- body-frame frozen pivot;
- raw hand frame and generated hand-body frame split;
- hand-relative visual relation;
- visual hand deviation guard;
- nearby grab damping;
- inertia normalization and restore across unique motion slots;
- connected held body set handling;
- custom constraint atom chain;
- finite-force motor controller:
  - mass cap;
  - angular-to-linear ratio;
  - collision tau;
  - tau lerp;
  - startup angular fade;
  - loose weapon multiplier surfaces;
- release velocity using object/hand history plus COM only for tangential release.

ROCK current gaps relative to HIGGS dynamic behavior:

- Ordinary one-hand loose object and loose non-equipped weapon grabs still use `HeldObjectDriveMode::NativeMouseSpring`.
- `GrabMotionController` is only fed into `updateConstraintGrabDriveMotors(...)`, and that path only runs for `SharedConstraint`.
- The native one-hand path queues target frames and adaptive lead, but it does not apply HIGGS-style per-frame finite linear/angular motor caps to the held body.
- Native one-hand mouse spring has scale knobs, but it does not expose the full HIGGS policy surface:
  - linear force max;
  - angular force max derived from linear;
  - per-frame mass cap;
  - collision tau softening;
  - finite angular startup fade;
  - per-axis motor recovery fields.
- `PhysicsStepDriveCoordinator` has a verified `betweenCollideAndSolve` vtable slot at `+0x28`, but the callback is currently no-op and the public callback interface only supports whole-pre-step and substep-pre-collide.
- Shared custom constraint target/motor writes currently occur from held-object update flow, not from the verified after-collide/before-solve slot.
- ROCK body-A design is still unsettled:
  - HIGGS uses the generated hand collision body as body A and disables collision while held.
  - ROCK's generated palm anchor is root-flattened and valid as a hand frame source, but it has generated-collider lifecycle/rebuild responsibilities.
  - A separate no-contact authority body may be better in FO4VR, but this is an FO4VR design choice, not a HIGGS requirement.
- Loose weapon behavior in ROCK should remain separate from equipped/two-hand weapon behavior. HIGGS source confirms loose weapons are treated in ordinary dynamic held-object logic, not equipped weapon grip logic.

### Corrected explanation for why native mouse spring feels smoother than old custom motors

This explanation now combines FO4VR Ghidra findings and HIGGS source behavior:

- Native mouse spring is smoother because ROCK flushes it from a physics-step boundary and the native hknp action internally has dt-aware smoothing/caps/deadband-like behavior.
- The old custom motor path is likely not failing because of current ROCK atom runtime offset overlap; that earlier suspicion was corrected by FO4VR switch-table and range-disassembly evidence.
- The remaining likely old-custom stutter causes are systemic:
  - target/constraint fields updated from game update rather than the solver-safe phase;
  - body A drive and constraint target not committed in the same phase;
  - generated body drive using `DriveToKeyFrame`, which has a verified cap-triggered transform snap path;
  - contact palm body lifecycle/rebuild behavior interacting with constraint ownership;
  - ordinary one-hand path not using the finite-force motor controller, so tuning was never exercised in the real path;
  - possible competing authority during promotion/destroy transitions.

### FO4VR proxy drive follow-up: direct hard-keyframe velocity versus DriveToKeyFrame

This section records the post-compaction Ghidra pass on FO4VR-native ways to drive a future no-contact grab authority proxy. This is not an implementation decision. It narrows which native path is safer to keep researching if ROCK replaces mouse-spring authority with a custom body-A plus linear/angular motor constraint.

Confirmed functions:

- `0x14153ABD0`:
  - locks the world through `world + 0x690`;
  - calls `0x14153A6A0` to compute linear and angular velocity from current body transform to a target position/rotation over the supplied inverse timestep;
  - calls `0x141539F30` to write those velocities to the body's motion;
  - unlocks the world.
- `0x14153A6A0`:
  - computes hard-keyframe linear and angular velocity for a body id and target transform;
  - reads the body from `world + 0x20 + bodyId * 0x90`;
  - reads the motion from `world + 0xE0 + motionIndex * 0x80`;
  - compares current body/motion transform against the target position/quaternion and writes velocity outputs;
  - does not visibly perform a transform snap in this function.
- `0x141539F30`:
  - writes linear velocity to motion `+0x40`;
  - writes angular velocity to motion `+0x50`;
  - wakes/activates the body through `0x141546EF0` when the body flags require activation;
  - calls `0x14153D440`;
  - notifies the world listener/list at `world + 0x538` through `0x14153EE00`.
- `0x141E086E0`:
  - is the DriveToKeyFrame-style path used by wrapper-level keyframe driving;
  - resolves world/body through wrapper helpers;
  - calls `0x14153A6A0` to compute hard-keyframe velocities;
  - reads motion properties from the table at `world + 0x5D0`, indexed by the motion-properties id at motion `+0x38`, stride `0x40`;
  - checks max linear velocity at motion-properties `+0x10`;
  - checks max angular velocity at motion-properties `+0x14`;
  - if either computed velocity exceeds the caps, calls `0x141DF55F0(world, bodyId, targetTransform, 1)` and then applies zero velocities;
  - otherwise applies the computed velocities through `0x141DF56F0`.

Interpretation:

- `0x14153ABD0` is the cleaner direct velocity-authority candidate because the inspected path computes and writes hard-keyframe velocity without the visible cap-triggered transform snap used by `0x141E086E0`.
- `0x141E086E0` is still useful for understanding existing generated collider drive behavior, but its cap-triggered `0x141DF55F0` branch is a concrete stutter/snap risk if reused unchanged for a grab authority proxy.
- This supports the current research direction: if ROCK creates a dedicated no-contact body-A proxy, the proxy drive should be studied around direct velocity writes and solver-phase timing rather than blindly copying generated collider `DriveToKeyFrame`.
- This does not mean transform correction should never exist. It means correction needs to be explicit, bounded, and coordinated with the custom constraint target/motor update, not hidden inside a wrapper cap branch.

Still unresolved:

Follow-up confirmed:

- `0x141DF55F0`:
  - is a thread-aware wrapper around direct transform set `0x1415395E0`;
  - if TLS flag `+0x1528` says direct execution is allowed, calls `0x1415395E0(world, bodyId, targetTransform, flag)`;
  - otherwise queues a command through `0x141DFC8D0` with command-like local header bytes `0x50 / 0x60100`.
- `0x1415395E0`:
  - locks the world;
  - validates broadphase/body mappings;
  - compares current transform against target transform;
  - writes the body transform matrix directly into the body slot at `world + 0x20 + bodyId * 0x90`;
  - calls `0x14153CC40(...)`;
  - fires transform-change style notifications through the listener list at `world + 0x538`;
  - unlocks the world.
- `0x141DF56F0`:
  - is a thread-aware wrapper around velocity write `0x141539F30`;
  - if direct execution is allowed, calls `0x141539F30(world, bodyId, linearVelocity, angularVelocity)`;
  - otherwise queues a command through `0x141DFC8D0` with command-like local header bytes `0x30 / 0x90100`;
  - after the write/queue, compares requested linear and angular velocities against zero thresholds and calls `0x141DF60C0(world, bodyId)` if either is non-zero.
- `0x141539F30`:
  - locks the world and resolves the body and motion;
  - only writes when body flags include dynamic/motion-backed bit `0x02`;
  - compares requested velocities against current motion velocities;
  - wakes/activates the body through `0x141546EF0` when body flags do not include the no-activation bits tested by `(flags & 9)`;
  - writes linear velocity into motion `+0x40`;
  - writes angular velocity into motion `+0x50`;
  - calls `0x14153D440(...)`;
  - notifies listeners at `world + 0x538` through `0x14153EE00`.
- `0x14153D440`:
  - iterates a body chain using body offset `+0x64`;
  - calls `0x1417D3BF0(...)` for each body in that chain using the velocity/motion pointer and world data at `+0x1E0`, `+0x330`, and `+0x180`;
  - likely updates broadphase/motion-derived body state after velocity changes, but exact semantic name remains unresolved.
- `0x141546EF0`:
  - locks the world;
  - checks the body slot at `world + 0x20 + bodyId * 0x90`;
  - if body `+0x6C` is not `-1`, calls `0x1417D8590(*(world + 0x4A0), bodyId)`;
  - likely wakes or activates the body/island. Exact side effects inside `0x1417D8590` remain unresolved.
- `0x141DF60C0`:
  - validates body id, flags, and body `+0x6C`;
  - if the body is not already in the desired active path, checks body flags derived from `body + 0x40`;
  - for a specific high flag bit, enters a world-side queue/critical region at `world + 0x6E8` / `world + 0x708`;
  - likely schedules activation/island work for bodies with non-zero target velocity. Exact queue semantics remain unresolved.

Caller context:

- `0x141E4AA30`, the native mouse-spring action update, calls:
  - `0x14153A6A0` to compute velocity to its current target;
  - `0x141539F30` to write linear/angular velocities;
  - additional velocity/angular-impulse style helpers after damping/capping.
- `0x141E086E0`, the DriveToKeyFrame-style wrapper, is called by:
  - `0x141E18F90`, a recursive collision-object scene/update path;
  - `bhkNPCollisionObject::vfunction44` around `0x141E09AB7`, where generated collision object transforms are driven and can fall through to `0x141DF55F0`.
- `0x141DF5930` is a separate wrapper that calls:
  - `0x14153A6A0` to compute hard-keyframe velocity;
  - `0x141539F30` or the queued equivalent to apply velocity;
  - `0x141DF60C0` if the requested velocity is non-zero;
  - it does not contain the motion-property cap branch or transform-snap branch found in `0x141E086E0`.
- `0x141DF5930` is called by:
  - `bhkNPCollisionObject::vfunction44`;
  - `0x1412CB4E0`, a camera/update path that drives a body by velocity when global timestep `DAT_1465A3D84` is positive.

Updated interpretation:

- FO4VR has both a snap-cap keyframe wrapper and a compute-velocity keyframe wrapper.
- The generated collision-object path can use both:
  - direct velocity drive through `0x141DF5930`;
  - direct transform set through `0x141DF55F0`;
  - cap-triggered transform set through `0x141E086E0`.
- Native mouse spring smoothness is not because it is magic or because it owns the correct grab model. It is smoother because its update does dt-aware target math and then writes velocities through the hknp motion path, with damping/capping before the write.
- A future custom motor body-A proxy should therefore research using `0x141DF5930` / `0x14153A6A0 + 0x141539F30` semantics for velocity drive, not `0x141E086E0` as-is.
- If transform snapping is needed for large correction or teleport recovery, it should be a deliberate state transition outside steady held-grab solve, not an implicit cap branch inside the per-frame drive.

Remaining unresolved:

- `0x1417D8590` needs direct inspection before relying on the exact wake/island semantics of `0x141546EF0`.
- `0x14153CC40` needs inspection before fully naming the transform-set side effects of `0x1415395E0`.
- `0x1417D3BF0` needs inspection before fully naming the post-velocity chain update in `0x14153D440`.
- `0x141DFC8D0` and the command headers need inspection if ROCK ever calls these wrappers from non-physics threads instead of an already-safe physics step callback.

Second follow-up confirmed:

- `0x1417D8590`:
  - receives a manager pointer and body id;
  - reads the world through `*param_1`;
  - resolves the body slot from `world + 0x20 + bodyId * 0x90`;
  - skips when body flag bit `0x01` is set;
  - for ordinary bodies without body flag bit `0x08`, walks a linked island/body list through a table at `param_1[3]`;
  - marks entries at offset `+0x59` and pushes their ids into an array at `param_1[0xB]`, growing it through `0x14155D820` if needed;
  - for bodies with flag bit `0x08`, uses motion index `body + 0x68` and writes a byte at a motion-manager slot `param_1[1] + motionIndex * 0x10 + 0x0B`;
  - this supports naming it as a body/island activation or dirty-island enqueue helper, not a general transform setter.
- `0x14153CC40`:
  - is called by direct transform set `0x1415395E0`;
  - for static/no-motion bodies, updates broadphase/AABB state and fires/listens through world offsets including `+0x510`, `+0x178`, and `+0x4A0`;
  - computes packed AABB/min-max state from shape bounds and world scale data;
  - calls broadphase update methods at `world + 0x178` vtable slots including `+0x18` and `+0x50`;
  - may wake overlapping bodies through `0x1417D8590` depending on `param_5` and AABB overlap changes;
  - for motion-backed bodies, updates motion transform/body bounds through `0x1417D51E0` and `0x14153CB10`, then wakes the body through `0x1417D8590`.
- `0x1417D3BF0`:
  - is called by `0x14153D440` after velocity writes;
  - recomputes body extents/AABB-like packed values at body offsets around `+0x50..+0x76`;
  - uses shape/body transform callbacks and world scale/bounds data;
  - supports naming `0x14153D440` as velocity-postupdate body-chain bounds refresh, not just a listener notification.
- `0x141DFC8D0`:
  - is a thread-local command queue/enqueue function;
  - switches on a short command id read from the caller-provided command header;
  - stores command payloads in per-thread arrays at `DAT_1465A3E30 + threadIndex * 0xA8 + offsets`;
  - recognizes command ids including `6`, `7`, `9`, `0x12`, and `0x2C`;
  - uses `0x141DFD7A0` / `0x141E04930` to register or synchronize against the target world key;
  - command headers seen so far:
    - transform set wrapper `0x141DF55F0`: local bytes `0x50 / 0x60100`;
    - velocity wrapper `0x141DF56F0` and hard-keyframe velocity wrapper `0x141DF5930`: local bytes `0x30 / 0x90100`.

Updated side-effect interpretation:

- Direct transform set is expensive and broadphase-visible:
  - writes body transform;
  - updates bounds/AABB state;
  - may wake nearby/overlapping bodies;
  - notifies transform listeners.
- Direct velocity drive is solver/motion-friendly:
  - writes linear/angular motion velocity;
  - refreshes body-chain bounds;
  - wakes the body/island if the requested velocity is non-zero;
  - avoids steady-state transform snapping when used through `0x141DF5930` or `0x14153ABD0`.
- For a future custom dynamic grab proxy, the cleaner steady-state drive candidate remains:
  - calculate proxy target from the flattened real hand frame;
  - commit proxy velocity through direct hard-keyframe velocity semantics;
  - commit constraint target and motor fields in the same after-collide/before-solve phase;
  - reserve direct transform set for create/teleport/recovery transitions only.
- If ROCK updates body A through direct transform set every frame, it risks broadphase wake churn and implicit snap-like correction. That is consistent with the old "wrist-breaking" and stutter failure mode when combined with a custom constraint on body B.

New unresolved items:

- Need verify whether `0x141DF5930` can be called safely from the physics listener phase ROCK controls, or whether direct `0x14153A6A0 + 0x141539F30` is preferable inside an already-locked/safe context.
- Need identify the exact native caller/phase for `bhkNPCollisionObject::vfunction44` relative to physics step. If it runs outside the desired between-collide-and-solve phase, copying that call site timing is not sufficient for custom grab.
- Need map the filter/layer path for creating a no-contact proxy body in FO4VR, because body-A should be an authority anchor, not contact evidence.

### FO4VR proxy filter/layer follow-up

This section verifies the collision-filter write path that a future hidden authority proxy would use if it is created through the current `BethesdaPhysicsBody` lifecycle.

ROCK source facts reconfirmed:

- Current generated layers:
  - `ROCK_LAYER_HAND = 43`;
  - `ROCK_LAYER_WEAPON = 44`;
  - `ROCK_LAYER_BODY = 47`;
  - `ROCK_LAYER_RELOAD = ROCK_LAYER_HAND`.
- Vanilla configured layer names stop at `46`, but FO4VR's hknp matrix is addressable through 64 rows.
- `ROCK_LAYER_EXTENDED_FIRST = ROCK_LAYER_BODY`, `ROCK_LAYER_EXTENDED_LAST = 63`.
- `isRockOwnedReusableLayer()` currently returns only `43`, `44`, and `47`.
- `applyRockGeneratedLayerPolicies()` currently writes only hand/reload, weapon, and body rows.
- Active grab classification rejects:
  - ROCK hand bodies;
  - ROCK generated bodies;
  - ROCK weapon source bodies;
  - player/character-controller bodies;
  - actor/biped layers for ordinary active grab;
  - `FO4_LAYER_NONCOLLIDABLE = 15`;
  - unsupported non-dynamic prop layers.

Ghidra-confirmed filter-info write path:

- `0x141DF5B80`:
  - thin wrapper;
  - calls `0x14153AF00(world, bodyId, filterInfo, rebuildMode)`.
- `0x14153AF00`:
  - locks world `+0x690`;
  - resolves body slot from `world + 0x20 + bodyId * 0x90`;
  - writes collision filter info to body offset `+0x44`;
  - if `rebuildMode == 0` and body `+0x6C` is not `-1`, calls `0x14153C5A0(world, bodyId)`;
  - notifies world listener list `world + 0x538` through `0x14153EE00`.
- `bhkNPCollisionObject::vfunction51` at `0x141E08EC5`:
  - resolves the body id from the collision object's physics system;
  - calls `0x141DF5B80(nativeWorld, bodyId, newFilterInfo, 0)`.
- `0x14153C5A0`:
  - handles the expensive rebuild/dirty path after filter changes;
  - for non-motion bodies calls `0x141547050`;
  - for dynamic eligible bodies calls `0x141546EF0` activation/dirty-island helper;
  - clears packed AABB/broadphase state in the table at `world + 0x38`;
  - marks body flags with `|= 0x400`.

Interpretation:

- Filter info is the authoritative per-body value at hknp body offset `+0x44`.
- Changing a proxy's filter after creation can be more expensive than choosing the right filter at creation, because rebuild mode `0` invalidates broadphase/body state and can wake/dirty islands.
- `BethesdaPhysicsBody::create(...)` already sets the requested filter immediately after add-to-world using `havok_runtime::setFilterInfo(world, bodyId, filterInfo, 1)`, which writes/notifies without the `0x14153C5A0` rebuild branch.
- Later suppression/restoration paths often use the default rebuild mode `0`, which is correct for real contact bodies but is a cost/risk to avoid for a hidden proxy during steady held state.

Proxy layer/filter implications:

- A hidden body-A authority proxy should be born already non-contact if possible.
- `FO4_LAYER_NONCOLLIDABLE = 15` remains a viable low-scaffolding candidate because:
  - current active-grab classification rejects it;
  - current ROCK hand/weapon masks exclude it;
  - it avoids extending the ROCK-owned layer namespace.
- A dedicated extended proxy layer in `48..63` remains architecturally cleaner if ROCK wants explicit ownership/logging/watchdog control, but it is not free:
  - add a named layer constant;
  - extend reusable-layer policy;
  - write the row/column to zero or an explicit expected mask;
  - extend watchdog validation;
  - extend classifier rejection/debug reporting;
  - ensure generated contact suppression ignores it as a hidden authority body.
- The current user layer clarification fits the source:
  - layer `43` for hands/reload tools;
  - layer `44` for guns/weapons;
  - layer `47` for generated body/full-body;
  - rows above those are technically usable only after ROCK owns and applies the full matrix/policy path.

Still unresolved:

- The exact hknp filter function that tests the no-collide suppression bit `1 << 14` has not been re-found in this pass. ROCK source documents it as already Ghidra-confirmed, but this active pass has only reconfirmed the write path to body `+0x44`.
- Need a separate blind verification wave if the implementation plan depends on filter bit `14` rather than layer-row no-contact.
- Need decide later whether proxy non-contact should be achieved by:
  - layer `15`;
  - a dedicated extended zero-row layer;
  - filter bit `14`;
  - or a combination at creation time only.

### Research conclusion from this pass

The next custom-authority design should be judged against these confirmed HIGGS rules:

- Preserve object rotation unless an explicit authored grab transform is selected.
- Seat selected contact/mesh point into palm/pocket by translation, not COM.
- Use body A as a real-hand/root-frame-aligned keyframed authority.
- Disable body-A collision while held or use a no-contact authority body that has the same effect.
- Drive body B through one custom linear/angular motor constraint.
- Update angular target and transform-B translation from the same captured hand/object frame every held update.
- Feed finite motor fields every frame:
  - linear max force;
  - angular max force derived from linear;
  - mass cap;
  - collision tau;
  - startup angular fade;
  - recovery/damping.
- Normalize/restore held connected-body inertia.
- Track visual hand deviation and release instead of applying unlimited authority.
- Use COM only for mass/inertia/release/tangential velocity data.
- Keep loose non-equipped weapon handling in the loose-object dynamic path, with weapon-specific force policy but without equipped weapon grip assumptions.

### Open questions after this pass

- FO4VR-specific: should custom target/motor writes happen in `betweenCollideAndSolve` rather than game update? Current evidence strongly suggests yes, but the implementation plan still needs a concrete callback API shape.
- FO4VR-specific: should body A be the existing palm anchor with held collision disabled, or a new dedicated no-contact authority proxy rooted in the same flattened hand frame?
- FO4VR-specific: if a new proxy is used, should it be driven by direct hard-keyframe velocity (`ApplyHardKeyFrame`/direct velocity path) instead of generated-collider `DriveToKeyFrame`?
- FO4VR-specific: does the current generated palm anchor rebuild deferral (`isConstrained`) create any stale body-frame issue if it becomes ordinary one-hand constraint body A?
- HIGGS behavior extension: loose weapon long-axis / grip-to-COM lever length is not an explicit dynamic motor branch in inspected HIGGS. ROCK may be able to surpass HIGGS by deriving angular force scaling from grip-to-COM/long-axis inertia, but that must be treated as a new FO4VR design, not a copied HIGGS fact.
- HIGGS timing: HIGGS updates constraint params in its normal update loop on Skyrim hkp. FO4VR hknp step ordering may require a stricter solver-phase update to avoid stutter.
