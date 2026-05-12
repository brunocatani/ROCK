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

### FO4VR physics listener command-safety follow-up

This section maps whether future proxy/constraint writes from a physics-step callback should use native wrappers, direct world functions, or command queues.

Ghidra-confirmed `bhkWorld::vfunction43` / update facts:

- Function address inspected: `0x141DF73A0`.
- The update pushes timer labels for:
  - `TtPhysicsStepListeners-BeforeWholePhysicsUpdate`;
  - `TtPhysicsStepListeners-BeforeAnyPhysicsStep`;
  - `TtCollide`;
  - `TtPhysicsStepListeners-BetweenPhysicsCollideAndSolve`;
  - `TtSolve`;
  - `TtPhysicsStepListeners-AfterAnyPhysicsStep`;
  - `TtPhysicsStepListeners-AfterWholePhysicsUpdate`.
- For each listener phase, FO4VR:
  - locks/copies the bhkWorld listener array;
  - calls the listener vtable slot;
  - calls a phase companion slot;
  - drains a command list associated with `local_128` through a phase helper.

Phase callback/flush pattern:

- Before whole:
  - listener slot `+0x08`;
  - command drain helper `0x141DFDB30`;
  - companion slot `+0x10`.
- Before any substep:
  - listener slot `+0x18`;
  - command drain helper `0x141DFE3B0`;
  - companion slot `+0x20`.
- Between collide and solve:
  - listener slot `+0x28`;
  - command drain helper `0x141DFDE70`;
  - companion slot `+0x30`.
- After any substep:
  - listener slot `+0x38`;
  - command drain helper `0x141DFDCD0`;
  - companion slot `+0x40`.
- After whole:
  - listener slot `+0x48`;
  - command drain helper `0x141DFD990`;
  - companion slot `+0x50`.

Command drain helper findings:

- `0x141DFDB30`, `0x141DFE3B0`, `0x141DFDE70`, and `0x141DFDCD0` decompile to the same command-drain shape.
- Each checks command count at `param_1[1]`.
- If one command is present and a small command field equals `1`, it calls the command object's vtable `+0x20` directly.
- Otherwise it builds command batches through `0x141DFFBB0`, schedules work through `0x141BCFE80` / `0x141BCFE60`, waits on a global handle at `DAT_145A3CD30 + 0x38`, and releases refs.
- This means commands queued by wrappers during a listener callback are not simply "later someday"; the world update has explicit phase drains immediately after callbacks.

World lock helper findings:

- `0x141DF5FB0(worldLike)` calls `0x141B93330(worldLike + 0x6D8)`.
- `0x141DF5FD0(worldLike)` calls `0x141B93580(worldLike + 0x6D8)`.
- `0x141DF6010(worldLike)` calls `0x141B93570(worldLike + 0x6D8)`.
- `0x141B93330` is a recursive current-thread lock/acquire helper using `GetCurrentThreadId`.
- `0x141B93580` is a corresponding release helper that clears the owner when the recursion count reaches the terminal value.
- `0x141B93570` is a lighter decrement/release helper.
- Native mouse spring update `0x141E4AA30` uses these helpers around its world/body work, which is another reason it avoids unsafe concurrent direct writes.

hknp thread-safety helpers:

- `0x1415388F0` increments a read counter in the world thread-safety check at `world + 0x690`.
- `0x141538A30` decrements that read counter.
- `0x141538AF0` increments a write counter and stamps the current thread id.
- `0x141538CB0` decrements the write counter and clears the writer thread marker when write count reaches zero.
- These are checks/guards around hknp world access, not grab drive policies.

Interpretation:

- A future between-collide-and-solve custom authority callback can be made phase-coherent because FO4VR has a native listener slot exactly between collision generation and solver build.
- Wrapper calls made inside that callback may queue and then be drained immediately by the phase helper, depending on TLS state.
- Direct world functions also take hknp thread-safety locks, but using direct calls in the wrong TLS/world-lock state can fight FO4VR's command scheduling model.
- The safest implementation design cannot be chosen until this is decided explicitly:
  - call wrapper functions and rely on the phase command drain;
  - call direct compute/write functions only after proving the callback is already in a safe execution mode;
  - or build a ROCK-owned phase command that mirrors native queue semantics.

Design implication for future custom motor authority:

- Body-A proxy target, constraint transform A/B, angular target, and motor field updates should be committed in one phase.
- The strongest candidate remains `BetweenPhysicsCollideAndSolve`, but the write primitive should be selected as part of the implementation plan, not assumed.
- If a proxy velocity wrapper queues and drains after the listener callback, then constraint target writes performed directly inside the callback may become one phase earlier than the proxy velocity write. That would reintroduce body-A/body-B disagreement.
- Therefore proxy drive and constraint field writes must share the same execution model:
  - both direct in the callback, if verified safe;
  - or both queued/drained in that phase;
  - but not one direct and one deferred without proof of ordering.

### FO4VR between-collide-and-solve command context follow-up

Research question:

- If ROCK writes future custom body-A proxy motion from the between-collide-and-solve listener, will native wrappers apply immediately or queue into the phase command drain before solve?

Ghidra-confirmed `bhkWorld::vfunction43` details at `0x141DF73A0`:

- The world update creates a local command/list state around `local_128`.
- For each listener phase, `bhkWorld::vfunction43` passes `&local_128` into the listener callback.
- Between collide and solve:
  - emits timing label `TtPhysicsStepListeners-BetweenPhysicsCollideAndSolve`;
  - locks/copies the listener array;
  - calls listener vtable slot `+0x28` with `(listener, threadCount, &local_128, collideState, substepProgress, substepDt)`;
  - immediately calls command drain helper `0x141DFDE70(&local_128, workerQueue, threadCount, 1)`;
  - calls companion listener slot `+0x30`;
  - then emits timing label `TtSolve` and calls solve helper `0x141DFE1E0`.
- This establishes a hard phase order:
  - collide;
  - between-collide-and-solve callback;
  - between-collide-and-solve command drain;
  - companion callback;
  - solve.

Ghidra-confirmed command drain behavior:

- `0x141DFDE70`:
  - checks command count at `param_1[1]`;
  - if exactly one command is present and the command field at `+0x08` equals `1`, directly calls the command object's vtable `+0x20`;
  - otherwise builds batches through `0x141DFFBB0`;
  - schedules work through `0x141BCFE80` / `0x141BCFE60`;
  - waits on the global job handle at `DAT_145A3CD30 + 0x38`;
  - releases command refs and clears command counts before returning.
- `0x141DFC8D0`:
  - is the shared wrapper queue path used by set-transform, set-velocity, set-keyframed, and other world commands;
  - stores command payloads in per-thread command arrays keyed by TLS `+0x152C`;
  - registers/links the command list for the target world;
  - handles command type `9` for velocity payloads from `0x141DF56F0` / `0x141DF5930`;
  - handles command type `6` for transform payloads from `0x141DF55F0`.

Confirmed conclusion:

- FO4VR has an explicit command-list handoff inside every physics listener phase.
- Commands queued during between-collide-and-solve are drained before solve.
- This is the strongest binary evidence so far that a solver-adjacent proxy drive can be phase-coherent if it uses the same command model intentionally.

Important unresolved detail:

- This pass did not prove whether TLS command mode byte `+0x1528` is set while ROCK's listener callback is executing.
- Native wrappers such as `0x141DF5930` and `0x141DF56F0` branch on TLS `+0x1528`:
  - if clear, they write velocity directly through `0x141539F30`;
  - if set, they queue command `9` through `0x141DFC8D0`.
- The local command-list argument and immediate drain prove the queue path is supported by the phase, but not that every wrapper call from a custom ROCK listener will automatically queue.

Design implication:

- A future implementation cannot assume "call wrapper from listener" is enough without verifying runtime execution mode.
- The proxy velocity write and constraint atom writes must still be designed as one coherent unit:
  - direct proxy velocity plus direct atom writes under a verified safe write context;
  - queued proxy velocity plus queued/deferred atom writes that drain in the same phase;
  - or a ROCK-owned phase command that writes both proxy motion and constraint atom fields together.
- The current evidence supports between-collide-and-solve as the right phase, but not yet the exact write primitive.

Additional physics nuance to resolve:

- Direct hard-keyframe velocity changes body velocity, not necessarily body transform before the solver builds the constraint.
- A custom constraint target update must decide whether transform A/B should be computed from:
  - current proxy body transform;
  - desired proxy target transform;
  - or a predicted proxy transform from current transform plus hard-keyframe velocity over the substep.
- This matters because writing atoms from the desired hand frame while body A remains at its previous transform could recreate a one-frame mismatch.

### ROCK custom constraint source reread after phase-safety mapping

Current source facts:

- `Hand::createConstraintGrabDrive(...)`:
  - currently uses `_handBody.getBodyId()` as body A;
  - resolves live hand body world transform if possible;
  - computes pivot A from `_grabFrame.pivotAHandBodyLocalGame`;
  - computes pivot B from `constraintDrivePivotBBodyLocalGame(_grabFrame)`;
  - creates `ActiveConstraint` through `createGrabConstraint(...)`;
  - sets `_heldDriveMode = HeldObjectDriveMode::SharedConstraint`.
- `Hand::updateConstraintGrabDriveTarget(...)`:
  - recomputes desired body-in-hand-body space from `_grabFrame.constraintHandSpace` and `_grabFrame.bodyLocal`;
  - resolves live hand body world transform;
  - directly writes transform A position at `constraintData + offsets::kTransformA_Pos`;
  - directly writes transform B rotation and ragdoll target matrix through `grab_constraint_math::writeInitialGrabAngularFrame(...)`;
  - directly writes transform B translation through `grab_constraint_math::writeDynamicTransformBTranslation(...)`.
- `Hand::updateConstraintGrabDriveMotors(...)`:
  - calls `grab_motion_controller::solveMotorTargets(...)`;
  - feeds held collision state, position error, rotation error, base tau, collision tau, tau lerp speed, mass, force-to-mass ratio, angular-to-linear ratio, startup fade, and loose-weapon multipliers;
  - directly writes linear motor tau/damping/recovery/min/max force;
  - directly writes angular motor tau/damping/recovery/min/max force.
- `Hand::updateHeldObject(...)`:
  - ordinary native path queues `_nativeGrab.queueTarget(nativeTargetBodyWorld)`;
  - constraint path calls `updateConstraintGrabDriveTarget(...)`;
  - motor update only runs when `_heldDriveMode == HeldObjectDriveMode::SharedConstraint`.

Important interpretation:

- ROCK already has most of the finite-force HIGGS-style policy surface in `GrabMotionController`.
- The missing quality path is not primarily "invent motor values"; it is authority architecture:
  - ordinary one-hand loose objects/weapons do not use the custom constraint path;
  - custom target/motor writes happen from held-object update flow, not a verified solver-safe phase;
  - body A is the contact/lifecycle palm body, not a dedicated hidden authority proxy;
  - proxy/body-A drive and constraint target writes are not currently one atomic phase operation.
- Any future custom-authority implementation should treat current `createConstraintGrabDrive`, `updateConstraintGrabDriveTarget`, and `updateConstraintGrabDriveMotors` as the source scaffolding to preserve, but the owning phase and body-A source must be redesigned.

Specific risk to avoid:

- Do not only switch `HeldObjectDriveMode::NativeMouseSpring` to `SharedConstraint`.
- That would reuse the existing direct game-update writes and contact palm body, which the current research identifies as likely stutter/fighting sources.
- The replacement needs a coherent authority update unit:
  - proxy target capture from flattened hand frame;
  - proxy drive primitive;
  - constraint transform A/B update;
  - angular target update;
  - finite motor field update;
  - all committed in the same selected physics phase.

### ROCK held hand collision suppression reread

This source reread matters because HIGGS dynamic grab uses contact to find and shape the grab, but does not let the held object fight a collidable hand body during the held constraint. ROCK currently has both contact evidence bodies and grab authority bodies tied together more tightly than the future custom-authority design should.

Current source facts:

- `Hand::suppressHandCollisionForGrab(RE::hknpWorld* world)`:
  - clears `_grabHandCollisionDelayedRestore` before applying active suppression;
  - returns if there is no world or no hand collision body;
  - suppresses every generated bone-collider body when `_boneColliders.getBodyCount() > 0`;
  - falls back to `_handBody.getBodyId()` only when no bone-collider bodies exist;
  - reads each body's current filter with `body_collision::tryReadFilterInfo`;
  - records the body/filter in `_grabHandCollisionSuppression` through `hand_collision_suppression_math::beginSuppression`;
  - acquires a shared registry lease with owner `CollisionSuppressionOwner::Grab` and context `"held-grab-hand"`.
- `CollisionSuppressionRegistry::acquire(...)`:
  - captures the original filter the first time a body is suppressed;
  - records whether the body already had suppression bit `1 << 14`;
  - ORs `kSuppressionNoCollideBit = 1u << 14` into the current filter;
  - writes the changed filter through `body_collision::setFilterInfo`.
- `CollisionSuppressionRegistry::release(...)`:
  - keeps the no-collide bit set while any other owner lease remains;
  - restores the bit to the original pre-suppression state only after the final owner releases;
  - defers if the current filter cannot be read.
- `body_collision::setFilterInfo(...)` calls `havok_runtime::setFilterInfo(world, bodyId, filterInfo)` with the default rebuild mode.
- `BethesdaPhysicsBody::create(...)` sets the creation-time filter with `havok_runtime::setFilterInfo(world, bodyId, filterInfo, 1)`, which is different from the later suppression path.
- `Hand::grabSelectedObject(...)` calls `suppressHandCollisionForGrab(world)` before inertia normalization and drive creation.
- `Hand::updateHeldObject(...)` calls `suppressHandCollisionForGrab(world)` again every held update.
- `Hand::releaseGrabbedObject(...)` either begins delayed restore through `hand_collision_suppression_math::beginDelayedRestore(...)` or calls `restoreHandCollisionAfterGrab(world)` immediately.

Ghidra tie-in from the filter write path already mapped in this file:

- The default filter write path reaches `0x14153AF00` with rebuild mode `0`.
- In rebuild mode `0`, if body `+0x6C` is not `-1`, `0x14153AF00` calls `0x14153C5A0`.
- `0x14153C5A0` is the expensive dirty/rebuild path that wakes or dirties bodies/islands and clears packed broadphase/AABB state.
- Creation-time filter assignment through `BethesdaPhysicsBody::create(...)` uses rebuild mode `1`, avoiding that rebuild branch.

Interpretation for future custom dynamic grab authority:

- Real hand/contact bodies still need suppression while holding a grabbed object, because those bodies are contact evidence and would otherwise collide against the constrained held object.
- The authority body A should not be one of those real contact bodies if the goal is a stable custom one-hand motor path.
- A hidden authority proxy born with a non-contact filter/layer avoids steady held-state filter changes and avoids using a contact/rebuild lifecycle body as the solver authority body.
- The proxy should therefore be created already non-contact, not repeatedly suppressed/restored through the shared collision suppression registry.
- This is not a "no contact" grab design: contact still drives detection, point selection, palm seating, finger pose, collision response, and release/deviation behavior. The no-contact proxy is only the hidden constraint authority anchor.

Open verification still needed:

- Re-find the actual FO4VR hknp collision-filter branch that consumes `1 << 14`; ROCK source and old notes treat it as Ghidra-confirmed, but this pass has only reconfirmed the filter write path.
- Verify whether a noncollidable-layer body can still participate as constraint body A without being filtered from constraint solving.
- Verify whether a dedicated extended zero-row proxy layer in `48..63` gives better diagnostics/lifecycle control than layer `15` without touching detection semantics.

### ROCK palm anchor versus hidden authority proxy source audit

Research question:

- Should the future custom one-hand dynamic grab use the existing generated palm anchor as constraint body A, or a separate hidden no-contact authority proxy rooted in the same flattened hand frame?

Current source facts:

- `Hand::_handBody` is the generated palm anchor body.
- `Hand::createCollisionBodies(...)` calls `_boneColliders.create(world, bhkWorld, _isLeft, _handBody)`.
- `HandBoneColliderSet::create(...)`:
  - captures the live flattened hand bone lookup;
  - builds the `PalmAnchor` frame from the flattened root hand frame and finger/back direction;
  - creates `_handBody` as a `BethesdaPhysicsBody` with `BethesdaMotionType::Keyframed`;
  - places it immediately with `placeGeneratedKeyframedBodyImmediately(...)`;
  - initializes `_palmAnchorDriveState`;
  - publishes the palm anchor body id as the primary hand collider id.
- `HandBoneColliderSet::update(...)`:
  - captures the current flattened hand bone lookup each game-frame update;
  - queues the palm anchor target with `queueBodyTarget(palmAnchorBody, anchorFrame.transform, deltaTime, _palmAnchorDriveState)`;
  - queues all other hand collider role targets the same way.
- `HandBoneColliderSet::flushPendingPhysicsDrive(...)`:
  - drives the palm anchor first through `driveGeneratedKeyframedBody(...)`;
  - then drives each finger/palm role body through the same generated keyframed drive.
- `driveGeneratedKeyframedBody(...)`:
  - rejects stale source samples;
  - uses immediate placement only when `pendingTeleport` is set;
  - otherwise calls `BethesdaPhysicsBody::driveToKeyFrame(...)`;
  - currently ignores its max linear/angular velocity arguments.
- `HandBoneColliderSet::update(...)` defers full generated hand collider rebuilds while `palmAnchorBody.isConstrained()` is true.
- `Hand::computeGrabPivotAWorld(...)` intentionally does not read `_handBody` during grab-frame capture. Its source comment states:
  - ROCK computes grab authority from the raw tracked hand transform;
  - the generated hknp palm anchor is a driven follower flushed on the physics-step boundary;
  - reading it during game-frame setup can capture a one-step-old collider pose;
  - the generated hknp palm anchor remains contact evidence and debug geometry, not grab-frame transform authority.
- `Hand::createConstraintGrabDrive(...)` currently uses `_handBody.getBodyId()` as constraint body A.
- `Hand::updateConstraintGrabDriveTarget(...)` currently resolves live `_handBody` world transform and writes constraint atoms from held-object update flow.
- `Hand::updateHeldObject(...)` only calls `updateConstraintGrabDriveTarget(...)` and `updateConstraintGrabDriveMotors(...)` when `_heldDriveMode == HeldObjectDriveMode::SharedConstraint`; ordinary one-hand loose grabs use `_nativeGrab.queueTarget(...)`.

Confirmed interpretation:

- The existing palm anchor is a contact/evidence collider and generated-body follower.
- It is useful for detection, semantic contact, debug geometry, and palm/finger collision evidence.
- It is not an ideal future body-A authority for ordinary one-hand custom motor grab because:
  - it is driven in the generated-collider pre-collide path;
  - it uses `DriveToKeyFrame`, which has a cap-triggered transform snap in the native wrapper;
  - its lifecycle is tied to hand collider rebuild/shape ownership;
  - rebuilds are explicitly deferred while constrained;
  - it is published as a real contact body and must be suppressed during held grab;
  - the source itself already warns that it can be one physics step stale relative to raw hand authority.

Design implication for future custom dynamic grab authority:

- Reusing the current `SharedConstraint` path as-is would repeat the wrong body-A/timing architecture.
- The future body-A should be a separate hidden no-contact authority proxy:
  - rooted in the same flattened hand-frame sample used to build the palm anchor;
  - not published as semantic contact evidence;
  - born with a no-contact filter/layer rather than repeatedly suppressed/restored;
  - driven in the selected solver-adjacent phase;
  - used only as the constraint body A anchor for the held object;
  - independent from hand/finger contact collider rebuilds.
- The current `createConstraintGrabDrive(...)`, `updateConstraintGrabDriveTarget(...)`, `updateConstraintGrabDriveMotors(...)`, `GrabConstraintMath`, and `GrabMotionController` remain valuable scaffolding, but the future authority path must replace the body-A owner and update phase.

Important boundary:

- This is not a no-contact grab design.
- Contact bodies remain required for dynamic detection, mesh/contact pivot selection, palm seating, finger pose, collision feedback, and release/deviation policy.
- Only the hidden authority proxy should be no-contact; the real held object keeps normal object collision behavior.

Still unresolved:

- Need design the proxy owner/lifecycle surface separately from `HandBoneColliderSet` so it cannot participate in contact metadata or collider rebuild deferral.
- Need verify whether `BethesdaPhysicsBody` is the safest first proxy body wrapper, or whether a later direct hknp body helper is worth the lifecycle cost.
- Need decide whether the proxy should use bit 14, layer 15, or a dedicated extended layer after comparing debug visibility, matrix behavior, and collision-suppression restore risk.
- Need map how the latest root-flattened hand sample is cached for a between-collide-and-solve callback without reading stale generated body transforms.

### ROCK root-flattened hand-frame authority source audit

Research question:

- How should a future hidden body-A proxy get the same hand-frame convention that already works, without reading stale generated hknp palm-anchor transforms?

Current source facts:

- `PhysicsInteraction::update()`:
  - calls `refreshHandBoneCache()`;
  - calls `sampleHandTransformParity()`;
  - then builds one `PhysicsFrameContext` through `buildFrameContext(bhk, hknp, _deltaTime)`.
- `HandBoneCache::resolve()`:
  - captures `DebugSkeletonBoneMode::HandsAndForearmsOnly`;
  - uses `DebugSkeletonBoneSource::GameRootFlattenedBoneTree`;
  - finds `RArm_Hand` and `LArm_Hand`;
  - stores copied `RE::NiTransform` values for both hands;
  - stores skeleton/bone-tree/power-armor identity;
  - does not store scene-node authority.
- `HandFrameResolver::resolve(...)`:
  - returns the root-flattened copied hand transform when the cache is ready;
  - returns `node = nullptr`;
  - labels the frame as `left-root-flattened-hand-bone` or `right-root-flattened-hand-bone`;
  - explicitly avoids mixing scene-node conventions into collision, palm selection, grab math, and debug axes.
- `PhysicsInteraction::buildFrameContext(...)`:
  - reads `getInteractionHandTransform(isLeft)`;
  - fills `HandFrameInput::rawHandWorld`;
  - computes `grabAnchorWorld` through `hand.computeGrabPivotAWorld(hknp, input.rawHandWorld)`;
  - computes `palmNormalWorld` and `pointingWorld` from the same raw hand frame;
  - gives selection, grab input, soft contact, debug, and held-object logic one coherent per-frame hand snapshot.
- `HandBoneColliderSet::captureBoneLookup(...)` independently reads the same `GameRootFlattenedBoneTree` source for generated hand colliders.
- `Hand::computeGrabPivotAWorld(...)` deliberately uses the raw hand transform, not `_handBody`, because the generated hknp palm anchor is a physics-step follower and can be stale during game-frame grab setup.

Confirmed interpretation:

- The proven ROCK convention is already the root-flattened hand bone transform, not native FO4VR mouse-spring conventions and not scene-node fallback transforms.
- The future custom grab authority should keep that convention:
  - raw/root-flattened hand frame is authority;
  - generated palm/finger colliders are contact evidence;
  - hidden no-contact proxy is the solver-side body-A representative of that raw hand frame.
- The proxy should not read `_handBody` as its authority source.
- The proxy should not read FRIK/skeleton state directly from inside a physics-step callback unless a future thread-safety pass proves that is safe.

Current architecture gap:

- `PhysicsStepDriveCoordinator` currently exposes only:
  - whole-pre-step callback;
  - substep-pre-collide callback.
- The native vtable already has a `betweenCollideAndSolve` slot, but ROCK's current implementation is a no-op.
- The current callbacks pass only `(userData, hknpWorld, timing)`.
- There is no dedicated cached `DynamicGrabAuthorityFrame` or similar payload for solver-adjacent held-grab authority.

Design implication for future custom dynamic grab authority:

- The future body-A proxy update should consume a cached hand authority sample produced during game-frame update from the same `PhysicsFrameContext` convention.
- That cache should include, at minimum:
  - right/left raw root-flattened hand transforms;
  - right/left palm pivot positions;
  - right/left palm normals/pointing vectors if needed for diagnostics;
  - frame delta/time sequence;
  - disabled/valid flags;
  - world/lifecycle generation guards.
- The between-collide-and-solve callback should consume the latest valid cached sample, not call `DirectSkeletonBoneReader` or read `_handBody` transforms.
- Visual hand override from held-object deviation should remain visual-only and must not feed back into this authority sample.

Still unresolved:

- Need design a thread-safe cache handoff from game-frame `PhysicsFrameContext` to the physics-step coordinator.
- Need verify whether the existing coordinator can be extended cleanly to support a `betweenCollideAndSolve` callback without disrupting generated contact-collider pre-collide driving.
- Need define staleness policy for proxy authority if no fresh root-flattened hand sample is available.
- Need map how player-space movement compensation should apply to the cached hand authority sample versus the held object constraint target.

### ROCK held player-space compensation source audit

Research question:

- How does ROCK currently prevent held objects from stuttering or drifting when the player/room space moves, and what does that imply for a future custom motor authority?

Current source facts:

- `PhysicsInteraction::update()` calls these in order:
  - `updateSelection(frame)`;
  - `_heldObjectPlayerSpaceFrame = sampleHeldObjectPlayerSpaceFrame(frame.deltaSeconds)`;
  - `applyHeldPlayerSpaceVelocity(hknp)`;
  - `updateGrabInput(frame)`.
- The source comment before `applyHeldPlayerSpaceVelocity(...)` states that compensation is applied before held-object grab constraints are updated so the constraint target does not solve against stale body velocity.
- `HeldObjectPlayerSpaceFrame` stores:
  - translation delta in game units;
  - player-space velocity in Havok units;
  - previous/current player-space transforms;
  - rotation delta;
  - source string;
  - enabled/warp flags.
- `sampleHeldObjectPlayerSpaceFrame(...)`:
  - uses `FRIKApi::getSmoothedPlayerPosition()`;
  - uses `PlayerNodes::roomnode` rotation when available;
  - produces `velocityHavok` from smooth-position delta;
  - marks distance and rotation warps separately;
  - stores previous/current player-space transforms for runtime transform warp.
- `applyHeldPlayerSpaceVelocity(...)`:
  - gathers primary and connected held body ids from both hands;
  - computes residual velocity keep from grab velocity damping;
  - decides whether runtime transform warp should apply;
  - calls `held_player_space_registry::applyCentralPlayerSpaceVelocity(...)`;
  - stores `_lastCentralHeldPlayerSpaceVelocityHavok` only when compensation stays enabled and writes at least one motion.
- `HeldPlayerSpaceRegistry`:
  - registers held bodies by body id and motion index;
  - deduplicates writes by motion index;
  - subtracts previous player velocity to recover local body velocity;
  - adds current player velocity back with residual keep;
  - applies runtime transform warp by transforming body world through previous/current player-space transforms;
  - records the expected steady-state writer mask as `ConstraintTarget | PlayerSpaceCentral`.
- `Hand::recordHeldControllerMotionSample(...)`:
  - computes raw hand velocity from root-flattened hand position delta;
  - subtracts `playerSpaceFrame.velocityHavok` when player-space compensation is enabled and not warped;
  - stores local hand velocity and angular velocity histories;
  - clears histories on player-space warp.
- `Hand::recordHeldObjectVelocitySample(...)`:
  - samples held object motion after subtracting previous player-space velocity;
  - stores local held-object velocity history for release/lead behavior;
  - updates `_lastPlayerSpaceVelocityHavok`.
- `Hand::captureHeldReleaseMotion(...)` is called before release and records both hand and object local motion using the same player-space frame.
- `Hand::applyReleaseVelocitySnapshot(...)` applies the captured release velocity only from the release snapshot path, deduping connected bodies by the saved snapshot.

Confirmed interpretation:

- ROCK already has the correct high-level HIGGS-like concept here: held-object motion is expressed relative to player-space movement, while player movement itself is handled centrally.
- The current design deliberately avoids per-hand held loops writing player-space velocity, because two hands or connected multipart bodies could otherwise write the same hknp motion more than once.
- This is directly relevant to old motor stutter: any future custom motor authority that adds another per-hand compensation writer would violate the existing writer-mask invariant.

Design implication for future custom dynamic grab authority:

- The hidden body-A proxy should be driven from the cached root-flattened hand frame.
- The held object body set should continue to receive player-space compensation from one central writer unless research proves it must move phases.
- The proxy itself should not be registered as a held object in `HeldPlayerSpaceRegistry`.
- Constraint target/motor updates should account for the same frame of player-space movement as the held object, but should not create a second player-space velocity path.
- The target architecture should preserve the steady-state mental model:
  - one constraint/proxy target authority for hand-to-object relation;
  - one central player-space writer for room/player movement;
  - release history samples local hand/object motion with player velocity removed.

Still unresolved:

- Need decide whether central player-space compensation must remain in game update before `updateGrabInput(...)`, or move into the same physics phase as proxy/constraint updates for the custom-authority path.
- Need map whether moving only proxy/constraint updates to between-collide-and-solve while leaving player-space compensation earlier creates a one-phase mismatch.
- Need decide how runtime transform warp should interact with the hidden proxy during teleport-like room/player-space changes.
- Need preserve `HeldPlayerSpaceRegistry` dedupe by motion index for multipart loose weapons and multi-body objects.

### HIGGS dynamic body-A and keyframed hand-body logic source audit

Research question:

- In the HIGGS dynamic grab path, what is the actual body-A authority and how does it avoid turning COM or mouse-spring behavior into the grip model?

Confirmed HIGGS source facts:

- `Hand::TransitionHeld(...)`:
  - computes `desiredNodeTransform = adjustedTransform`;
  - translates `desiredNodeTransform.pos += palmPos - ptPos`;
  - stores `desiredNodeTransformHandSpace = inverseHand * desiredNodeTransform`;
  - collects all grabbed/connected rigid bodies;
  - if `usePhysicsGrab` and `warpToHand`, teleports the grabbed body and sibling bodies so the selected point seats at the palm;
  - computes `hkPivotA` from `palmPos`;
  - computes `hkPivotB` from `ptPos`;
  - resolves `bodyA = handBody->hkBody`;
  - resolves `bodyB = selectedObject.rigidBody->hkBody`;
  - transforms pivot A into body-A local space;
  - transforms pivot B into body-B local space;
  - builds `handTransformHandSpace` from local pivot A;
  - builds `desiredHavokTransformHandSpace = desiredNodeTransformHandSpace * GetRigidBodyTLocalTransform(selectedObject.rigidBody)`;
  - builds `handTransformObjSpace = InverseTransform(desiredHavokTransformHandSpace)`;
  - stores pivot B into `handTransformObjSpace.pos`;
  - calls `CreateGrabConstraint(bodyA, bodyB, handTransformHandSpace, handTransformObjSpace)`;
  - adds the resulting constraint to the hand body and world.
- `UpdateKeyframedNode(...)`:
  - updates the node transform;
  - calls `NiAVObject_UpdateNode` with flag `0x2000` so the collision object is moved by velocity;
  - for `bhkRigidBodyT`, manually computes the rigid-body transform and calls `ApplyHardKeyframeVelocityClamped(...)`;
  - updates bone matrices so visual geometry is not a frame behind.
- `ApplyHardKeyframeVelocityClamped(...)`:
  - calls `hkpKeyFrameUtility_applyHardKeyFrame(...)`;
  - if computed linear velocity exceeds the body's native max, sets position directly;
  - if angular velocity exceeds max, sets rotation directly.
- `Hand::Update` while `state == HeldBody`:
  - refreshes finger animation values;
  - registers object/connected/contained body mass;
  - registers player-space bodies when player movement compensation is enabled;
  - computes `heldTransform = collidableNode->m_worldTransform`;
  - computes `inverseDesired = InverseTransform(desiredNodeTransformHandSpace)`;
  - computes `m_adjustedHandTransform = heldTransform * inverseDesired`;
  - lerps adjusted hand transform during startup;
  - tracks hand deviation from the real hand and drops when average deviation exceeds the configured max;
  - calls `UpdateHandTransform(m_adjustedHandTransform)` for the visual/adjusted hand;
  - updates `GrabConstraintData::setTargetRelativeOrientationOfBodies(...)`;
  - recomputes transform-B translation as `(desiredHandTransformHavokObjSpace * palmPosHandspace) * objectScale * havokWorldScale`;
  - updates linear/angular motor tau, damping, recovery, min/max force.
- HIGGS non-actor mass behavior:
  - loose weapons get `grabConstraintLinearMaxForceWeapon`;
  - generic loose objects get `grabConstraintLinearMaxForce`;
  - angular max force is derived from linear max force divided by `grabConstraintAngularToLinearForceRatio`;
  - collision switches linear/angular tau toward colliding tau values;
  - final non-actor force is capped by `mass * grabConstraintMaxForceToMassRatio`;
  - angular max force is capped again from the capped linear force.
- HIGGS player-space behavior:
  - held and connected bodies are registered with `RegisterPlayerSpaceBody(...)`;
  - room/player movement adds velocity to registered bodies when movement is smooth;
  - room/player rotation warp transforms registered bodies through previous/current room transforms;
  - hand/weapon collision is moved by the same player-space delta, then hand keyframe velocity overwrites it in the following hand update.

Confirmed interpretation:

- HIGGS dynamic grab body A is the hand collision body, not mouse spring and not the grabbed object COM.
- HIGGS body A is effectively a keyframed hand body driven from the hand/node transform.
- HIGGS body B is the selected dynamic held object body.
- The constraint frames are palm/contact based:
  - pivot A from palm;
  - pivot B from selected contact/mesh point;
  - object rotation preserved through `desiredNodeTransform`;
  - object translated so selected point seats at the palm.
- HIGGS uses COM/mass only to cap force, compute held weight/speed effects, and release/throw behavior.
- HIGGS does allow visual hand deviation: the rendered/adjusted hand follows the object when finite force cannot keep up, and deviation limits decide release instead of giving the player infinite strength.

FO4VR/ROCK mapping implication:

- The FO4VR equivalent of HIGGS body A should be a keyframed hand-frame body rooted in ROCK's root-flattened hand frame.
- Because ROCK's current palm anchor is also a contact/evidence collider with pre-collide `DriveToKeyFrame` lifecycle, a separate hidden no-contact authority proxy can reproduce the HIGGS body-A role more cleanly in FO4VR hknp.
- The proxy is not a different grab model. It is ROCK's FO4VR-native way to recreate HIGGS' "keyframed hand body A" without making the contact palm collider fight the held object.
- The constraint logic to preserve from HIGGS is:
  - contact/palm pivot capture;
  - body-A/body-B local pivots;
  - preserved object rotation;
  - per-frame transform-B pivot refresh;
  - angular target refresh;
  - finite motor force/tau policy;
  - visual hand deviation and release.
- The implementation must not copy HIGGS' hkp timing blindly. FO4VR hknp has a different step/listener/command model, so the HIGGS concept maps to FO4VR's solver-adjacent hidden proxy and live atom updates.

Still unresolved:

- Need verify whether FO4VR's hknp constraint solver expects body-A transform A to be current, desired, or predicted when body A is velocity-driven instead of transform-snapped.
- Need decide whether the hidden proxy should ever use direct transform set for teleport/warp recovery, matching HIGGS' clamp fallback, while using velocity drive for normal smooth movement.
- Need decide how ROCK's visual hand deviation system should map HIGGS' `m_adjustedHandTransform` for a future custom constraint authority without feeding visual hand correction back into the physical hand-frame source.

### FO4VR no-collide bit and constraint-contact separation

This Ghidra pass resolves the open `1 << 14` question and clarifies why a hidden no-contact authority proxy is still compatible with a custom constraint drive.

#### Contact filtering predicate

Confirmed functions:

- `bhkCollisionFilter::vfunction5` at `0x141E11800`:
  - iterates candidate body-id pairs;
  - reads each body's collision filter info from hknp body slot offset `+0x44`;
  - calls `0x141E115B0` with the two filter values;
  - keeps only the pairs where `0x141E115B0` returns true.
- `bhkCollisionFilter::vfunction6` at `0x141E11930`:
  - also calls `0x141E115B0` with two filter values.
- Contact filter predicate `0x141E115B0`:
  - reads bit 14 from both filter infos via `filterInfo >> 0xE & 1`;
  - if either filter has bit 14 set, it returns false before layer/matrix checks;
  - extracts the collision layer as `filterInfo & 0x7F`;
  - uses the matrix at `this + 0x1A0 + layer * 8` for normal layer-pair permission;
  - has extra group/subsystem branches for upper filter bits, including same-system checks and special handling around layer `0x1E`.

Confirmed conclusion:

- `kSuppressionNoCollideBit = 1u << 14` is a real FO4VR contact-filter bit.
- Setting bit 14 on a body disables ordinary contact generation against that body through the active `bhkCollisionFilter` path.
- This is not just a ROCK convention.

#### Constraint insertion is separate from contact filtering

Reconfirmed function:

- hknp world constraint creation at `0x1415469B0`:
  - reserves/adds a constraint id in the world constraint table;
  - initializes the constraint slot through `0x1417E39C0`;
  - emits the world constraint-add signal at `world + 0x560` through `0x14155A050`;
  - does not call `0x141E115B0`;
  - does not read body filter info `+0x44` as a condition for accepting the constraint.

Confirmed conclusion:

- Ordinary contact filter/layer permission is not the gate for constraint creation.
- A body can be non-contact filtered and still be a constraint body, at least by the insertion path evidence. This still needs runtime validation once implementation is allowed, but the binary does not show contact-filter rejection in the constraint creation path.

#### Optional hknp constraint collision filter

Confirmed functions:

- `hknpConstraintCollisionFilter` constructors:
  - `0x1417ED4F0`;
  - `0x1417ED530`.
- hknp world constructor path around `0x141540D90`:
  - initializes signal slots at `world + 0x560` and `world + 0x568`;
  - if cinfo field `+0xA8` is non-null and its byte at `+0x10` equals `1`, calls `0x1417ED680`.
- `0x1417ED680`:
  - unregisters old signal state through `0x1417ED6F0`;
  - registers member slots on `world + 0x560` and `world + 0x568`;
  - stores the world pointer at filter offset `+0x30`.
- `0x1417ED740`:
  - add-constraint signal handler;
  - resolves the constraint slot from `world + 0x128 + constraintId * 0x38`;
  - reads the two body ids from the constraint slot;
  - calls `0x14196DE70`.
- `0x14196DE70`:
  - sorts the two body ids into a stable pair key;
  - increments/stores that pair in a pair-filter map at filter offset `+0x18`;
  - calls `0x14153C5A0` on one selected body to dirty/wake/update collision state.
- `0x1417ED7D0`:
  - remove-constraint signal handler;
  - resolves the same pair from the constraint slot;
  - calls `0x14196DF80`.
- `0x14196DF80`:
  - decrements the pair count;
  - removes the pair when the count reaches zero;
  - if both body ids are still valid dynamic bodies, calls `0x14153C8F0` with the pair to refresh collision state.

Confirmed conclusion:

- FO4VR has a separate constraint-collision filter that suppresses contacts between constrained body pairs.
- That filter listens to constraint lifecycle signals. It is not the same thing as the layer/matrix filter predicate.
- Its job is to avoid contact generation fighting constrained pairs, not to stop constraints from solving.

Design interpretation for future custom dynamic grab authority:

- A hidden body-A authority proxy can be born non-contact through bit 14 or a no-contact layer and still participate in a body-A/body-B grab constraint.
- If the hknp constraint collision filter is active in the world cinfo, adding the grab constraint may also suppress contacts specifically between body A and body B through the pair-filter map.
- That pair suppression would be harmless for a hidden proxy because the proxy should not generate contacts anyway.
- The held object itself should keep its normal object layer and collision behavior; only the hidden proxy/contact-hand bodies should be non-contact relative to the held body.
- This supports a clean separation:
  - contact bodies: find/select/pose and then suppress against the held object while held;
  - hidden authority proxy: no-contact, flattened-hand-frame body A for stable custom motor authority;
  - held object body: normal dynamic body B with finite linear/angular motors.

Still unresolved:

- Whether layer `15`, bit 14, or a dedicated extended zero-row layer is the best proxy non-contact policy is still an implementation design choice.
- Need verify the exact `BethesdaPhysicsBody` creation flags/motion type that produce the best body-A proxy with no contact and no broadphase churn.
- Need verify whether constraint-add pair suppression is active in the actual FO4VR world cinfo used by gameplay, although the world constructor supports it.

### FO4VR gameplay world filter and proxy timing audit

This pass corrects the older "optional constraint collision filter may be active" interpretation. FO4VR contains the constraint-pair filter machinery, but the gameplay `bhkWorld` creation path inspected here does not appear to install it as the world collision filter. It installs Bethesda's normal `bhkCollisionFilter`.

#### Gameplay hknp world cinfo path

Confirmed functions:

- `bhkWorld::vfunction44` at `0x141DF99E0`:
  - constructs a stack hknp world cinfo through `0x141724500`;
  - sets world capacity/timing/broadphase/body-quality fields;
  - sets cinfo collision filter pointer `+0xA8` to global `DAT_1459429B8`;
  - allocates the hknp world object and inlines the `hknpBSWorld`/`hknpWorld` construction path.
- hknp world cinfo initializer `0x141724500`:
  - initializes cinfo filter pointers `+0xA0`, `+0xA8`, `+0xB0` to zero;
  - sets other default simulation/broadphase/body-quality fields;
  - does not create a constraint-pair filter by itself.
- `bhkCollisionFilter::bhkCollisionFilter` at `0x141E11480`:
  - writes `DAT_1459429B8 = this`;
  - sets its hknp collision-filter type byte at object offset `+0x10` to `4`;
  - initializes Bethesda layer/name/matrix data;
  - calls `0x141E11950`, which fits the active `bhkCollisionFilter` setup path already tied to `vfunction5`/`vfunction6` and predicate `0x141E115B0`.

Confirmed conclusion:

- The gameplay `bhkWorld` path feeds hknp world cinfo `+0xA8` with `DAT_1459429B8`.
- `DAT_1459429B8` is a `bhkCollisionFilter` whose type byte is `4`.
- This is the normal contact filter that applies bit 14 and the layer matrix.
- This is not the `hknpConstraintCollisionFilter` type.

#### Constraint collision filter support is present but not the default gameplay filter

Confirmed functions:

- `hknpConstraintCollisionFilter::hknpConstraintCollisionFilter` at `0x1417ED4F0`:
  - calls `hknpPairCollisionFilter` construction;
  - sets its hknp collision-filter type byte at object offset `+0x10` to `1`;
  - initializes its pair map state.
- hknp world constructor tail at `0x1415424D1` to `0x1415424E6`:
  - reads cinfo pointer `+0xA8`;
  - if non-null and byte `+0x10 == 1`, calls `0x1417ED680`;
  - `0x1417ED680` registers callbacks on world constraint add/remove signal slots `world + 0x560` and `world + 0x568`.
- `hkbnpPhysicsInterface::hkbnpPhysicsInterface` at `0x141956570`:
  - if cinfo `+0xA8` is null, allocates a group-style collision filter and sets its type byte to `2`;
  - then constructs an hknp world using that cinfo;
  - this confirms the constructor check is a generic hknp filter-type hook, not a guarantee that gameplay uses the constraint filter.

Corrected conclusion:

- FO4VR has the constraint-pair filter class and hknp world can register it.
- The inspected gameplay `bhkWorld` creation path uses `bhkCollisionFilter` type `4`, so automatic hknp constraint-pair contact suppression should not be assumed active for normal ROCK gameplay worlds.
- The earlier section's "if active" wording remains true only for worlds whose cinfo `+0xA8` is explicitly a type-`1` `hknpConstraintCollisionFilter`; it is not true for the gameplay world path mapped here.

Design implication:

- A future custom dynamic grab authority path cannot rely on hknp automatically suppressing contacts between constrained body A and body B.
- The clean split remains:
  - hand/contact bodies provide detection, selection, mesh evidence, finger pose, and collision/deviation data;
  - while held, those contact bodies need ROCK's existing bit-14 suppression against the held object;
  - a hidden authority proxy should be born no-contact, using bit 14 or a no-contact layer policy;
  - the held object stays on its normal layer and remains the only dynamic constrained body B.
- Replacing the gameplay collision filter with `hknpConstraintCollisionFilter` is not a safe inferred option because Bethesda's `bhkCollisionFilter` owns layer matrix behavior, the bit-14 suppression branch, and game-specific group/subsystem logic.

#### Body-A proxy motion type and drive surface

Current source facts:

- `BethesdaPhysicsBody::create(...)`:
  - creates a Bethesda-owned hknp physics system and `bhkNPCollisionObject`;
  - inserts it through `bhkNPCollisionObject` add-to-world;
  - for non-static bodies, supplies a local motion cinfo before add-to-world;
  - for `BethesdaMotionType::Keyframed`, calls `kFunc_SetBodyKeyframed` after add-to-world;
  - sets filter info with rebuild mode `1`;
  - enables flags `0x08020000`;
  - activates the body.
- Generated hand, body, and weapon colliders are all currently created as `BethesdaMotionType::Keyframed`.
- The generated palm anchor body is already rooted in the flattened hand-frame bone tree; future custom grab body A should use that same convention for target/source transform semantics.

Ghidra-confirmed functions:

- `kFunc_SetBodyKeyframed` / `0x141DF5CB0`:
  - direct path locks the world and calls `0x14153B640`;
  - queued path emits command `0x10` with command flags `0x120100`.
- `0x14153B640`:
  - sets the keyframed flag/state on the hknp body/motion;
  - dirties the body and linked/connected chain through `0x14153C5A0`;
  - notifies through `world + 0x538`;
  - this is a motion-state promotion path, not a per-frame target drive.
- `bhkNPCollisionObject::SetMotionType` / `0x141E07300`:
  - routes static/dynamic/keyframed modes through multiple hknp world helpers;
  - can rebuild mass/motion state depending on current flags;
  - should not be part of steady held-frame proxy movement.

Confirmed conclusion:

- A hidden authority proxy should probably be a keyframed generated Bethesda body, matching the existing hand/weapon generated-body ownership model.
- Keyframed state is a creation/setup property; it does not by itself define the grab authority behavior.
- Steady held motion still needs a separate per-frame drive primitive.

#### Current generated drive still uses DriveToKeyFrame

Current source facts:

- `GeneratedKeyframedBodyDrive::driveGeneratedKeyframedBody(...)`:
  - locks the drive state;
  - rejects stale sources after `0.10s`;
  - selects pending or interpolated target;
  - if `pendingTeleport` is set, calls immediate placement through `setTransform` + zero velocity;
  - otherwise calls `BethesdaPhysicsBody::driveToKeyFrame(...)`;
  - currently ignores the `maxLinearVelocityHavok` and `maxAngularVelocityRadians` parameters.
- `PhysicsInteraction` registers:
  - native held grab flush on whole-pre-step;
  - generated collider drive flush on substep-pre-collide.
- `PhysicsStepDriveCoordinator` has a native vtable slot for between-collide-and-solve but the current ROCK implementation is no-op.
- `HavokPhysicsTiming::PhysicsStepPhase` currently only names `WholePreStep` and `SubstepPreCollide`.

Ghidra-confirmed drive functions:

- `bhkNPCollisionObject::DriveToKeyFrame` wrapper `0x141E086E0`:
  - computes hard-keyframe velocity;
  - checks body motion-property max linear/angular velocity caps;
  - if a cap is exceeded, uses `0x141DF55F0` to set body transform and zeros velocity;
  - otherwise uses `0x141DF56F0` to apply velocity.
- Direct hard-keyframe velocity wrapper `0x141DF5930`:
  - calls `0x14153A6A0` to compute linear/angular velocities from current body transform to target transform using `dt`;
  - applies velocities through `0x141539F30` or queues the same velocity command;
  - does not contain the `DriveToKeyFrame` cap-triggered transform-snap branch.
- `0x141DF56F0`:
  - applies linear/angular velocity directly through `0x141539F30` when not in queued command mode;
  - queues command `0x30` with flags `0x90100` when TLS command mode is active;
  - calls `0x141DF60C0` when the velocity is nonzero enough to require body/island notification.
- `0x141DF55F0`:
  - direct path calls `0x1415395E0` to write body transform;
  - queued path emits command `0x50` with flags `0x60100`;
  - this is broadphase-visible and should be reserved for create/teleport/recovery, not normal held-frame drive.
- `0x141539F30`:
  - writes motion linear velocity and angular velocity;
  - wakes/activates the body if needed through `0x141546EF0`;
  - calls `0x14153D440`;
  - signals `world + 0x538`.

Follow-up Ghidra call-site audit:

- `0x141DF5930` xrefs:
  - data xref at `0x146AEA160`;
  - call from `0x1412CB4E0`;
  - call from `bhkNPCollisionObject::vfunction44` at `0x141E09890`.
- `0x1412CB4E0`:
  - is a camera/update-adjacent path that calls `TESCamera::vfunction4`;
  - later drives a body id stored around the owner object with either `0x141DF5680` or `0x141DF5930`;
  - uses world lock helpers around the operation when a world pointer is present.
- `bhkNPCollisionObject::vfunction44` at `0x141E09890`:
  - resolves the body id from the `bhkNPCollisionObject`;
  - uses world lock helper `0x141DF5FB0` unless TLS `+0x1529` indicates an already-active physics context;
  - reads the hknp body flags at body slot `+0x40`;
  - when the target body/path uses the DriveToKeyFrame branch, calls `0x141E086E0`;
  - when the body/path uses the velocity-drive branch and `DAT_1465A3D84 > 0.0`, calls `0x141DF5930`;
  - then, in the same native vfunction flow, can also call `0x141DF55F0` to sync/set the body transform.

Confirmed nuance:

- `0x141DF5930` itself is a compute-velocity/apply-velocity wrapper and has no cap-triggered transform snap branch.
- The generic `bhkNPCollisionObject::vfunction44` update path is not a pure velocity primitive, because it can combine direct velocity with transform setting/sync in the same object update.
- Therefore "use the direct velocity path" must mean using the world/body velocity primitive deliberately, not blindly routing the hidden proxy through the normal generated-collision-object vfunction/update behavior.

Thread-safety/command-mode findings:

- `0x14153ABD0`:
  - takes the hknp world write guard through `0x141538AF0`;
  - computes hard-keyframe velocity through `0x14153A6A0`;
  - writes velocity through `0x141539F30`;
  - releases the write guard through `0x141538CB0`;
  - does not use the queued command wrapper.
- `0x141DF5930`:
  - computes hard-keyframe velocity through `0x14153A6A0`;
  - writes through `0x141539F30` when TLS command mode `+0x1528` is clear;
  - queues command `0x30` with flags `0x90100` when TLS command mode `+0x1528` is set;
  - calls `0x141DF60C0` when nonzero velocity requires activation/island handling.
- `0x141DF56F0`:
  - has the same direct-versus-queued set-velocity behavior for caller-supplied linear/angular velocity.
- `0x141DF55F0`:
  - sets body transform directly through `0x1415395E0` or queues command `0x50`;
  - should remain create/teleport/recovery/sync behavior, not steady hidden-proxy authority.

Design implication:

- For a hidden no-contact body-A proxy, the future implementation should not reuse the existing generated-collider `driveGeneratedKeyframedBody` path as-is.
- The proxy needs a deliberately scoped drive primitive:
  - either a wrapper-style call to `0x141DF5930` inside a phase where queued commands drain before solve;
  - or a direct compute/write path equivalent to `0x14153ABD0` only after proving the callback already owns a safe hknp write context;
  - or a ROCK-owned phase command that keeps proxy velocity and constraint atom target writes in the same execution model.
- The proxy drive and custom constraint target/motor writes must stay phase-matched. A design where proxy velocity is deferred but constraint atoms are written immediately would reintroduce body-A/body-B disagreement.

Confirmed conclusion:

- The current generated-collider drive is still a `DriveToKeyFrame` user.
- The direct velocity path remains the cleaner candidate for a future no-contact authority proxy because it avoids the cap-triggered transform snap.
- This does not mean existing contact colliders should be moved to the new drive without separate validation; generated contact colliders need pre-collide placement for contact evidence, while a hidden grab authority proxy can be treated as a solver anchor.

#### Physics phase implications for custom linear/angular grab authority

Ghidra-confirmed `bhkWorld::vfunction43` order at `0x141DF73A0`:

- before-whole listeners;
- command drain `0x141DFDB30`;
- for each substep:
  - before-any listeners;
  - command drain `0x141DFE3B0`;
  - collide;
  - between-collide-and-solve listeners;
  - command drain `0x141DFDE70`;
  - solve;
  - after-any listeners;
  - command drain `0x141DFDCD0`;
- after-whole listeners;
- command drain `0x141DFD990`.

Confirmed conclusion:

- Between-collide-and-solve is the first phase where collision has finished and the solver has not yet consumed constraint state.
- If a future authority proxy uses a velocity wrapper that queues commands during the callback, FO4VR drains those queued commands before solve.
- Therefore a coherent future custom-authority update unit can be designed around one between-collide-and-solve callback:
  - compute proxy target from the latest flattened hand frame sample;
  - drive the hidden no-contact body A through direct hard-keyframe velocity semantics;
  - write constraint transform A/B, transform-B pivot, and angular target;
  - write finite linear/angular motor fields;
  - let the command drain run;
  - then solve consumes a matched proxy/constraint/motor frame.

Important caution:

- This is a future design implication, not an implementation change.
- Do not move all generated collider driving to between-collide-and-solve. Hand/body/weapon contact colliders exist to generate contact during collide and still need pre-collide placement.
- The future between-collide-and-solve path should be scoped to the hidden dynamic-grab authority proxy and grab constraint fields only.
- Do not only switch ordinary one-hand grabs to the current `SharedConstraint` path. The current `SharedConstraint` updates constraint fields from held-object/game update flow and uses the contact palm body as body A, which does not satisfy the matched physics-phase authority unit above.

Unresolved after this audit:

- Need verify whether calling `0x141DF5930` directly from a ROCK wrapper is safer than computing velocity through `0x14153A6A0` plus `setVelocity`, given available typed signatures and queued-command behavior.
- Need verify if future custom constraint field writes need any explicit constraint dirty notification beyond writing atom memory before solve. Current atom interpreter evidence says solver reads atom stream each build-jacobian pass, but this should get one more focused Ghidra pass.
- Need map whether the future proxy should share the existing palm-anchor body lifetime or become a separate `BethesdaPhysicsBody` with its own no-contact filter. Current evidence favors separate hidden proxy, but source ownership/lifecycle still needs a full pass.
- Need decide proxy no-contact policy: bit 14, layer 15, or dedicated extended zero-row layer. The binary confirms bit 14 is real; it does not decide the production policy.

### FO4VR live constraint atom update safety audit

Research question:

- If ROCK returns to a custom linear/angular motor authority, can it update constraint target/motor atom fields before solve, or does FO4VR require a separate dirty/rebuild notification for every target change?

#### Constraint creation stores live data pointers

Ghidra-confirmed function roles:

- `0x1415469B0`:
  - gameplay hknp world constraint creation path;
  - locks the world;
  - assigns a world constraint id with `0x1417E46F0`;
  - computes the constraint slot at `world[0x128] + id * 0x38`;
  - initializes the slot through `0x1417E39C0`;
  - emits the create signal at `world + 0x550`;
  - if enabled and the constraint data type is valid, adds/builds the live constraint through `0x14154A810`;
  - emits the add signal at `world + 0x560`.
- `0x1417E39C0`:
  - initializes a hknp world constraint slot from a constraint cinfo/data object;
  - stores body ids at slot offsets `+0x00` and `+0x04`;
  - stores a refcounted pointer to the constraint data at slot `+0x08`;
  - calls the constraint data vtable method at `+0x28` to get atom/constraint-info data;
  - stores the returned atom pointer at slot `+0x18`;
  - stores atom/runtime metadata at slot offsets around `+0x20`, `+0x22`, `+0x25`, and `+0x26`;
  - allocates and zeroes solver runtime memory at slot `+0x28` only when runtime size is nonzero.

Confirmed conclusion:

- Constraint creation does not copy the full atom stream into a separate immutable world buffer.
- The world slot keeps a pointer to the constraint data and a pointer to the atom stream reported by the constraint data.
- Runtime memory is separate from atom target/motor fields and is allocated once at creation.

Design implication:

- A future ROCK custom grab authority must treat the custom constraint data object as live-owned state.
- Target/motor field writes should update that live atom stream before the solver builds jacobians.
- Recreating the constraint or reallocating runtime every frame would throw away solver history and is a plausible source of the old custom-motor stutter.

#### Solver build reads current atom memory

Ghidra-confirmed function roles:

- `0x141A55550`:
  - live hknp build-jacobians atom interpreter;
  - receives an atom pointer, an atom byte count/end offset, a solver context, and an output cursor;
  - iterates from `atomBase` to `atomBase + atomByteCount`;
  - reads the atom type from the current atom memory with `*(ushort*)atom`;
  - dispatches through a switch table at `0x141A5A32C`;
  - reads transform, stabilization, ragdoll-motor, and linear-motor fields from the current atom pointer while building the solver output.
- `0x1419799E0`:
  - live constraint build caller;
  - passes the active constraint atom pointer and atom byte count into `0x141A55550`.
- `0x141979EF0`:
  - second live constraint build caller;
  - passes the active world-slot atom pointer and atom byte count into `0x141A55550`.
- `hknpMalleableConstraintData::vfunction22` at `0x141F5B891`:
  - wrapper path that builds a transformed local context and delegates into the same atom interpreter.

Confirmed conclusion:

- The solver build path consumes the current atom stream each build-jacobian pass.
- If ROCK writes transform A/B, transform-B pivot, angular target, linear motor targets, ragdoll motor targets, tau, damping, and max impulse/force fields before the relevant solve/build phase, the solver should consume those updated values in that pass.

#### Dirty notification evidence

Ghidra-confirmed supporting evidence:

- `0x14153C5A0` has many xrefs from body/motion/filter/material/inertia update paths.
- The inspected xrefs fit body dirtying and broadphase/body-state propagation.
- No inspected evidence shows `0x14153C5A0` is required for ordinary constraint target or motor field changes.

Confirmed conclusion:

- There is currently no binary evidence that per-frame target/motor atom writes require a body dirty call.
- Calling broad body dirtying for every grab target update would be the wrong default until a future binary pass proves otherwise.

Important caveats:

- This conclusion applies to target/motor field updates inside an existing constraint data atom stream.
- It does not apply to structural changes such as changing body ids, changing atom count/layout, changing runtime size, add/remove constraint, or rebuilding the constraint data object.
- Toggling motor enabled state every frame may still be solver-history-disruptive even if the atom offsets are correct.
- The stable design should create the grab constraint once, keep the motor atoms structurally enabled, and update values instead of repeatedly enabling/disabling/recreating.

#### Research conclusion from this audit

The custom-authority architecture should not be rejected because "motors need dirty notifications" unless later evidence proves that. The stronger current explanation for old motor stutter is:

- targets were written from the wrong timing phase;
- body A/contact palm and object B were not updated as one matched physics-phase unit;
- constraint fields were fed from game update instead of the solver-adjacent phase;
- one or more writers affected the same body;
- constraints or motors were toggled/recreated, clearing runtime history;
- body A was a contact collider instead of a hidden no-contact authority proxy rooted in the flattened hand frame.

For the future custom dynamic grab plan, the binary-backed update model is:

1. Keep one persistent custom constraint while held.
2. Keep atom layout and motor enablement structurally stable.
3. During the chosen physics phase, update the hidden proxy motion and the live atom target/motor fields together.
4. Let the normal solver build consume the current atom stream.
5. Avoid body dirty/rebuild calls for ordinary target changes unless a later Ghidra pass proves a specific required notification.

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

## 2026-05-12 continuation: HIGGS adjusted visual hand versus physical hand body timing

Research question:

- During dynamic held-state, does HIGGS drive the constraint body A from the raw controller/real hand, or from the adjusted visual hand that follows the finite-force held object?
- This matters for ROCK because the future custom-authority proxy must reproduce the correct authority split: raw input, physical proxy, visual hand lag/deviation, and object constraint drive.

Source inspected:

- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\hand.h`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\main.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\config.h`

Confirmed functions and fields:

- `Hand::PreUpdateHandsUpdate` (`src/hand.cpp:4462`)
  - Captures animation/world transforms into `fpAnimHandTransform` and `fpAnimWeaponTransform` before Bethesda hand update.
- `Hand::PreUpdate` (`src/hand.cpp:4065`)
  - Runs before `Hand::Update`.
  - Saves `m_handTransformWithoutVrikOffset = handNode->m_worldTransform` before optional VRIK offset is applied.
  - If VRIK offsetting is active, applies the offset to the visual hand through `UpdateHandTransform`.
- `Hand::Update` (`src/hand.cpp:2295`)
  - Saves `m_handTransform = handNode->m_worldTransform` at frame start, after `PreUpdate` may have applied the VRIK offset.
  - Uses this as the real/desired hand pose for held-object deviation checks.
- `Hand::UpdateHandTransform` (`src/hand.cpp:2235`)
  - Updates the first-person hand/clavicle scene transform. This is visual/scene hand transform work, not direct hkp body A velocity work.
- `Hand::ComputeHandCollisionTransform` (`src/hand.cpp:522`)
  - Explicit source comment: the hand transform must be where the hand is in real life because the hand collision drives the grab constraint.
  - Default input is `m_handTransformWithoutVrikOffset`, unless an explicit transform is supplied.
  - Builds the hand collision transform from configured hand-collision offset and real hand rotation.
- `Hand::UpdateHandCollision` (`src/hand.cpp:643`)
  - Disables hand-body collision while `state == State::HeldBody` by OR-ing collision filter bit 14.
  - Registers `handBody` for player-space movement compensation when enabled.
  - Does not itself compute the held-state adjusted visual hand. It manages filter/lifecycle/registration.
- `Hand::MoveHandAndWeaponCollision` (`src/hand.cpp:690`)
  - Drives `handBody` with `ComputeHandCollisionTransform(isBeast)` and `ApplyHardKeyframeVelocityClamped`.
  - Because no explicit transform is passed in the held path, the driven physical hand body uses `m_handTransformWithoutVrikOffset`, not `m_adjustedHandTransform`.
- `Hand::PostUpdate` (`src/hand.cpp:4110`)
  - Calls `UpdateHandCollision(world)` and `UpdateWeaponCollision()` after normal held-state logic.
  - Then removes the VRIK visual offset if needed.
- `Update` in `main.cpp` (`src/main.cpp:626-651`)
  - Runs both hands in this order: `PreUpdate`, `Update`, `PostUpdate`.
- `PlayerPostApplyMovementDeltaUpdate` related path in `main.cpp` (`src/main.cpp:483-496`)
  - Calls `MoveHandAndWeaponCollision(...)` for both hands while applying player movement compensation/delta handling.
  - This is where the HIGGS hand collision body is actually hard-keyframed to the saved real-hand collision transform plus player-space offset.

Confirmed HIGGS held-state visual/deviation loop:

- In `State::HeldBody`, HIGGS computes:
  - `heldTransform = collidableNode->m_worldTransform`
  - `inverseDesired = InverseTransform(desiredNodeTransformHandSpace)`
  - `m_adjustedHandTransform = heldTransform * inverseDesired`
- It then blends `m_adjustedHandTransform` from `m_handTransform` over `startGrabLerpHandDuration` during startup.
- It computes `handDeviation = length(m_adjustedHandTransform.pos - m_handTransform.pos)`.
- `handDeviations` is a rolling deque of 5 samples (`include/hand.h:355`).
- If average deviation exceeds `MaxHandDistance / havokWorldScale` after ignore windows, HIGGS requests idle/drop.
- Otherwise HIGGS calls `UpdateHandTransform(m_adjustedHandTransform)` so the rendered hand follows the object under finite constraint authority.
- After visual hand adjustment, HIGGS updates the live grab constraint:
  - recomputes desired hand/object transform from `desiredNodeTransformHandSpace * GetRigidBodyTLocalTransform(selectedObject.rigidBody)`;
  - writes angular target through `GrabConstraintData::setTargetRelativeOrientationOfBodies`;
  - writes transform-B translation from `desiredHandTransformHavokObjSpace * palmPosHandspace`;
  - updates linear/angular motor force, tau, damping, and recovery fields.

Confirmed timing split:

1. `PreUpdate` records real hand without VRIK offset into `m_handTransformWithoutVrikOffset`.
2. `PreUpdate` may apply VRIK offset to the scene hand.
3. `Update` records the current visual/real desired hand into `m_handTransform`.
4. Held dynamic logic derives `m_adjustedHandTransform` from the held object's actual finite-force pose.
5. HIGGS uses `m_adjustedHandTransform` to move the rendered hand/clavicle toward the object.
6. HIGGS measures deviation between adjusted visual hand and real desired hand and drops if too far.
7. `PostUpdate` updates collision/filter state and removes visual VRIK offset if needed.
8. The physical `handBody` used as constraint body A is driven by `MoveHandAndWeaponCollision`, which uses `ComputeHandCollisionTransform` from `m_handTransformWithoutVrikOffset`, not `m_adjustedHandTransform`.

Important correction to earlier proxy uncertainty:

- HIGGS does not use the adjusted visual hand as body-A authority in the inspected source path.
- HIGGS body A is the real-hand collision body, hard-keyframed from `m_handTransformWithoutVrikOffset`.
- HIGGS still makes the hand appear physically weighted by moving the rendered hand toward the object after solving/observing finite object lag.
- Therefore "virtual/adjusted hand follows object" is a visual/deviation feedback layer, not the body-A transform source.

Why this matters for ROCK:

- Future ROCK custom dynamic grab should keep these as separate roles:
  - raw/root-flattened controller hand frame: desired input authority;
  - hidden no-contact body A/proxy: physical constraint anchor driven from the raw hand frame, with player-space compensation;
  - held object body B: finite-force dynamic body driven by the custom constraint;
  - visual hand adjustment: derived from actual held-object transform and captured desired hand/object relation;
  - deviation release: compares adjusted visual hand against raw/root-flattened hand.
- If ROCK drives the proxy from the adjusted visual hand, it risks feedback-looping object lag back into the constraint anchor and weakening the finite-force effect.
- If ROCK drives only the object toward the raw hand with unlimited or overly high authority and does not adjust the visual hand, the player gets the current "superman" feel: the object obeys too much and the hand never visibly yields.
- The HIGGS-like quality is not just finite motor force. It is finite motor force plus a visual hand lag/deviation layer that admits the object did not reach the real hand.

HIGGS config values relevant to this split:

- `MaxHandDistance = 0.7`
- `physicsGrabIgnoreHandDistanceTime = 0.2`
- `sneakUnsneakIgnoreHandDistanceTime = 0.1`
- `physicsGrabLerpHandTimeMin = 0.1`
- `physicsGrabLerpHandTimeMax = 0.2`
- `physicsGrabLerpHandMinDistance = 0.1`
- `physicsGrabLerpHandMaxDistance = 0.2`
- `grabConstraintLinearMaxForce = 2000`
- `grabConstraintLinearMaxForceWeapon = 9000`
- `grabConstraintAngularToLinearForceRatio = 12.5`
- `grabConstraintMaxForceToMassRatio = 500`
- `grabConstraintCollidingAngularTau = 0.01`
- `grabConstraintCollidingLinearTau = 0.01`
- `grabConstraintTauLerpSpeed = 0.5`
- `grabConstraintFadeInStartAngularMaxForceRatio = 100`
- `grabConstraintFadeInTime = 0.1`

FO4VR design implication, not implementation:

- The future hidden dynamic-grab body A should still be rooted in the current flattened hand frame, matching the user's clarification that the generated palm body is created from the root-flattened hand-frame bone tree.
- The proxy should be no-contact / collision-suppressed during held dynamic grab, equivalent to HIGGS bit-14 suppression on `handBody` in `State::HeldBody`.
- The visual hand adjustment should be a separate ROCK layer, probably consuming:
  - captured `desiredNodeTransformHandSpace` / equivalent grab frame;
  - actual held object body/node transform;
  - raw flattened hand pose;
  - startup hand-lag lerp duration from capture distance;
  - rolling deviation average and release threshold.
- The physical constraint should not use the adjusted visual hand as its anchor unless a later FO4VR-specific design intentionally differs from HIGGS and proves the feedback loop is stable.

Open questions after this HIGGS timing pass:

- ROCK currently uses FRIK/root-flattened frames and generated colliders; HIGGS directly mutates the first-person hand/clavicle scene transform. Need map ROCK's visual hand adjustment options before copying the visual feedback concept.
- Need verify whether ROCK already has a visual hand deviation path separate from collision/proxy driving, and if so whether it is fed from actual held-object lag or only from target error.
- Need map where FO4VR can safely publish adjusted hand visuals without corrupting FRIK's next root-flattened sample or PAPER/weapon visual state.
- Need continue Ghidra/source research on FO4VR hknp proxy body lifecycle and direct hard-keyframe velocity command usage before any implementation plan.

## 2026-05-12 continuation: ROCK existing visual hand lag/deviation layer

Research question:

- Does ROCK already have the HIGGS-style rendered-hand feedback layer, or would that need to be designed from scratch when replacing mouse-spring authority with custom finite motors?

Source inspected:

- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/hand/HandVisual.h`
- `src/physics-interaction/hand/Hand.h`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
- `src/physics-interaction/core/PhysicsInteractionFrame.inl`
- `src/physics-interaction/hand/HandSkeleton.h`
- `src/api/FRIKApi.h`
- `E:\fo4dev\PROJECT_ROCK_V2\Fallout-4-VR-Body(FRIK)\src\api\FRIKApi.cpp`
- `E:\fo4dev\PROJECT_ROCK_V2\Fallout-4-VR-Body(FRIK)\src\skeleton\Skeleton.cpp`
- `E:\fo4dev\PROJECT_ROCK_V2\Fallout-4-VR-Body(FRIK)\src\FRIK.cpp`
- `E:\fo4dev\PROJECT_ROCK_V2\Fallout-4-VR-Body(FRIK)\src\pipboy\Pipboy.cpp`

Confirmed ROCK visual hand pieces:

- `HandVisual.h`
  - Explicitly states these helpers are visual interpolation only.
  - `buildHeldObjectRelativeHandWorld(heldObjectWorld, frozenObjectHandSpace)` computes the HIGGS-shaped relation:
    - adjusted/visual hand = live held object world transform * inverse(frozen object-in-hand transform).
  - `advanceTransform(...)` lerps position by speed and rotation by slerp/angular speed.
  - Helpers do not feed object targets, pivots, collision bodies, or rotation authority.
- `HandGrab.cpp:1535-1555`
  - Defines external visual tag `ROCK_GrabVisual`.
  - Applies it through FRIK `applyExternalHandWorldTransform`.
  - Clears it through FRIK `clearExternalHandWorldTransform`.
  - Priority is 90.
- `HandGrab.cpp:3790-3840`
  - On grab-frame capture, clears any previous `ROCK_GrabVisual`, initializes `_grabVisualHandTransform` from the current raw hand transform, clears visual deviation timers, and records whether a large initial sync should fade motor force.
- `HandGrab.cpp:4496-4602`
  - Visual adjustment is active when:
    - `_grabFrame.hasTelemetryCapture`, and
    - phase is `TouchHeld`, or the object is in near/gravity converge and within the acquisition visual envelope.
  - It resolves live held object visual/body-derived world transform.
  - Computes `targetVisualHandWorld = buildHeldObjectRelativeHandWorld(heldVisualNodeWorld, _grabFrame.rawHandSpace)`.
  - Preserves hand scale from current raw hand transform.
  - Lerps `_grabVisualHandTransform` toward target if `rockGrabHandLerpEnabled`.
  - Computes visual hand deviation against the raw hand transform.
  - Accumulates `_grabVisualDeviationExceededSeconds` through `held_object_physics_math::advanceDeviationSeconds`.
  - Releases if deviation exceeds `rockGrabMaxDeviation` for `rockGrabMaxDeviationTime`.
  - Publishes the visual hand through `applyGrabExternalHandWorldTransform(_isLeft, _grabVisualHandTransform)`.
  - Clears the external transform when no held visual/body transform is available or when the phase is not eligible.
- `Hand.h`
  - Stores `_grabVisualHandTransform`, `_hasGrabVisualHandTransform`, and `_grabVisualDeviationExceededSeconds`.
- `Hand.cpp` reset/world-loss paths
  - Clear `ROCK_Grab` pose and `ROCK_GrabVisual` external transform on reset and world-loss abandon.
- `HandGrab.cpp` release path
  - Clears `ROCK_GrabVisual` on release.

Confirmed ROCK raw/root hand frame source:

- `HandBoneCache::resolve` in `HandSkeleton.h`
  - Captures `HandsAndForearmsOnly` from `GameRootFlattenedBoneTree`.
  - Finds `RArm_Hand` and `LArm_Hand`.
  - Stores copied transforms, not node pointers.
- `PhysicsInteraction::refreshHandBoneCache`
  - Refreshes the cache once per update before frame context construction.
- `PhysicsInteractionFrame.inl`
  - `buildFrameContext` consumes the copied root-flattened hand transforms via `getInteractionHandTransform`.
  - Builds `rawHandWorld`, `grabAnchorWorld`, palm normal, and pointing direction from that coherent frame.
- `PhysicsInteraction::update`
  - Calls `refreshHandBoneCache`, `sampleHandTransformParity`, then `buildFrameContext`.
  - Later calls `updateGrabInput(frame)` with the already-built raw hand frame.
  - Therefore current-frame grab physics/math consumes the prebuilt raw/root-flattened hand snapshot, not the external visual hand transform applied later during `updateHeldObject`.

Confirmed FRIK external hand authority behavior:

- `FRIKApi.cpp`
  - External hand transforms are stored per hand as tagged entries with priority and generation.
  - Highest priority wins; if priority ties, newest generation wins.
  - `apiApplyExternalHandWorldTransform` applies the selected target immediately if the caller's tag wins.
  - `apiClearExternalHandWorldTransform` restores tracked hand authority if no other tag remains, or applies the next selected authority.
- `Skeleton::applyExternalHandWorldTransform`
  - Rejects non-finite transforms.
  - Restores the arm chain to FRIK baseline.
  - Runs full-arm IK to the external hand target.
  - Excludes the first-person weapon child while updating the primary-hand hierarchy so weapon ownership does not get reapplied accidentally.
  - Returns whether the resulting hand transform is finite.
- `FRIK::refreshAfterExternalHandAuthority`
  - Reapplies current hand pose and final world update after external authority.
- `Pipboy::syncAfterExternalHandAuthority`
  - Clears/stabilizes Pip-Boy movement dampening state when the external authority affects the Pip-Boy arm.

Priority/context findings:

- ROCK dynamic grab visual hand priority: `ROCK_GrabVisual = 90`.
- ROCK equipped/two-hand weapon external hand priority: `ROCK_WeaponPrimaryGrip` / `ROCK_WeaponSupportGrip = 100`.
- Soft-contact visual priority defaults to 80 and is clamped to 0..99.
- Therefore loose dynamic grab visual lag should lose to equipped/two-hand weapon authority and beat soft-contact visual authority.

Important parity conclusion:

- ROCK already has the HIGGS-shaped visual hand lag/deviation layer.
- It is explicitly visual-only and does not feed body A, mouse spring, constraint pivots, object orientation, or motor targets.
- This matches the corrected HIGGS timing split: raw hand drives the physical anchor, actual held object pose drives rendered hand lag/deviation.

Why ROCK can still feel "superman" despite this layer:

- The visual layer can only show lag if the held object actually lags behind the raw hand target.
- In the current ordinary one-hand path, body B is driven by native mouse spring with static response/lead tuning.
- If native mouse spring authority is too strong, the held object reaches the raw target too easily; `targetVisualHandWorld` stays close to `handWorldTransform`, so the FRIK visual layer has little visible yielding to show.
- The HIGGS-quality weight feel therefore still depends on replacing or governing the physical drive authority, not on adding a visual-hand system from scratch.

Current ROCK visual-layer gaps to track:

- The visual layer is gated by `TouchHeld` or near/gravity converge within acquisition visual envelope. HIGGS updates adjusted visual hand throughout `HeldBody` once dynamic held state is valid. ROCK likely reaches `TouchHeld` for true held contact, but this phase gate should be audited under far/pull/heavy-object cases.
- Deviation timing differs:
  - HIGGS uses a rolling average of 5 hand-deviation samples and explicit ignore windows.
  - ROCK uses accumulated seconds over threshold through `advanceDeviationSeconds`.
  - This may be fine, but it is not 1:1 and should be reviewed as a FO4VR design choice.
- `rockGrabHandLerpTimeMin/Max` exist in config, but the inspected visual update currently calls `advanceTransform` with `rockGrabLerpSpeed` and `rockGrabLerpAngularSpeed`. Need verify whether the min/max hand lerp settings are used elsewhere or are stale.
- Because FRIK refreshes final world state immediately after external hand authority, the next root-flattened sample could include the externally solved pose depending on frame order. Current ROCK frame context is built before applying `ROCK_GrabVisual`, so current-frame grab math is safe. Need verify next-frame FRIK normal update order before assuming no feedback across frames.
- Future custom proxy/constraint authority should continue consuming the prebuilt raw/root-flattened frame, not FRIK `getHandWorldTransform` after visual authority has been applied.

Updated design implication:

- Do not design a new visual hand lag system first. The existing one is close to the HIGGS pattern.
- The future custom motor/proxy research should focus on making body B lag believably under finite force and inertia while preserving this visual layer.
- The proxy should be driven from the raw/root-flattened frame captured before external visual writes.
- The visual layer should keep deriving from the actual held object transform, not from desired target transform, otherwise it cannot show physical yield.

## 2026-05-12 Ghidra continuation: FO4VR body lifecycle and direct velocity command bridge

Research question:

- If ROCK creates a future hidden no-contact body-A proxy for custom dynamic grab authority, what FO4VR native paths create, add, drive, remove, and destroy bodies?
- Which paths are remove-only versus full destroy?
- Is direct hard-keyframe velocity usable from physics/listener context, or does it require a command payload?

Binary inspected:

- Ghidra MCP on FO4VR runtime loaded in the current Ghidra project.
- User-provided executable reference if needed: `E:\fo4dev\reverse_engineering\Fallout4VR.exe.unpacked.exe`.

### Body creation / add path

Ghidra-confirmed functions:

- `0x141546350`
  - Allocates/initializes an hknp motion entry from a motion cinfo-like structure.
  - Uses the world motion allocator at `world + 0xD8`.
  - Writes/initializes a motion slot at `world[0xE0] + motionId * 0x80`.
  - Grows/updates the world motion metadata array at `world + 0x4A0`.
  - Emits a motion-created signal through `world + 0x500`.
- `0x141543FF0`
  - Creates an hknp body slot from body cinfo-like data.
  - Allocates or reuses a body id from the world body allocator at `world + 0x10`.
  - Initializes the body slot at `world[0x20] + bodyId * 0x90`.
  - If the body has motion data, calls the motion link/setup path at `0x1415486A0`.
  - Emits body-created callbacks/signals through `world + 0x4D0`.
  - If `param_4 != 0`, calls `0x1415441F0` to add the body to active/broadphase world state.
- `0x1415486A0`
  - Links a body slot to a motion slot.
  - Writes body `+0x68` / `+0x6C`-style motion linkage fields.
  - Updates motion/body linked-list membership.
  - Recomputes motion/body inertia/state fields and dirty/signals dependent body state.
- `0x1415441F0`
  - Adds bodies to the live broadphase/active world state.
  - Emits profiling markers `LtAddBodies`, `StSetup`, `StAddToBroadPhase`, and `StFireCallbacks`.
  - Updates active-body index tables at `world + 0x98` / `world + 0xA8`.
  - Calls broadphase add through world vtable paths at `world + 0x178`.
  - Emits add callbacks/signals, including through `world + 0x4D8`.

Interpretation:

- A future dedicated proxy should not be made by only fabricating a body slot; it needs the full create + motion + add path or the existing `BethesdaPhysicsBody::create` wrapper that already exercises the equivalent engine route.
- The previous design preference for a separate hidden body-A proxy remains viable, but lifecycle must be owned as a real hknp body with a real remove/destroy path, not as a borrowed generated collider.

### Remove versus destroy path

Ghidra-confirmed functions:

- `0x141544B00`
  - Body remove path.
  - Fires `TtFireBodyRemoved`.
  - Walks removed-body listeners/signals at `world + 0x4E0`.
  - Removes body from active/island/broadphase tables.
  - Clears active/motion linkage state, including body `+0x6C = -1`.
  - Marks body flags with `& 0xfffbffff | 0x400`.
  - Calls `0x14154AA50` for motion/body unlink cleanup.
  - Does not release the body shape reference or free the body id as a destroyed/free body.
- `0x141544E80`
  - Body destroy path.
  - Acquires world lock/scheduler state, then calls `0x141544B00`.
  - Fires `TtFireBodyDestroyed` through listeners/signals at `world + 0x4E8`.
  - Releases shape/mass-properties reference from body `+0x48`.
  - Clears low body flags at `+0x40`.
  - Pushes the body id back to the free/dead body-id table at `world + 0x98` / `world + 0xA8`.
  - Increments world destroy/change counter at `world + 0x6C`.
  - Flushes body-destroy bookkeeping through `world + 0x4A0`.

Caller evidence:

- `0x141544E80` is called by:
  - `~hknpPhysicsSystem` at `0x1415655CE`;
  - a bhk/collision-object replacement path around `0x1412CE290`;
  - `hknpBallGun::vfunction6` at `0x142032F6F`, where old temporary ball bodies are destroyed after ring-buffer expiry.
- `0x141544B00` is called by:
  - `0x141544E80`;
  - `0x141565A00`, which removes all bodies in a physics system-like container;
  - `bhkPhysicsSystem::vfunction44` at `0x141E0D5C0`, which removes bodies and tears down constraints that reference removed body ids.

Interpretation:

- Remove-only and destroy are not equivalent.
- A persistent proxy that may be reused within one world can be removed if the wrapper expects reuse, but final lifecycle cleanup must reach the destroy path or a Bethesda wrapper that does.
- If ROCK creates a new proxy per grab/world lifecycle, final cleanup must not stop at `0x141544B00`; otherwise shape refs/body ids can leak or remain in stale free/active state.
- Constraint cleanup must happen before or during remove. `bhkPhysicsSystem::vfunction44` explicitly scans constraints for removed body ids and calls constraint removal (`0x141546C60`) before removing bodies.

### Direct hard-keyframe velocity path

Ghidra-confirmed functions:

- `0x14153A6A0`
  - Computes hard-keyframe linear and angular velocity from:
    - target position;
    - target rotation/quaternion;
    - current body/motion transform;
    - `deltaSeconds`-style time value.
  - It computes reciprocal `1 / param_5`; this means the wrapper's time parameter is delta time, not pre-inverted dt.
  - Output is linear velocity and angular velocity vectors.
  - It uses the current hknp body slot and motion slot, so the body must be a valid live dynamic/keyframed body with motion.
- `0x141539F30`
  - Direct set-body-velocity path.
  - Reads the body slot and linked motion slot.
  - Wakes/marks body if needed through `0x141546EF0`.
  - Writes linear velocity to motion fields around `motion + 0x10`.
  - Writes angular velocity-derived fields around `motion + 0x14..0x17`/derived rotation velocity fields.
  - Calls `0x14153D440`.
  - Emits velocity/body-change signal through `world + 0x538`.
- `0x141DF5930`
  - Wrapper: compute hard-keyframe velocity through `0x14153A6A0`, then apply it.
  - If TLS byte `+0x1528` is false, calls `0x141539F30` directly.
  - If TLS byte `+0x1528` is true, queues a command through `0x141DFC8D0`.
  - Queued velocity command uses:
    - command size/type field `0x30`;
    - command id/flags `0x90100`;
    - body id;
    - computed linear and angular velocities.
  - If either computed velocity is nonzero beyond tiny thresholds, calls `0x141DF60C0` after the write/queue.
- `0x141DF56F0`
  - Direct set-body-velocity wrapper.
  - Same direct-versus-queued TLS branch as `0x141DF5930`.
  - Queued command is also `0x30 / 0x90100`.
- `0x141DF55F0`
  - Set-body-transform wrapper.
  - If TLS byte `+0x1528` is false, calls direct transform write `0x1415395E0`.
  - If TLS byte `+0x1528` is true, queues command:
    - command size/type field `0x50`;
    - command id/flags `0x60100`;
    - body id;
    - transform payload;
    - transform-mode/int parameter.
- `0x141DF60C0`
  - Post-write/post-queue activation or dirty/wakeup helper.
  - Validates body id and active/body flags.
  - If body is not already in the desired active state and a flag at body `+0x40 >> 0x12` is set, queues/records the body id in a world-side set around `world + 0x6E8`.

Interpretation:

- The direct hard-keyframe velocity wrapper is a better proxy-drive candidate than `DriveToKeyFrame` because it does not include the verified DriveToKeyFrame cap-triggered transform snap branch.
- It is still not a magic "set target transform" authority. It computes and writes velocities from current body state toward a target for the current dt.
- In a physics-step listener/command context, the wrapper may queue velocity command `0x30 / 0x90100` rather than writing immediately, depending on TLS `+0x1528`.
- The previously mapped between-collide-and-solve drain means queued commands can be consumed before solve if they are queued into the active local command list. However, this pass still does not prove TLS `+0x1528` is set in ROCK's callback context. That remains a required verification point before implementation.

### Command bridge / queue path

Ghidra-confirmed functions:

- `0x141DFC8D0`
  - Shared command enqueue bridge.
  - Reads TLS `+0x152C` to select a per-thread command staging area at `DAT_1465A3E30 + threadIndex * 0xA8`.
  - Handles command type/size variants including:
    - `0x50 / 0x60100` set-transform payload;
    - `0x30 / 0x90100` set-velocity payload;
    - other command groups `6`, `7`, `9`, `0x12`, and `0x2C` that stage different payload sizes and references.
  - Links the command list for the target world.
- `0x141DFDE70`
  - Command drain/dispatch function.
  - If exactly one command exists and its command-list metadata indicates immediate/direct (`second qword == 1`), dispatches through command vtable `+0x20`.
  - Otherwise builds a work batch, starts worker dispatch, waits, then clears command refs.
  - Clears command list count/state after execution and releases referenced command objects.

Interpretation:

- Future proxy drive from a physics listener should prefer an engine wrapper that is correct in both direct-write and queued contexts.
- If ROCK bypasses the wrapper and writes velocity fields manually, it also bypasses the engine's dirty/signal/activation path at `0x141539F30` and `0x141DF60C0`.
- If ROCK calls the wrapper from a context where TLS queue state is active but not linked to the local pre-solve command list, the command may not be applied before the constraint solve. This is the remaining timing risk.

### Updated proxy-drive conclusion

Binary-backed requirements for a future hidden no-contact body-A proxy:

1. Create it through a full body/motion/add lifecycle path or through ROCK's existing `BethesdaPhysicsBody::create` wrapper if that wrapper is verified to call the same native chain.
2. Keep it persistent while the dynamic grab is held; do not create/destroy every frame.
3. Drive it from the raw/root-flattened hand frame through a velocity path, not by transforming the held object or using COM as authority.
4. Prefer the direct hard-keyframe velocity semantics (`0x141DF5930` / `0x14153A6A0` / `0x141539F30`) over DriveToKeyFrame for hidden proxy research, because DriveToKeyFrame can snap transforms when caps are exceeded.
5. Use set-transform only for creation/initial placement or verified warp cases, not as the steady held proxy drive.
6. Keep remove/destroy ownership explicit. Final proxy cleanup must reach the destroy-equivalent path, not only remove.
7. Do not assume queued wrapper calls made from ROCK's future between-collide-and-solve callback apply before solve until TLS command-list state is verified in that callback context.

Unresolved after this Ghidra pass:

- Verify whether `BethesdaPhysicsBody::create`, `reset`, and destruction paths in ROCK exactly map to the create/add/remove/destroy paths above for a generated standalone body.
- Verify whether `0x141DF5930` is directly callable from ROCK with stable typed parameters, or whether ROCK should compute velocity via a safer local wrapper around `0x14153A6A0` + `setBodyVelocityDeferred`.
- Verify TLS `+0x1528` / `+0x152C` state inside `PhysicsStepDriveCoordinator` callbacks, especially the future between-collide-and-solve slot.
- Verify whether direct velocity command application before solve updates the proxy body's transform early enough for the constraint solver's body-A frame, or whether the constraint target should be computed against predicted proxy transform instead of current body transform.
- Verify whether a no-contact proxy should use bit-14 suppression, layer 15, or a ROCK-owned extended layer, now that lifecycle and drive are clearer.

## 2026-05-12 continuation: ROCK `BethesdaPhysicsBody` wrapper against Ghidra lifecycle

This pass reconciles the existing ROCK generated-body wrapper with the Ghidra-confirmed hknp body/motion/add/remove paths above. The goal is not implementation. The goal is to decide what parts of the existing wrapper are a reliable future body-A proxy foundation, and what parts are unsuitable for steady custom dynamic grab authority.

### Source files inspected

- `src/physics-interaction/native/BethesdaPhysicsBody.h`
- `src/physics-interaction/native/BethesdaPhysicsBody.cpp`
- `src/physics-interaction/native/HavokOffsets.h`
- `src/physics-interaction/native/HavokRuntime.cpp`
- `src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp`

### Current ROCK wrapper creation path

`BethesdaPhysicsBody::create(...)` is a full generated-body wrapper, not a shallow hknp body-slot write.

Confirmed source behavior:

- Requires:
  - `RE::hknpWorld* world`;
  - `bhkWorld`;
  - non-null `RE::hknpShape* shape`;
  - filter info;
  - material id;
  - `BethesdaMotionType`;
  - optional debug/name string.
- Allocates `hknpPhysicsSystemData` as `0x78` bytes.
- Calls `offsets::kFunc_PhysicsSystemData_Ctor = 0x5EAB0`.
- Appends:
  - motion cinfo entry at system data `+0x30` for non-static bodies, stride `0x70`;
  - body cinfo entry at system data `+0x40`, stride `0x60`;
  - material entry at system data `+0x10`, stride `0x50`;
  - shape ref entry at system data `+0x60`, stride `8`.
- Body cinfo fields written by ROCK:
  - shape pointer at `+0x00`;
  - invalid generated id at `+0x08`;
  - generated local motion index at `+0x0C`;
  - quality id at `+0x10`;
  - local material index at `+0x12`;
  - collision filter info at `+0x14`;
  - name pointer at `+0x20`;
  - user data / extra pointer at `+0x28` set to zero.
- Adds a ref to the shape when recording the shape slot.
- Allocates `bhkPhysicsSystem` as `0x28` bytes.
- Calls `offsets::kFunc_PhysicsSystem_Ctor = 0x1E0C2B0`.
- Allocates `bhkNPCollisionObject` as `0x30` bytes under Bethesda allocator TLS context `0x41`.
- Calls `offsets::kFunc_CollisionObject_Ctor = 0x1E07710`.
- Creates and links a NiNode owner.
- Calls `offsets::kFunc_CollisionObject_AddToWorld = 0x1E07BE0`.
- Calls `offsets::kFunc_PhysicsSystem_GetBodyId = 0x1E0C460`.
- Validates generated body motion.
- Applies generated material.
- Applies generated body motion type.
- Calls `havok_runtime::setFilterInfo(world, bodyId, filterInfo, 1)`.
- Calls `offsets::kFunc_EnableBodyFlags = 0x153C090` with flags `0x08020000`, mode `1`.
- Activates the body.
- Reads a snapshot and verifies body `+0x88` collision-object back-pointer equals the wrapper collision object.

Ghidra cross-check:

- `0x141E07BE0` is `bhkNPCollisionObject::vfunction49`.
- It ensures the collision object has a native physics system instance, attaches or moves it to the current bhkWorld's hknp world, obtains body id through `0x141E0C460`, and writes the collision-object back-pointer into the hknp body slot at body `+0x88`.
- This matches ROCK's post-create back-pointer verification.

Interpretation:

- `BethesdaPhysicsBody::create(...)` is currently the safest known production-style path for a generated proxy body because it exercises Bethesda's collision object and physics-system ownership path instead of only fabricating hknp slots.
- The wrapper still requires a real shape. A future hidden no-contact proxy therefore needs a minimal valid shape policy; it should not be shape-less unless a new Ghidra-backed path proves a shape-less body is valid in FO4VR.
- The wrapper's creation path can accept a no-contact filter at birth. It already writes creation-time filter info after add-to-world with rebuild mode `1`.
- This supports a separate hidden proxy body design more than reusing the visible palm/collider body, because the wrapper can create a real body that is not published as hand contact metadata and can carry its own filter policy.

### Current ROCK wrapper destruction path

`BethesdaPhysicsBody::destroy(void* bhkWorld)`:

- returns if no created/collision/ni node state exists;
- fetches `nativePhysicsSystemInstance(_physicsSystem)`;
- if `bhkWorld` and instance are valid, calls `offsets::kFunc_BhkWorld_RemovePhysicsSystemInstance = 0x1DFAD00`;
- destroys the owner NiNode;
- releases the collision object;
- resets local pointers/body id/created flag.

Ghidra cross-check:

- `0x141DFAD00`:
  - obtains native hknp world from `bhkWorld + 0x60`;
  - if TLS byte `+0x1529` is false and world exists, locks/synchronizes around world `+0x6D8`;
  - calls `0x141565A00(param_2)`;
  - unlocks after removal.
- Earlier Ghidra mapping of `0x141565A00`:
  - removes all bodies in the physics system-like container;
  - reaches the body remove path `0x141544B00`;
  - `bhkPhysicsSystem::vfunction44` also scans constraints referencing removed body ids and removes those constraints before removing bodies.
- The wrapper then releases the collision object. The release path is outside this pass, but the native physics-system instance removal does not by itself prove shape/body-id destruction; final release/refcount behavior still depends on the collision object / physics system ownership chain.

Interpretation:

- `BethesdaPhysicsBody::destroy(...)` is still the preferred current cleanup route over direct body remove because it goes through Bethesda's physics-system-instance removal and collision-object ownership.
- A future proxy must ensure any custom constraint using the proxy body is removed before or as part of this destruction path. The Ghidra remove path is constraint-aware in the physics-system path, but relying on late cleanup is still a bad ownership boundary.
- The remaining unresolved point is whether wrapper destruction always reaches the hknp destroy-equivalent cleanup for generated system bodies after refcount release. Current evidence is strong enough to prefer the wrapper, but not strong enough to bypass explicit constraint cleanup or to create/destroy proxies per frame.

### Current ROCK wrapper drive methods

`BethesdaPhysicsBody` exposes:

- `driveToKeyFrame(target, dt)`:
  - calls `offsets::kFunc_CollisionObject_DriveToKeyFrame = 0x1E086E0`.
- `setTransform(transform)`:
  - calls `offsets::kFunc_CollisionObject_SetTransform = 0x1E08A70`.
- `setVelocity(linVel, angVel)`:
  - calls `offsets::kFunc_CollisionObject_SetVelocity = 0x1E082A0`.
- `setPointVelocity(targetVel, worldPoint)`:
  - explicitly ignores `worldPoint`;
  - calls `setVelocity(targetVel, zeroAngular)`;
  - source comment says native point-specific velocity still needs binary validation.

Ghidra cross-check:

- `0x141E082A0`:
  - validates native physics system through `0x141E0C530`;
  - obtains body id through `0x141E0C460`;
  - obtains hknp world through `0x141E0C4E0`;
  - calls world/body velocity wrapper `0x141DF56F0(world, bodyId, linear, angular)`.
- `0x141E08A70`:
  - validates native physics system;
  - obtains body id and hknp world;
  - calls world/body transform wrapper `0x141DF55F0(world, bodyId, transform, 1)`.
- `0x141E086E0`:
  - sets Bethesda allocator TLS context `0x41` during the call;
  - validates native physics system and positive dt;
  - converts the target transform through `0x141722C10`;
  - obtains body id, body slot, motion array, and motion properties;
  - calls hard-keyframe velocity compute `0x14153A6A0`;
  - compares computed linear/angular velocity magnitudes against motion-property caps at motion-properties `+0x10` and `+0x14`;
  - if either cap is exceeded, calls set transform `0x141DF55F0(..., mode 1)` and then set velocity `0x141DF56F0(..., zero, zero)`;
  - otherwise calls set velocity `0x141DF56F0(..., computedLinear, computedAngular)`;
  - returns success value `0x901` on normal positive-dt path.
- `0x141DF5930`:
  - computes hard-keyframe velocity with `0x14153A6A0`;
  - writes or queues the resulting velocity through direct body velocity function/command;
  - calls activation/dirty helper when nonzero;
  - does not contain the cap-triggered transform snap branch seen in `0x141E086E0`.

Interpretation:

- The existing wrapper is suitable for lifecycle, but its current steady drive options are not yet the right custom-authority proxy drive:
  - `driveToKeyFrame` can snap transforms if motion-property caps are exceeded.
  - `setTransform` is a warp/placement tool, not steady solver-friendly authority.
  - `setVelocity` is a raw velocity writer and does not compute target-transform-to-velocity correction.
  - `setPointVelocity` is not point-specific despite its name and must not be used for off-COM authority.
- The direct hard-keyframe velocity wrapper at `0x141DF5930` remains the cleaner research target for future proxy drive because it gives FO4VR's target-transform-to-velocity math without DriveToKeyFrame's cap-triggered transform snap.
- `BethesdaPhysicsBody` does not currently expose `0x141DF5930`. That is a future implementation-plan item only after the timing/TLS questions are resolved.

### Relationship to generated hand/body colliders

Current generated hand/body/weapon collision code uses `BethesdaPhysicsBody::create(..., Keyframed)` and `driveGeneratedKeyframedBody(...)`.

`driveGeneratedKeyframedBody(...)`:

- selects an immediate placement target only for pending teleports / hard sync;
- otherwise converts target to Havok transform and calls `body.driveToKeyFrame(targetHavok, driveDelta)`;
- logs native drive failure but does not avoid the native DriveToKeyFrame snap branch;
- currently ignores its `maxLinearVelocityHavok` and `maxAngularVelocityRadians` parameters at the top of the function.

Interpretation:

- Reusing the generated collider drive loop as-is for a hidden authority proxy would inherit DriveToKeyFrame's cap-triggered transform snap behavior.
- This does not make current hand collision wrong; it explains why the hidden authority proxy should have its own drive semantics even if it reuses `BethesdaPhysicsBody` creation/destruction.
- The generated visible collider path and the future hidden authority proxy path should be separated:
  - visible colliders: contact evidence, detection, gameplay collision, and debug/telemetry;
  - hidden proxy: no-contact body-A authority for custom finite-force linear/angular constraint.

### Updated body-A proxy conclusion

Current best-supported future design shape, pending more Ghidra timing checks:

1. Create a separate persistent `BethesdaPhysicsBody` for the dynamic grab authority proxy.
2. Use the same root-flattened hand frame source that currently creates the generated palm body.
3. Give the proxy its own no-contact filter policy at creation time.
4. Do not publish the proxy as hand contact metadata.
5. Do not drive the proxy with existing `driveGeneratedKeyframedBody(...)`.
6. Do not use `setPointVelocity(...)`; it is not point-specific.
7. Research exposing or wrapping `0x141DF5930` for target-transform-to-velocity proxy drive.
8. Keep the proxy persistent over a held grab, then remove custom constraints before destroying it.
9. Use set-transform only for initial placement/teleport, not steady held drive.

### Confirmed facts from this pass

- ROCK's current generated-body wrapper uses Bethesda's collision object + physics system path and validates body `+0x88` back-pointer.
- `0x141E07BE0` writes the collision-object back-pointer to body `+0x88`.
- `BethesdaPhysicsBody::driveToKeyFrame(...)` maps to `0x141E086E0`.
- `0x141E086E0` contains a motion-cap-triggered transform snap branch.
- `BethesdaPhysicsBody::setVelocity(...)` maps to `0x141E082A0`, which calls `0x141DF56F0`.
- `BethesdaPhysicsBody::setTransform(...)` maps to `0x141E08A70`, which calls `0x141DF55F0`.
- `BethesdaPhysicsBody::setPointVelocity(...)` currently ignores the point and zeroes angular velocity.
- `driveGeneratedKeyframedBody(...)` still calls `driveToKeyFrame(...)` for normal steady generated collider motion.

### Unresolved after this pass

- Verify whether `0x141DF5930` can be safely called from ROCK with a direct wrapper around `world`, `bodyId`, `targetPosition`, `targetQuaternion/rotation`, and `dt`.
- Verify exact parameter convention of `0x141DF5930` versus `0x14153A6A0` using disassembly if needed, especially whether the rotation input expects quaternion, transform rotation rows, or the converted target payload from `0x141722C10`.
- Verify TLS `+0x1528/+0x152C` state inside the existing physics step listener callbacks.
- Verify whether pre-solve velocity writes update body transforms early enough for the same solver step, or whether the custom constraint should use predicted proxy transform as its target frame.
- Verify final generated-body destroy ownership through collision object / physics system refcount release if the future proxy creates many bodies across world reloads.

## 2026-05-12 continuation: ApplyHardKeyFrame / ComputeHardKeyFrame convention map

This pass follows the user's note that `ApplyHardKeyFrame` may be relevant. It is relevant, but it must be treated precisely: it is a native FO4VR body velocity application path, not the grab model itself and not a replacement for finite-force held-object motors.

### Names and addresses

Binary-confirmed functions:

- `0x14153A6A0`
  - Proposed name: `hknpWorld::ComputeHardKeyFrame`.
  - Role: compute linear and angular velocity required to move one live body toward a target position/rotation over `dt`.
- `0x14153ABD0`
  - Proposed name: `hknpWorld::ApplyHardKeyFrame`.
  - Role: lock world/body access, call `ComputeHardKeyFrame`, then call direct set-body-velocity.
- `0x141DF5930`
  - Proposed name: deferred/command-safe `ApplyHardKeyFrame` wrapper.
  - Role: call `ComputeHardKeyFrame`, then either direct-write velocity or queue velocity command `0x30 / 0x90100` depending on TLS command state.
- `0x141E086E0`
  - Existing wrapper target: `bhkNPCollisionObject::DriveToKeyFrame`.
  - Role: compute hard-keyframe velocity, but snap target transform if motion-property velocity caps are exceeded.
- `0x141722C10`
  - Proposed name: transform/matrix-to-quaternion helper.
  - Role: converts a transform rotation matrix payload into quaternion xyzw-like float4.

Secondary local header evidence:

- `libraries_and_tools/CommonLibF4VR/CommonLibF4/include/RE/Havok/hknpWorld.h` names:
  - `ComputeHardKeyFrame(...)`;
  - `ApplyHardKeyFrame(...)`;
  - comments target rotation as quaternion `xyzw`.
- This header is not used as authority because CommonLibF4VR can be wrong. It is recorded only because its naming matches the independent Ghidra behavior.

### `0x14153ABD0` direct ApplyHardKeyFrame

Decompiled shape:

```text
lock world/body access at world + 0x690
ComputeHardKeyFrame(world, bodyId, targetPos, targetRotQuat, dt, localLinVel, localAngVel)
SetBodyVelocity(world, bodyId, localLinVel, localAngVel)
unlock world/body access
```

Confirmed behavior:

- Calls `0x141538AF0(world + 0x690)` before the compute/write.
- Calls `0x14153A6A0(...)`.
- Calls `0x141539F30(...)` direct velocity write.
- Calls `0x141538CB0(world + 0x690)` after the write.
- Has no TLS queued-command branch.
- Has no DriveToKeyFrame cap-triggered transform snap branch.

Interpretation:

- This is the clean native "apply hard keyframe velocity now" path.
- It is probably the function users and local headers call `ApplyHardKeyFrame`.
- It is not automatically safe from every ROCK callback context because it locks directly and does not route through the command bridge.
- It may be appropriate when the caller is in a normal game-thread/non-command context. For a physics listener context, the command-safe wrapper may be safer if TLS state is active and drains before solve.

### `0x141DF5930` deferred/command-safe ApplyHardKeyFrame wrapper

Confirmed behavior:

- Calls `0x14153A6A0(world, bodyId, targetPos, targetRotQuat, dt, localLinVel, localAngVel)`.
- If TLS byte `+0x1528` is false:
  - calls `0x141539F30(world, bodyId, localLinVel, localAngVel)` directly.
- If TLS byte `+0x1528` is true:
  - builds velocity command:
    - command size `0x30`;
    - command id/flags `0x90100`;
    - body id;
    - computed linear velocity;
    - computed angular velocity;
  - enqueues with `0x141DFC8D0`.
- If computed velocity is nonzero beyond small thresholds:
  - calls activation/dirty helper `0x141DF60C0(world, bodyId)`.
- Has no DriveToKeyFrame cap-triggered transform snap branch.

Known native callers:

- `0x1412CB4E0`
  - Appears to update a camera or camera-attached object/body.
  - If global dt `DAT_1465A3D84` is positive, calls `0x141DF5930(world, bodyId, targetPos, identity/defaultQuat, dt)`.
  - If dt is not positive, uses `0x141DF5680` set-position-style wrapper instead.
- `bhkNPCollisionObject::vfunction44` at `0x141E09C37`
  - Builds a transform payload from Ni/object state.
  - Converts rotation through `0x141722C10`.
  - Calls `0x141DF5930(world, bodyId, targetPos, targetQuat, dt)` in a keyframed/update branch.
  - In another branch it also calls `DriveToKeyFrame`, confirming that `ApplyHardKeyFrame` and `DriveToKeyFrame` are separate native behaviors.

Interpretation:

- `0x141DF5930` is currently the better candidate for a future ROCK hidden hand-proxy drive than `0x14153ABD0` because it respects the engine's direct-versus-queued command state.
- This still needs TLS verification inside ROCK's listener phases. If the wrapper queues into a command list that drains before solve, it is ideal. If it queues too late or into a different list, the proxy may be one step stale.

### `0x14153A6A0` ComputeHardKeyFrame parameter convention

Confirmed by Ghidra:

- Parameter 1: `hknpWorld*`.
- Parameter 2: body id.
- Parameter 3: target position float4.
- Parameter 4: target rotation quaternion float4.
- Parameter 5: delta seconds.
- Parameter 6: output linear velocity float4.
- Parameter 7: output angular velocity float4.

Evidence:

- The function computes reciprocal `1 / dt` using `RCPPS` plus Newton-Raphson refinement.
- It reads the live hknp body slot at `world + 0x20 + bodyId * 0x90`.
- It reads the live motion slot through body `+0x68`, stride `0x80`.
- It calls `0x141722C10` on the current body transform to obtain the current quaternion.
- It compares target quaternion against current quaternion to compute angular velocity.
- Native callers pass:
  - target position pointer to the transform translation float4;
  - target rotation pointer to a quaternion produced by `0x141722C10`;
  - not a raw transform matrix pointer.

Important convention finding:

- `targetRot` must be quaternion xyzw-style float4, not a matrix pointer.
- Native callers produce that quaternion from transform rotation with `0x141722C10`.
- Passing the wrong rotation representation, wrong quaternion order, or unconverted `NiMatrix3` rows/columns would break axes and can reproduce the old "hand target does not follow real hand rotation" failure.

### `0x141722C10` transform-to-quaternion helper

Confirmed behavior:

- Reads a transform/matrix-like float array.
- Uses matrix diagonal trace path when trace is positive.
- Uses fallback dominant-axis path when trace is not positive.
- Writes four floats that native callers feed as target rotation to `ComputeHardKeyFrame`.

Interpretation:

- Future ROCK wrapper planning should not invent a new rotation conversion without comparing it against the existing transform math conventions.
- If ROCK already has a reliable `NiTransform` to Havok rotation conversion for constraints, it still needs to confirm the output quaternion order matches `0x14153A6A0`.
- This is one of the highest-risk convention points because a mostly correct position path with wrong target quaternion convention produces exactly the broken wrist/rotated-axis behavior that already happened.

### Relationship to mouse spring

`hknpBSMouseSpringAction` update at `0x141E4AA30` also calls `0x14153A6A0`.

Confirmed from decompilation:

- It validates the target body id and motion.
- It converts the stored/target transform rotation through `0x141722C10`.
- It calls `ComputeHardKeyFrame`.
- It then adds its own mouse-spring behavior:
  - previous error/deadband checks;
  - velocity/motion damping;
  - local grab point handling;
  - force/scale fields;
  - direct velocity and angular/force writes;
  - dirty/activation flag writes.

Interpretation:

- Mouse spring smoothness is partly because it uses the same native hard-keyframe velocity math, then layers damping/deadband/local-point correction on top.
- Replacing mouse spring with custom authority does not mean ignoring this path. The useful native part is `ComputeHardKeyFrame` / `ApplyHardKeyFrame` for the hidden hand proxy, while held-object authority should still come from ROCK's finite-force linear/angular constraint.
- The proxy drive must follow raw/root-flattened hand transform smoothly; the held object must then lag/yield through finite motor force. Those are separate layers.

### Relationship to DriveToKeyFrame

`bhkNPCollisionObject::DriveToKeyFrame` at `0x141E086E0`:

- converts target rotation through `0x141722C10`;
- calls `ComputeHardKeyFrame`;
- checks linear/angular velocity against motion-property caps;
- if caps are exceeded:
  - calls set transform `0x141DF55F0`;
  - then zeroes velocity through `0x141DF56F0`;
- otherwise:
  - applies computed velocity through `0x141DF56F0`.

Interpretation:

- DriveToKeyFrame is not equivalent to ApplyHardKeyFrame.
- DriveToKeyFrame is acceptable for visible generated colliders where snapping may be tolerable after stale/teleport conditions, but it is a bad steady drive candidate for a hidden constraint authority proxy because snap-to-target destroys smooth finite-force behavior.
- Future custom dynamic grab authority should not reuse `driveGeneratedKeyframedBody(...)` as the proxy drive loop unless that loop is split to use ApplyHardKeyFrame semantics.

### ApplyHardKeyFrame conclusions for future custom dynamic grab

Confirmed design-relevant facts:

- ApplyHardKeyFrame operates on a body transform target, not a grab pivot.
- It should be used, if used, only to drive a hidden no-contact hand/palm proxy body toward the raw/root-flattened hand transform.
- It must not drive the grabbed object directly.
- It must not choose the grip pivot.
- It must not define the hand/object relation.
- It does not replace finite-force object motors; it gives the motor constraint a stable body-A frame.
- The grabbed object should still be governed by finite linear/angular motor constraints with contact/palm/object pivot frames and mass/collision/deviation force limits.

Open questions after this pass:

- Which wrapper is correct inside ROCK's future between-collide-and-solve listener:
  - direct `0x14153ABD0`, or
  - command-safe `0x141DF5930`?
- Does a velocity write through `0x141DF5930` update the proxy transform early enough for the same solver step, or only its velocity for integration?
- Should the constraint target frame be computed from the raw hand target, current proxy transform, or predicted proxy transform after ApplyHardKeyFrame velocity?
- Does ROCK's current `transform_math::niRowsToHavokColumns(...)` plus any quaternion helper produce the same xyzw convention as `0x141722C10`?
- Should a future wrapper call the native `0x141722C10` helper directly to avoid quaternion convention drift, or should ROCK implement and test an equivalent conversion?

## 2026-05-12 continuation: physics listener timing and ApplyHardKeyFrame command safety

This pass checks whether a future hidden proxy can be driven from the native between-collide-and-solve listener and whether `ApplyHardKeyFrame` would direct-write or queue in that callback context.

### ROCK source state

Source files inspected:

- `src/physics-interaction/native/PhysicsStepDriveCoordinator.h`
- `src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp`
- `src/physics-interaction/core/PhysicsInteraction.cpp`

Current ROCK coordinator facts:

- `PhysicsStepDriveCoordinator::setDriveCallbacks(...)` exposes only:
  - whole-pre-step callback;
  - substep-pre-collide callback.
- Current native listener vtable layout has slots:
  - `+0x08` before whole;
  - `+0x18` before any;
  - `+0x28` between collide and solve;
  - `+0x38` after any;
  - `+0x48` after whole.
- Current `betweenCollideAndSolve(...)`, `afterAny(...)`, and `afterWhole(...)` functions are no-ops.
- `PhysicsInteraction` currently registers:
  - `driveGeneratedCollidersFromPhysicsWholeStep`;
  - `driveGeneratedCollidersFromPhysicsSubstep`.
- Generated hand/body/weapon colliders are therefore driven pre-collide, which is correct for contact generation, but there is no current ROCK callback for custom proxy + constraint target/motor writes after collide and before solve.

Interpretation:

- Future custom dynamic grab authority needs a new coordinator surface for between-collide-and-solve.
- This should be scoped to the hidden authority proxy and grab constraint fields, not to visible generated contact colliders.
- The existing pre-collide generated collider driving should remain a separate phase because those bodies need to be in place before collision/contact generation.

### Ghidra-confirmed listener order

`bhkWorld::vfunction43` at `0x141DF73A0` confirms the per-substep order:

1. Before-any listener slot `+0x18`.
2. Before-any command drain helper `0x141DFE3B0`.
3. Listener companion slot `+0x20`.
4. Collide helper `0x141DFE010`.
5. Between-collide-and-solve listener slot `+0x28`.
6. Between command drain helper `0x141DFDE70`.
7. Between companion slot `+0x30`.
8. Solve helper `0x141DFE1E0`.
9. After-any listener slot `+0x38`.
10. After-any command drain helper `0x141DFDCD0`.
11. After-any companion slot `+0x40`.

Important detail:

- The between-collide-and-solve callback is followed by a command drain before solve.
- Therefore commands that are actually enqueued into the phase command list during the between callback can be applied before solve.

### Command-drain helpers

Ghidra-decompiled helpers:

- `0x141DFDB30`
- `0x141DFE3B0`
- `0x141DFE010`
- `0x141DFDE70`
- `0x141DFE1E0`
- `0x141DFDCD0`
- `0x141DFD990`

Observed common behavior:

- Inspect command list count in `param_1[1]`.
- If one command exists and its metadata marks it direct/immediate, call command vtable `+0x20`.
- Otherwise build worker batch with `0x141DFFBB0`, dispatch through worker queue helpers, wait on worker completion, then release command refs.
- Clear command list count/state after drain.

Interpretation:

- The drain after between-collide-and-solve is real and not only a label.
- This supports the future phase choice if the write primitive actually queues into the command list.

### TLS command-state uncertainty

Relevant native wrappers branch on TLS byte `+0x1528`:

- `0x141DF55F0` set transform wrapper:
  - direct `0x1415395E0` when TLS `+0x1528 == 0`;
  - command `0x50 / 0x60100` when TLS `+0x1528 != 0`.
- `0x141DF56F0` set velocity wrapper:
  - direct velocity write when TLS `+0x1528 == 0`;
  - command `0x30 / 0x90100` when TLS `+0x1528 != 0`.
- `0x141DF5930` command-safe ApplyHardKeyFrame wrapper:
  - computes hard-keyframe velocity;
  - direct `0x141539F30` when TLS `+0x1528 == 0`;
  - command `0x30 / 0x90100` when TLS `+0x1528 != 0`.
- `0x141DFC8D0` command bridge reads TLS `+0x152C` to select thread command storage at `DAT_1465A3E30 + threadIndex * 0xA8`.

What was checked:

- `bhkWorld::vfunction43` decompilation and disassembly around listener calls.
- Phase drain helpers.
- Command bridge `0x141DFC8D0`.
- World lock helpers:
  - `0x141DF5FB0`;
  - `0x141DF5FD0`;
  - `0x141DF6010`.

Current finding:

- `bhkWorld::vfunction43` visibly checks TLS byte `+0x1529` for world locking decisions.
- This pass did not find `bhkWorld::vfunction43` setting TLS byte `+0x1528` around listener callbacks.
- The visible listener call sequence passes `&local_128` command list to listener callbacks, then drains it, but the standard set-transform/set-velocity/ApplyHardKeyFrame wrappers only queue when TLS `+0x1528` is already active.
- `0x141DFC8D0` itself does not decide direct-versus-queued; it assumes the caller already chose to queue.

Interpretation:

- It is confirmed that the between phase has a pre-solve command drain.
- It is not confirmed that a normal wrapper call from a ROCK between listener will automatically queue into that drain.
- If TLS `+0x1528` is false in the listener callback, `0x141DF5930` will direct-write velocity through `0x141539F30`.
- Direct `0x14153ABD0` is different: it locks `world + 0x690`, computes hard-keyframe velocity, writes velocity, then unlocks. It does not queue, but it owns its body-array lock internally.

### Consequence for future proxy drive primitive

Current evidence does not let us blindly choose `0x141DF5930` just because it is command-capable.

Candidate future primitives:

1. `0x141DF5930` from between-collide-and-solve
   - Pros:
     - command-capable if TLS command mode is active;
     - no DriveToKeyFrame snap branch;
     - native callers already use it for transform-targeted velocity application.
   - Risk:
     - if TLS `+0x1528` is false in ROCK's listener, it direct-writes through `0x141539F30` without the explicit lock seen in `0x14153ABD0`.
2. `0x14153ABD0` direct ApplyHardKeyFrame from between-collide-and-solve
   - Pros:
     - no snap branch;
     - explicitly locks around compute/write;
     - parameter convention now mapped.
   - Risk:
     - not command-queued; must verify it is safe to call while inside bhkWorld update and before solve.
3. A custom explicit command-list path
   - Pros:
     - would guarantee the proxy velocity command drains before solve.
   - Risk:
     - requires more binary-backed command object layout knowledge; higher implementation risk.

Research conclusion from this pass:

- Between-collide-and-solve remains the correct phase for future proxy + constraint updates.
- The exact ApplyHardKeyFrame primitive is still unresolved.
- The safest next research is to verify whether direct body velocity writes in the between listener affect the same solve step, and whether `0x14153ABD0` is safe under the world lock state present in `bhkWorld::vfunction43`.
- Do not implement a proxy drive until this is resolved; choosing the wrong primitive can create one-frame stale body-A motion or unsafe direct writes.

### Open questions after this pass

- Where, if anywhere, is TLS byte `+0x1528` set before physics callbacks?
- Is it possible or appropriate for ROCK to explicitly set command mode around its callback, or is that engine-owned state?
- Does `0x14153ABD0` direct ApplyHardKeyFrame produce same-step proxy motion before solver build when called after collide?
- Does the solver use body-A transform immediately, integrated velocity prediction, or both when building rows for the custom constraint?
- If ApplyHardKeyFrame only writes velocity, should ROCK compute constraint targets from the raw hand target/predicted proxy instead of the current proxy body transform?

## 2026-05-12 continuation: solver consumption of ApplyHardKeyFrame velocity

This pass follows up the `ApplyHardKeyFrame` question from the solver side. The research question is not whether `ApplyHardKeyFrame` is useful in general; it is whether a hidden no-contact body-A proxy driven by hard-keyframe velocity would be seen by the same solver pass as the intended hand transform, or only as current body transform plus velocity.

### Functions inspected

- `0x141DFE1E0`
  - Solve-phase helper called from `bhkWorld::Update` after the between-collide-and-solve listener and its command drain.
- `0x1415435E0`
  - hknp solve dispatch reached by `0x141DFE1E0`.
- `0x1419799E0`
  - Constraint row build caller that gathers hknp body and motion data before delegating to the atom interpreter.
- `0x141979EF0`
  - Related constraint row build caller with the same body/motion data pattern.
- `0x141F5B800`
  - Malleable-constraint wrapper build delegate; confirms wrapped constraint data eventually reaches the normal atom interpreter path.
- `0x141539F30`
  - Direct body velocity writer used by `ApplyHardKeyFrame` and the command-safe velocity path.
- `0x14153D440`
  - Helper called after velocity writes to update linked bodies/broadphase-related body data.
- `0x1417D3BF0`
  - Body broadphase/swept-extent update helper called from `0x14153D440`.
- `0x1415395E0`
  - Direct body transform writer.
- `0x14153CC40`
  - Helper called after transform writes to update body/motion transform state.
- `0x1415451A0`
  - Native body transform prediction/interpolation helper.
- `0x14153EE00`
  - Signal path reached after velocity changes.

### Solve dispatch evidence

`0x141DFE1E0` is the solve helper reached after the between-collide-and-solve callback and the command drain. It calls:

```text
0x1415435E0(*param_5, param_5[1], param_1)
```

before its cleanup/drain tail.

`0x1415435E0` locks the world-side state around the hknp solve dispatch and then calls through the live hknp world/task machinery. This confirms that any body/proxy write intended to affect the same solve must be visible before this solve dispatch begins.

### Constraint row build consumes live hknp body/motion arrays

`0x1419799E0` and `0x141979EF0` gather body and motion data directly from the hknp world arrays:

- body slot base:
  - `world + 0x20 + bodyId * 0x90`
- motion slot base:
  - `world + 0xE0 + body.motionIndex * 0x80`

They extract transform/orientation/motion fields from those slots and pass the assembled input to the atom interpreter path, including `0x141A55550`.

Confirmed implication:

- The solver row build consumes live hknp body/motion arrays.
- It does not consume NiNode transforms or high-level ROCK visual hand transforms.
- A hidden proxy design must make the hknp body/motion arrays correct before row build, or compute target frames from a separate predicted/raw hand transform and write those target frames into the constraint atoms before row build.

### What direct SetBodyVelocity actually writes

`0x141539F30` is the direct body velocity writer reached by `ApplyHardKeyFrame`-style paths.

Observed behavior:

- Locks or enters the world body/motion update path.
- Validates the body id and body flags.
- If velocity is effectively unchanged, can early-exit.
- Wakes/activates the body when the body state requires it.
- Writes linear velocity into the motion slot fields corresponding to motion `+0x40..+0x4C`.
- Writes angular velocity into the motion slot fields corresponding to motion `+0x50..+0x5C`.
- Calls `0x14153D440(world, bodyId, motionSlot)`.
- Emits a velocity/body-change signal through the `0x14153EE00` path.

`0x14153D440` iterates bodies linked to the same motion and calls `0x1417D3BF0(...)`.

`0x1417D3BF0` updates body broadphase/swept-extent data using the motion velocity and world margins/scale. It writes packed body extent data around body `+0x50`. It does not look like a full committed transform integration step.

Confirmed implication:

- Direct velocity write is more than a raw field poke: it wakes/signals and updates swept/broadphase-related body data.
- It is not the same as setting the body transform.
- This pass did not prove that `0x141539F30` immediately makes the proxy body's transform equal the hard-keyframe target before constraint row build.

### What direct SetBodyTransform does differently

`0x1415395E0` is the direct body transform writer.

Observed behavior:

- Writes placement/transform state directly.
- Calls `0x14153CC40(...)`.

`0x14153CC40` updates motion/body transform state and marks associated world/broadphase/island data dirty.

Confirmed implication:

- SetBodyTransform is the placement/warp path.
- It is the path that makes body transform state immediately match a target transform.
- It is not the desired steady proxy drive by itself because it can behave like a teleport/keyframe snap and can erase the finite lag the proxy is supposed to express.

### Native predicted body transform helper

`0x1415451A0` computes a predicted/interpolated transform for a body at a time offset.

Observed behavior:

- Reads the body slot and its motion slot.
- For non-positive time offsets, uses one set of motion/orientation fields around the motion slot's previous/current transform state.
- For positive time offsets, uses motion velocity fields around `+0x40..+0x5C` plus current orientation fields around `+0x10..+0x1C`.
- Integrates orientation using angular velocity/quaternion math and writes an output transform.

Native usage note:

- `bhkNPCollisionObject::vfunction44` uses this helper with the engine's remainder/interpolation dt before updating a high-level object transform path.

Confirmed implication:

- FO4VR has a native helper capable of predicting where a velocity-driven body will be over a time slice.
- This is directly relevant to hidden-proxy design because `ApplyHardKeyFrame` writes velocity, not guaranteed immediate target transform.
- A future custom dynamic-grab design may need to compute the constraint body-A/target frame from the raw hand target or from a predicted proxy transform, rather than assuming the proxy body's current transform already equals the intended palm transform in the same step.

### ApplyHardKeyFrame relevance after this solver-side pass

Confirmed:

- `ApplyHardKeyFrame` / `ComputeHardKeyFrame` is relevant as a native way to turn a target proxy transform into linear/angular velocity.
- It avoids `DriveToKeyFrame`'s cap-triggered transform snap.
- Its velocity write path updates hknp motion velocities, wakes/signals the body, and updates swept/broadphase body data.

Not confirmed:

- That calling `ApplyHardKeyFrame` after collide makes the proxy body's transform equal the target palm transform before the same constraint row build.
- That the atom interpreter builds motor rows against predicted body transforms rather than current body/motion transforms.

Design consequence:

- Do not design the custom motor authority path around the assumption that body-A proxy transform is already at the raw hand target after an `ApplyHardKeyFrame` velocity write.
- The safer architecture is to treat body-A proxy drive and constraint target update as a coupled solver-adjacent operation:
  - drive the hidden proxy with hard-keyframe velocity;
  - compute/write constraint target frames from the same raw/root-flattened hand frame, or from a verified predicted proxy transform;
  - let finite-force linear/angular motors govern the held object;
  - never let proxy drive become object drive.

### Current answer to the user's ApplyHardKeyFrame note

`ApplyHardKeyFrame` is relevant, but only for the hidden hand/palm authority proxy. It should not become the grabbed-object drive and should not define grip pivot, orientation, or object seating. Its value is that it can move a generated hknp proxy with native velocity semantics without the `DriveToKeyFrame` transform-snap fallback.

### Open questions after this pass

- Does the live constraint row builder call any predicted-transform helper equivalent to `0x1415451A0` for velocity-driven bodies before atom interpretation?
- If not, should ROCK compute body-A transform atoms from raw hand target or explicit predicted proxy target instead of current proxy transform?
- Is `0x14153ABD0` safe to call inside the between-collide-and-solve listener when the world update is already in progress?
- Can ROCK use the command-safe `0x141DF5930` wrapper in a way that guarantees queued velocity application in the between-collide-and-solve command drain?

## 2026-05-12 continuation: constraint row builder prediction audit

This pass narrows the previous open question: whether the live hknp constraint row builder predicts a velocity-driven body's pose before interpreting constraint atoms.

### `0x1415451A0` xref audit

Direct code xrefs to the native predicted-transform helper `0x1415451A0`:

- `0x141958656`
  - `hkbnpPhysicsInterface::vfunction16_for_hkReferencedObject`.
  - Calls `0x1415451A0(world, bodyId, param_2, outTransform)`.
  - Copies the predicted transform into a high-level physics interface output.
- `0x141E09AF7`
  - `bhkNPCollisionObject::vfunction44`.
  - Calls `0x1415451A0(world, bodyId, DAT_1465A3D7C, outTransform)`.
  - Uses the predicted transform to update/sync the high-level collision object transform path.
  - In a separate branch, calls `0x141DF5930` and then `0x141DF55F0`, showing native collision-object sync can combine hard-keyframe velocity and explicit transform setting depending on state.
- `0x141E0A4F7`
  - `bhkNPCollisionProxyObject::vfunction44`.
  - Calls `0x1415451A0(world, bodyId, DAT_1465A3D7C, outTransform)`.
  - Applies additional proxy transform composition before updating the high-level object transform path.

Not found:

- No direct xref from the live constraint row builders `0x1419799E0` or `0x141979EF0`.
- No direct xref from the atom interpreter `0x141A55550`.

Current interpretation:

- `0x1415451A0` is a real FO4VR predicted-transform helper, but the direct users found in this pass are high-level physics-interface / collision-object sync paths, not constraint solver row construction.
- This weakens the assumption that a proxy velocity written after collide automatically becomes a predicted body-A pose for the same constraint solve.

### Row builder input assembly

`0x1419799E0` and `0x141979EF0` assemble a solver/atom input structure before calling `0x141A55550`.

Relevant input layout observed from the stack structure passed as `param_3` to `0x141A55550`:

- `param_3 + 0x00..0x2C`
  - solver context scalars copied by `0x141979960`;
  - includes timestep-like and inverse-timestep-like values from the hknp task/world context.
- `param_3 + 0x30`
  - pointer to a per-body motion/cache block assembled on the stack for one constraint body.
- `param_3 + 0x38`
  - pointer to the paired per-body motion/cache block for the other constraint body.
- `param_3 + 0x40`
  - pointer to one hknp body slot.
- `param_3 + 0x48`
  - pointer to the paired hknp body slot.

The row builders source those pointers directly from:

- body array:
  - `world + 0x20 + bodyId * 0x90`
- motion array:
  - `world + 0xE0 + motionIndex * 0x80`

### Body slot transform versus velocity fields

In `0x141A55550`, the atom interpreter begins by copying transform-like data from the body slot pointers:

- reads `*(param_3 + 0x40)` into local body transform basis;
- reads `*(param_3 + 0x48)` into paired local body transform basis;
- copies rows/translation from those body slot pointers into local variables used by later atom cases.

The row builders also copy motion fields into the per-body cache blocks:

- motion fields around `+0x40..+0x5C` are copied and made available to the atom interpreter;
- motion fields around `+0x60..+0x7C` are also copied;
- inverse mass/inertia-like data from the body/motion cache are used by constraint row formulas.

Observed behavior inside `0x141A55550`:

- Setup-style atom cases use body slot transform rows and translation as the positional/orientation basis.
- Velocity/motion fields are used in row equations and effective-mass/velocity terms.
- No direct call to `0x1415451A0` was found in this path.
- No obvious full `position += velocity * dt` / quaternion-integrated predicted transform replacement was found before the body transform basis is used.

Confirmed narrow finding:

- The live row builder consumes current body slot transforms plus current motion velocity/cache fields.
- This pass does not support the claim that the constraint row builder automatically replaces the body-A transform with the `ApplyHardKeyFrame` target pose in the same frame.

### Consequence for hidden body-A proxy design

For a future custom dynamic-grab authority path:

- `ApplyHardKeyFrame` can still be a good way to drive a hidden no-contact proxy because it computes physically meaningful proxy linear/angular velocities without `DriveToKeyFrame`'s snap fallback.
- But a between-collide-and-solve `ApplyHardKeyFrame` call should be treated as a velocity update, not as proof that the proxy body slot transform now equals the raw hand/palm target for that same constraint build.
- The custom constraint update must therefore be designed around one of these verified models:
  - body-A proxy transform from the previous integrated step plus fresh velocity terms;
  - explicit raw/root-flattened hand target written into constraint target atoms;
  - explicit predicted proxy transform computed by ROCK or by a verified native helper before writing constraint target atoms.

What not to do:

- Do not use `DriveToKeyFrame` transform snapping to force the proxy body slot to the hand every step.
- Do not assume native row build predicts body-A pose unless a later pass finds a separate inline prediction path.
- Do not drive the grabbed object with `ApplyHardKeyFrame`; the object remains finite-force linear/angular motor driven.

### Updated open questions

- Which body-A frame should ROCK use for constraint atom target updates: current proxy body transform, raw/root-flattened hand target, or a predicted proxy transform?
- If ROCK computes a predicted proxy transform, should it call `0x1415451A0`, duplicate the relevant math safely, or avoid prediction by using raw hand target directly for constraint targets?
- Does using raw hand target for constraint atoms while body-A proxy lags create solver inconsistency, or is that acceptable because body-A is a keyframed/kinematic authority body?
- Can the hidden proxy be driven early enough in the frame that its body slot transform is already integrated before row build without introducing contact/render lag?

## 2026-05-12 continuation: native combined velocity-plus-transform pattern

The row-builder prediction audit showed that pure `ApplyHardKeyFrame` velocity should not be assumed to update body slot transform for the same constraint row build. This pass checks whether FO4VR has a native pattern that combines hard-keyframe velocity with an explicit transform update.

### `bhkNPCollisionObject::vfunction44` combined path

`0x141E09AF7` / `bhkNPCollisionObject::vfunction44` contains a native branch that does both operations:

1. Build target transform from the high-level object/Ni transform path.
2. If the body is motion-capable and `DAT_1465A3D84 > 0`:
   - calls `0x141722C10` to convert the target transform/matrix into a quaternion;
   - calls `0x141DF5930(world, bodyId, targetPosition, targetQuaternion, dt)` to compute/apply hard-keyframe velocity.
3. Calls `0x141DF55F0(world, bodyId, targetTransform, 1)` to set the body transform.

Important distinction:

- This is not the same as `DriveToKeyFrame`.
- `DriveToKeyFrame` can snap only when hard-keyframe velocity exceeds motion-property caps.
- This vfunction branch intentionally computes velocity and then explicitly sets transform as separate operations.

Why this matters for a future hidden proxy:

- A pure velocity proxy may leave body-A transform one integrated step behind the raw hand target.
- A transform-only proxy would make body-A pose correct but may lose velocity information needed for believable constraint response.
- The native collision-object sync path shows FO4VR sometimes wants both: pose correctness plus velocity correctness.

### Other `ApplyHardKeyFrame` wrapper caller

`0x1412CB4E0` is the other direct code caller of `0x141DF5930` found in this pass.

Observed behavior:

- It updates a body associated with a camera/player-related object path.
- If `DAT_1465A3D84 <= 0`, it calls a linear-velocity wrapper path.
- Otherwise it calls `0x141DF5930(world, bodyId, targetPosition, identityQuaternion, DAT_1465A3D84)`.
- This caller does not immediately pair the hard-keyframe velocity call with `0x141DF55F0` in the same branch.

Interpretation:

- FO4VR uses `ApplyHardKeyFrame` both as a standalone velocity target and as part of a velocity-plus-transform sync pattern.
- The combined velocity-plus-transform pattern is not universal; it is specifically visible in the collision-object sync branch where high-level transform and hknp body transform need to be reconciled.

### `DriveToKeyFrame` comparison

`0x141E086E0` / `DriveToKeyFrame` behavior remains distinct:

- Converts target transform to quaternion.
- Calls `0x14153A6A0` to compute hard-keyframe linear/angular velocity.
- Checks motion-property max linear/angular velocity caps.
- If a cap is exceeded:
  - calls `0x141DF55F0` to set transform;
  - then calls `0x141DF56F0` with zero velocities.
- If caps are not exceeded:
  - calls `0x141DF56F0` with computed linear/angular velocity.

Confirmed distinction:

- `DriveToKeyFrame` is either velocity drive or cap-triggered transform snap plus zero velocity.
- `bhkNPCollisionObject::vfunction44` can do hard-keyframe velocity and set transform in the same update branch.
- For a hidden no-contact proxy, the latter pattern is more relevant than `DriveToKeyFrame` if the design requires both same-step body-A pose correctness and body-A velocity correctness.

### Command flush ordering

`0x141DF4F60` processes queued command groups in a fixed order:

1. body transform commands:
   - validates body id;
   - calls `0x1415395E0` direct set transform.
2. body velocity commands:
   - validates body id;
   - calls `0x141539F30` direct set velocity.
3. other body commands, including activation/motion-property-like operations.

Implication:

- If `0x141DF55F0` and `0x141DF56F0` / `0x141DF5930` both queue into the same command buffer, the flush applies transform commands before velocity commands.
- That ordering is good for a hidden proxy pattern that wants:
  - body slot transform current for row building;
  - velocity fields current for constraint velocity terms.
- It still does not prove that normal ROCK listener calls enter queued mode; TLS command state remains separately unresolved.

### Updated proxy-drive interpretation

The research direction should now distinguish three possible proxy drive primitives:

1. Pure `ApplyHardKeyFrame` velocity
   - Pros:
     - native velocity computation;
     - no `DriveToKeyFrame` snap fallback.
   - Cons:
     - row-builder evidence does not prove same-step body slot transform correctness.
2. Pure set transform
   - Pros:
     - row-builder sees the correct body-A pose immediately.
   - Cons:
     - may leave velocity terms stale or zero, weakening finite-force response and making constraint rows less physically honest.
3. Combined hard-keyframe velocity plus set transform
   - Pros:
     - native FO4VR collision-object sync has this pattern;
     - can provide both correct body-A pose and correct velocity terms before row build if applied in the right phase.
   - Cons:
     - must be verified for hidden no-contact generated bodies and the chosen callback/command context;
     - must not be confused with driving the grabbed object or snapping the held object.

Current best research hypothesis:

- A future hidden body-A authority proxy may need the combined pattern, not pure `ApplyHardKeyFrame`.
- This does not violate the no-superman requirement because the proxy is only the hand-side constraint body. The held object would still be governed by finite-force linear/angular motors, mass caps, angular caps, collision tau, and deviation logic.

### Open questions after this pass

- Can ROCK safely issue both transform and hard-keyframe velocity updates for a hidden proxy inside the chosen physics phase?
- Does the generated `BethesdaPhysicsBody` wrapper expose enough to call the combined pattern without reusing `driveGeneratedKeyframedBody`?
- If the combined pattern is used, should transform be raw/root-flattened hand pose while velocity is computed from previous proxy pose toward raw/root-flattened hand pose?
- Does setting transform on a hidden no-contact keyframed proxy every step introduce any solver instability, or is it equivalent to the native collision-object sync expectation?

## 2026-05-12 continuation: wrapper queue versus physics-step local command drain

This pass separates two command mechanisms that looked similar in earlier notes:

1. The physics-step listener local command list passed through `bhkWorld::Update`.
2. The global/thread wrapper command buffer used by standard body wrappers when TLS byte `+0x1528` is set.

They are not proven to be the same drain path.

### Standard wrapper queue behavior

`0x141DF55F0` set transform wrapper:

- If TLS byte `+0x1528 == 0`:
  - calls `0x1415395E0(world, bodyId, transform, mode)` directly.
- If TLS byte `+0x1528 != 0`:
  - builds command `0x50 / 0x60100`;
  - sends it through `0x141DFC8D0`.

`0x141DF56F0` set velocity wrapper:

- If TLS byte `+0x1528 == 0`:
  - calls `0x141539F30(world, bodyId, linearVelocity, angularVelocity)` directly.
- If TLS byte `+0x1528 != 0`:
  - builds command `0x30 / 0x90100`;
  - sends it through `0x141DFC8D0`.
- After either path, if linear/angular velocity is nonzero, calls `0x141DF60C0(world, bodyId)` to activate/wake the body.

`0x141DF5930` command-safe `ApplyHardKeyFrame` wrapper:

- Computes target linear/angular velocity with `0x14153A6A0`.
- If TLS byte `+0x1528 == 0`, direct-calls `0x141539F30`.
- If TLS byte `+0x1528 != 0`, queues the same velocity command through `0x141DFC8D0`.

### `0x141DFC8D0` global/thread wrapper command bridge

`0x141DFC8D0` uses:

- TLS dword `+0x152C` as a thread index;
- global command storage rooted at `DAT_1465A3E30`;
- per-thread stride `0xA8`;
- per-command grouped buffers inside that global storage.

It appends command records by command type:

- command `6` / wrapper metadata `0x60100` for transform-like commands;
- command `7` for another body command class;
- command `9` / wrapper metadata `0x90100` for velocity-like commands;
- command `0x12` and `0x2C` for other body/object command classes.

Important finding:

- `0x141DFC8D0` does not append into the `local_128` listener command list visible in `bhkWorld::Update`.
- It appends into `DAT_1465A3E30`-based global/thread command storage.

### Global wrapper-command drain `0x141DF4F60`

`0x141DF4F60` drains the global/thread wrapper command storage:

- It processes transform commands first and calls `0x1415395E0`.
- It processes velocity commands next and calls `0x141539F30`.
- It then processes other body/object command groups and clears counts.

Observed xrefs:

- `0x140D06610`
- `0x140D06DB0`
- one data/code reference around `0x140D04C90` that Ghidra did not identify as a normal function start.

The decompiled callers `0x140D06610` and `0x140D06DB0` look like higher-level update/tick paths:

- call time/update helpers;
- call `0x141DF4F60`;
- then call a vtable function on a player/camera/current-world object.

Current interpretation:

- `0x141DF4F60` is a global wrapper-command flush.
- It is not the same as the between-collide-and-solve local command drain in `bhkWorld::Update`.
- This means queueing a wrapper command through `0x141DFC8D0` from a ROCK physics-step listener would not automatically mean the command drains before the immediately following solve.

### Physics-step local command drain

The between-collide-and-solve local drain is `0x141DFDE70`.

Observed behavior:

- Takes `param_1` as the local command list passed through `bhkWorld::Update` (`local_128` in the decompiler).
- If the list has one direct command, calls that command's vtable `+0x20`.
- Otherwise builds worker batches with `0x141DFFBB0`, dispatches work, waits, releases local command refs, and clears the local list.
- It does not reference `DAT_1465A3E30`.
- It does not call `0x141DF4F60`.
- It is effectively identical in shape to the other phase-local drain helpers (`0x141DFDB30`, `0x141DFDCD0`, etc.).

Confirmed implication:

- The pre-solve listener drain is real, but it drains the listener local command list.
- The standard body wrappers do not obviously write to that local command list.
- Therefore, relying on wrapper queue mode for same-step pre-solve updates is not supported by current evidence.

### Consequence for future hidden proxy updates

For a future custom dynamic-grab body-A proxy:

- Calling standard wrappers from the between-collide-and-solve listener has two possible behaviors:
  - if TLS `+0x1528 == 0`, the wrapper direct-writes through the engine's direct body update functions;
  - if TLS `+0x1528 != 0`, the wrapper queues into global/thread wrapper storage, which is not proven to drain before the current solve.
- The earlier assumption "command-capable wrapper equals between-phase command drain" is wrong.

Research-safe design options now look like:

1. Direct locked writes in the between-collide-and-solve callback
   - Use direct/standard wrapper paths only if they direct-call `0x1415395E0` and `0x141539F30`.
   - Must verify reentrancy/safety inside `bhkWorld::Update`.
2. Explicit local listener-command integration
   - Requires understanding the local command-list item layout and vtable command objects used by `0x141DFDE70`.
   - More binary-risky, but would guarantee pre-solve drain.
3. Earlier-frame update outside the physics listener
   - Avoids direct writes inside the physics step.
   - Risks one-frame stale proxy pose relative to row building.

### Updated open questions

- Is TLS `+0x1528` clear during ROCK's between-collide-and-solve listener callback in practice?
- If clear, are direct `0x1415395E0` / `0x141539F30` calls safe while `bhkWorld::Update` is between collide and solve?
- If set, where is `DAT_1465A3E30` flushed relative to `bhkWorld::Update`, and would queued wrapper commands miss the current solve?
- Can ROCK append body transform/velocity commands into the local `local_128` listener command list without reverse-engineering too much engine-owned command object state?

## 2026-05-12 continuation: direct body setter safety map

This pass checks the direct transform and velocity writers used by the standard wrappers when TLS byte `+0x1528` is clear.

### `0x1415395E0` direct set body transform

Observed behavior:

- Acquires the world/body update lock through `0x141538AF0(param_1 + 0x690)`.
- Computes body slot:
  - `world + 0x20 + bodyId * 0x90`.
- Compares the requested transform against current body transform using `0x14158E790`.
- If transform differs:
  - converts the input transform orientation into quaternion-like body/motion representation;
  - writes the new transform rows/translation into the hknp body slot;
  - calls `0x14153CC40(world, bodyId, quaternion, 0, mode)`;
  - signals body-transform listeners through a list rooted around `world + 0x538`;
  - releases/removes listener nodes as needed.
- Releases the world/body update lock through `0x141538CB0(param_1 + 0x690)`.

Confirmed:

- Direct set-transform is an engine-owned path with lock, dirty update, and listener signaling.
- It is not a raw write to the body array.
- It makes body slot transform current immediately, unlike pure velocity write.

### `0x14153CC40` transform dirty/update helper

Observed behavior:

- Receives world, body id, quaternion/rotation data, and mode flags.
- Updates motion/body transform state for motion-backed bodies.
- Rebuilds or updates bounds/filter/proxy data associated with the moved body.
- Calls island/broadphase dirty paths such as `0x1417D8590(...)`.
- Can notify lower-level world systems through vtable calls from `world + 0x178`.

Confirmed:

- Set-transform has a large follow-up path. It is not only changing visual transform.
- If used for a hidden proxy, the proxy should be no-contact and minimal specifically to reduce broadphase/contact side effects.

### `0x141539F30` direct set body velocity

Observed behavior:

- Acquires the world/body update lock through `0x141538AF0(param_1 + 0x690)`.
- Validates the body and only proceeds for motion-capable bodies (`body + 0x40` bit `2`).
- Computes body slot and motion slot.
- Compares requested linear/angular velocity with existing motion velocity.
- If velocity differs:
  - wakes/activates body via `0x141546EF0(world, bodyId)` when body flags allow;
  - writes linear velocity into motion fields around `+0x40..+0x4C`;
  - writes angular velocity into motion fields around `+0x50..+0x5C`, transformed into the body's motion frame;
  - calls `0x14153D440(world, bodyId, motionSlot)`;
  - signals velocity/body-change listeners through `0x14153EE00(world + 0x538, world, bodyId)`.
- Releases the world/body update lock.

Confirmed:

- Direct set-velocity is also an engine-owned path with lock, wake, swept update, and listener signaling.
- It updates velocity and swept/broadphase-related body data, but it does not directly set the body slot transform to the target pose.

### `0x14153D440` velocity dirty/update helper

Observed behavior:

- Iterates all bodies linked to the same motion.
- Calls `0x1417D3BF0(bodySlot, motionSlot, ...)` for each linked body.
- `0x1417D3BF0` updates body broadphase/swept extents from velocity/motion data.

Confirmed:

- Velocity writes update swept bounds for all bodies sharing the motion.
- This matters for multipart weapons/objects: a proxy must not share motion with held object parts, and held object connected-body handling must remain separate.

### Safety interpretation

Current evidence supports the following narrow statement:

- `0x1415395E0` and `0x141539F30` are proper native hknp body mutation paths.
- They take the world/body lock internally.
- They dirty/signals the same world systems native callers rely on.

What is still not proven:

- That calling both direct setters from a ROCK between-collide-and-solve listener cannot deadlock or violate a phase invariant.
- Whether the body-list lock held by `bhkWorld::Update` around listener callbacks is compatible with body update locks in all cases.

Design consequence:

- Direct setters are now better-supported candidates than global wrapper queueing for same-step pre-solve hidden proxy updates.
- They still need a final reentrancy/lock-order pass before any implementation.
- If used, the proxy must be hidden/no-contact/minimal because transform dirty paths are broadphase-aware.

### Updated open questions

- Does `bhkWorld::Update` hold any lock during listener callbacks that can conflict with `0x141538AF0(world + 0x690)`?
- Do native physics-step listeners ever call direct body setters from before/between/after callbacks?
- Is it safer to call direct setters from a before-any step callback instead of between-collide-and-solve if broadphase/contact state is involved?
- For a no-contact proxy, is broadphase update cost/state relevant if filter bit 14 or noncollidable layer ensures no contacts?

## 2026-05-12 continuation: ROCK step-coordinator source reread after command-drain audit

This pass rereads the current ROCK source after the Ghidra command-drain work. It is documentation only; no implementation has been changed.

### Source files reread

- `src/physics-interaction/native/PhysicsStepDriveCoordinator.h`
- `src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
- `src/physics-interaction/hand/HandGrab.cpp`

### Current ROCK callback shape

`PhysicsStepDriveCoordinator` has a native-compatible step-listener vtable with slots matching the Ghidra-observed FO4VR listener call offsets:

- `beforeWhole` at vtable offset `+0x08`.
- `beforeAny` at vtable offset `+0x18`.
- `betweenCollideAndSolve` at vtable offset `+0x28`.
- `afterAny` at vtable offset `+0x38`.
- `afterWhole` at vtable offset `+0x48`.

The public callback API currently exposes only:

- `_wholePreStepCallback`
- `_substepPreCollideCallback`

The current between/after functions are no-ops:

- `betweenCollideAndSolve(...) {}`.
- `afterAny(...) {}`.
- `afterWhole(...) {}`.

`PhysicsInteraction::PhysicsInteraction` wires the two exposed callbacks as:

- whole-pre-step: `PhysicsInteraction::onNativeGrabPhysicsStep`.
- substep-pre-collide: `PhysicsInteraction::onGeneratedColliderPhysicsSubstep`.

`driveNativeGrabFromPhysicsStep(...)` runs whole-pre-step and calls:

- `_rightHand.flushPendingHeldNativeGrab(world, timing)`.
- `_leftHand.flushPendingHeldNativeGrab(world, timing)`.

`driveGeneratedCollidersFromPhysicsSubstep(...)` runs pre-collide substep and calls:

- `_rightHand.flushPendingCollisionPhysicsDrive(world, timing)`.
- `_leftHand.flushPendingCollisionPhysicsDrive(world, timing)`.
- `_bodyBoneColliders.flushPendingPhysicsDrive(world, timing)`.
- `_weaponCollision.flushPendingPhysicsDrive(world, timing)`.

`Hand::flushPendingHeldNativeGrab(...)` only flushes `_nativeGrab`. It does not update custom constraint targets or any future proxy body.

### Confirmed source-level gap

ROCK already has the native vtable slot needed for between-collide-and-solve work, but the slot is not surfaced to `PhysicsInteraction` and currently does nothing.

Therefore any future custom one-hand dynamic-grab authority cannot be correctly placed in the between phase without adding a first-class callback path. Piggybacking on the existing callbacks would put work in the wrong phase:

- whole-pre-step for native mouse-spring target flush;
- before-any/pre-collide for generated contact colliders;
- no current path for post-collide/pre-solve proxy or constraint target writes.

### Design consequence for later implementation planning

The future custom-authority path should not reuse `driveNativeGrabFromPhysicsStep(...)` as-is, because that name and timing are tied to native mouse-spring flushing.

The likely future architecture needs an explicitly named between-phase callback for:

- hidden palm/body-A proxy pose/velocity update, if a proxy is used;
- custom constraint target atom updates;
- custom motor tuning updates;
- any held-body finite-force authority state that must be consumed by the same solver step.

This is still research-only. The exact callback shape is not an implementation decision until the remaining binary safety questions are closed.

## 2026-05-12 continuation: between-phase direct setter lock and solver-consumption audit

This pass checks whether the FO4VR between-collide-and-solve listener phase is structurally compatible with direct transform/velocity writes for a future hidden authority proxy. The goal is not to choose implementation yet; it is to understand what the solver can consume in the same step.

### `bhkWorld::vfunction43` / update loop `0x141DF73A0`

The FO4VR update loop calls listener phases in this order:

1. `BeforeWholePhysicsUpdate`
2. each substep:
   - `BeforeAnyPhysicsStep`
   - `Collide`
   - `BetweenPhysicsCollideAndSolve`
   - `Solve`
   - `AfterAnyPhysicsStep`
3. `AfterWholePhysicsUpdate`

For every listener phase, the update loop acquires a listener-array lock around iteration:

- acquire: `0x141B932B0`
- release: `0x141B93570`

The between phase specifically does:

- call listener vtable slot `+0x28`;
- drain the local command list with `0x141DFDE70`;
- call companion listener vtable slot `+0x30`;
- release the listener-array lock;
- then enter `TtSolve` through `0x141DFE1E0`.

The listener-array lock is not the hknp body/world mutation guard used by the direct body setters.

### Listener-array lock functions

`0x141B932B0` is a recursive/shared-style lock helper:

- if current thread already owns it, it increments a count;
- otherwise it atomically increments the low-count field;
- waits with `Sleep(...)` if unavailable.

`0x141B93570` decrements the count.

`0x141B93330` / `0x141B93580` are exclusive-style helpers used elsewhere in the same update loop when clearing listener arrays after the step.

### hknp thread-safety guard functions

The direct hknp body writers use a separate guard at `world + 0x690`.

`0x141538AF0` is the write/mutate guard:

- enters a critical section;
- asserts no read lock bits are active (`low 5 bits`);
- validates owner thread if already owned;
- increments write-depth bits (`0xE0`);
- records the current thread id in the high owner bits.

`0x141538CB0` releases that write/mutate guard:

- validates owner thread;
- decrements write-depth bits;
- clears owner state when depth reaches zero.

`0x1415388F0` / `0x141538A30` are the read guard pair:

- read enter increments low 5 bits;
- read exit decrements low 5 bits;
- the write guard rejects mutation if those read bits are active.

### Collide helper lock timing

`0x141DFE010` is the update-loop collide helper wrapper. It calls `0x1415433B0(...)`.

`0x1415433B0`:

- enters the hknp write/mutate guard with `0x141538AF0(world + 0x690)`;
- runs the collide phase work under the `TtCollide` timer;
- exits through `0x141538CB0(world + 0x690)` before returning.

Because `bhkWorld::vfunction43` calls the between listener phase only after `0x141DFE010` returns, the hknp write/mutate guard used by collide is not still held by collide when the between callback starts.

### Solve helper lock timing

`0x141DFE1E0` is the update-loop solve helper wrapper. It calls `0x1415435E0(...)`.

`0x1415435E0`:

- enters the hknp write/mutate guard with `0x141538AF0(world + 0x690)`;
- runs the solve path under the `TtSolve` timer;
- performs command dispatch/constraint-row/solve work from the current body/motion/constraint state;
- exits through `0x141538CB0(world + 0x690)`.

Because `bhkWorld::vfunction43` calls `0x141DFE1E0` only after the between listener phase and its local command drain, changes made during the between phase can be visible to the solve helper if they update the body/motion/constraint arrays directly or through same-phase local commands.

### Direct setter compatibility finding

Current binary evidence supports this narrow statement:

- The between listener callback runs after collide has released the hknp write/mutate guard and before solve has acquired it.
- Direct body setters acquire the hknp write/mutate guard themselves.
- The update loop still holds the listener-array iteration lock while invoking between callbacks, but that is a different lock family from `world + 0x690`.
- Therefore a between-phase direct body update is not immediately disqualified by an already-active hknp read/write guard from collide or solve.

This does not prove every future direct write is safe. It does remove one major blocker: collide does not appear to leave the hknp mutation guard active across the between listener callback.

### Same-step solver consumption implication

The solve helper is entered after the between callback and after the between local command drain.

Combined with earlier row-builder evidence that constraint row building reads current body/motion arrays, the current best binary-backed model is:

- pure `ApplyHardKeyFrame` / velocity write before solve updates motion velocity fields but does not set the body slot pose to the target transform;
- pure set-transform before solve updates the body slot pose but may leave velocity stale;
- combined hard-keyframe velocity plus set-transform before solve can make both current pose and velocity visible to the solve phase;
- a future body-A proxy should be driven before solve if its transform is used as constraint body A in the same solver step.

For a hidden no-contact proxy, the strongest researched timing remains the between-collide-and-solve phase, not whole-pre-step and not after-solve.

### Local command processor evidence

`hknpApiCommandProcessor::vfunction5` at `0x1417F258F` confirms the local command machinery can execute body mutation commands:

- command `6`: calls direct set body transform `0x1415395E0`.
- command `9`: calls direct set body velocity `0x141539F30`.
- command `2`: destroy body path `0x141544E80`.
- command `3`: remove body path `0x141544B00`.
- command `0x15`: set body motion properties `0x14153B2F0`.

This matters because the between-phase drain `0x141DFDE70` executes commands from the local step command list before solve.

However, ROCK does not currently have a mapped safe way to append transform/velocity commands into that `local_128` list from its listener callback. The standard Bethesda wrappers queue through `0x141DFC8D0` into global/thread wrapper storage, not into this local step list.

### Updated future proxy timing options

The researched options now rank like this:

1. Direct setter calls in a new between-collide-and-solve callback.
   - Best current same-step evidence.
   - Uses native direct writer paths with hknp mutation guard.
   - Must still account for listener-array lock and body-change listener side effects.
2. Explicit local command-list integration.
   - Would match FO4VR's intended local command drain timing.
   - Requires mapping the command object allocation/appending ABI; not yet researched enough.
3. Whole-pre-step or before-any drive.
   - Existing ROCK callbacks can do it today, but the proxy/body-A pose may be stale by the time solve consumes rows, especially under substep/controller timing mismatch.
4. Wrapper queued commands.
   - Not acceptable as a same-step pre-solve assumption because wrapper queue storage is separate from the between local command list.

### Updated open questions

- Does direct set-transform from inside a listener callback trigger body-change listeners that can mutate the same step-listener array or otherwise recurse into unsafe update paths?
- Is command-list integration practical enough to avoid direct setter calls, or would it require too much engine-owned allocation/state reconstruction?
- For a hidden no-contact proxy, does set-transform after collide and before solve have any negative consequence if the proxy is not expected to contribute contacts?
- If the proxy ever needs contact feedback later, it cannot be moved after collide because collision data would be from the previous pose.
- The future implementation must decide whether body-A proxy velocity is computed through `ComputeHardKeyFrame` or through ROCK's own raw/root-flattened hand velocity sample, but this is a later design decision after the command/listener safety map is complete.

## 2026-05-12 continuation: direct setter notification side effects and current ROCK constraint timing

This pass connects the direct-setter Ghidra findings to ROCK's current custom constraint source path.

### Direct set-transform notification side effects

`0x1415395E0` direct set body transform:

- enters the hknp write/mutate guard at `world + 0x690`;
- validates body slot / body id mappings;
- writes the body transform into the hknp body slot;
- calls `0x14153CC40(...)`;
- then iterates a listener list rooted at `hknpWorld + 0x538` and calls each listener's vtable slot `+0x10` with `(listener, world, bodyId)`;
- exits the hknp write/mutate guard.

`0x14153CC40(...)` is a broad dirty/update path. It:

- re-enters the same hknp write/mutate guard, so direct set-transform uses nested/reentrant hknp mutation guards;
- calls `0x14153EE00(hknpWorld + 0x510, world, bodyId)` in one path;
- updates dirty broadphase/island/body ranges and calls `0x1417D8590(...)` for affected bodies;
- notifies lower-level world systems through vtable calls rooted around `world + 0x178`.

`0x14153EE00(...)` iterates a linked listener list encoded in the low bits of its list head. It calls listener vtable slot `+0x10` and removes marked nodes.

Important separation:

- The physics step listener array in `bhkWorld::vfunction43` is stored/iterated from the `bhkWorld` listener storage around the decompiler's `offset_0x10`, `offset_0x20`, and `offset_0x40`.
- Direct hknp body setters notify hknp-world body-change lists at `hknpWorld + 0x510` and `hknpWorld + 0x538`.
- Current evidence shows these are separate listener structures, not the same step-listener array.

Open side-effect:

- Even if the storage is separate, body-change listeners can still run during a between-phase direct setter. Their side effects are not fully mapped.
- For a hidden no-contact proxy this risk is narrower than for a normal gameplay body, but it still exists.

### Direct set-velocity notification side effects

`0x141539F30` direct set body velocity:

- enters the hknp write/mutate guard at `world + 0x690`;
- validates the body is motion-capable (`body.flags & 2`);
- reads the motion slot from the body motion index;
- wakes/activates the body through `0x141546EF0(...)` when allowed;
- writes linear velocity to motion fields around `+0x40..+0x4C`;
- writes angular velocity to motion fields around `+0x50..+0x5C`, transformed through the body's motion frame;
- calls `0x14153D440(...)`;
- notifies `hknpWorld + 0x538` through `0x14153EE00(...)`;
- exits the hknp write/mutate guard.

`0x14153D440(...)` updates swept/broadphase extents for all bodies sharing the same motion.

Confirmed implication:

- A future hidden proxy must use its own motion and must not share motion with the held object or any multipart weapon/object body.
- A proxy velocity write has real native broadphase/swept side effects, even if the proxy is no-contact.

### ROCK current custom constraint creation

Source files:

- `src/physics-interaction/grab/GrabConstraint.cpp`
- `src/physics-interaction/grab/GrabConstraint.h`
- `src/physics-interaction/grab/GrabConstraintMath.h`
- `src/physics-interaction/grab/GrabMotionController.h`
- `src/physics-interaction/hand/HandGrab.cpp`

`createGrabConstraint(...)` builds a persistent custom atom stream:

- custom vtable copied from the ragdoll constraint data vtable with overrides for type/info/runtime/addInstance;
- atom chain:
  - set-local-transforms atom at `0x20`;
  - setup-stabilization atom at `0xB0`;
  - ragdoll motor atom at `0xC0`;
  - linear motor atoms at `0x120`, `0x138`, `0x150`;
- runtime reports `0x100` bytes and 12 solver results;
- position motors are explicit `HkPositionMotor` allocations with vtable `0x142E95FE8`;
- the same angular position motor is assigned to the three ragdoll axes;
- the same linear position motor is assigned to the three linear axes;
- motors are enabled before `hknpWorld::CreateConstraint(...)`.

The custom constraint atom/motor layout is already consistent with the earlier Ghidra solver atom map.

### ROCK current custom constraint target update

`Hand::updateConstraintGrabDriveTarget(...)` currently runs from `Hand::updateHeldObject(...)`, not from a physics-step between callback.

It:

- starts with raw-hand desired object/body output for diagnostics;
- requires `_activeConstraint`, `_handBody`, and live `_handBody` world transform;
- recomputes desired object/body world from `liveHandBodyWorld`, `_grabFrame.constraintHandSpace`, and `_grabFrame.bodyLocal`;
- writes transform A translation from `_grabFrame.pivotAHandBodyLocalGame`;
- writes transform B rotation and ragdoll angular target from `desiredBodyTransformHandSpace`;
- writes dynamic transform B translation from:
  - `desiredBodyTransformHandSpace`;
  - `_grabFrame.pivotAHandBodyLocalGame`;
  - `grab_constraint_math::computeDynamicTransformBTranslationGame(...)`.

Important:

- Transform B is not supposed to be a static frozen pivot when the angular frame changes.
- ROCK already has math for dynamic transform-B translation, preserving contact/palm relation instead of COM authority.
- The current source gap is timing/authority: these writes are performed in game/update flow for `SharedConstraint`, not in the solver-consumed between phase for ordinary one-hand dynamic grab.

### ROCK current custom motor update

`Hand::updateConstraintGrabDriveMotors(...)` also currently runs from `Hand::updateHeldObject(...)`, only when `_heldDriveMode == SharedConstraint`.

It calls `grab_motion_controller::solveMotorTargets(...)` with:

- held-body collision flag;
- position and rotation error;
- base linear/angular tau;
- collision tau;
- tau lerp speed;
- base max force;
- adaptive max-force multiplier;
- body mass;
- force-to-mass ratio;
- angular-to-linear force ratio;
- startup fade state;
- loose-weapon multipliers.

`GrabMotionController` confirms the desired finite-force behavior already exists in source form:

- linear authority is capped by mass via `capForceByMass(force, mass, forceToMassRatio)`;
- angular max force is derived from linear max force via `linearForce / angularToLinearForceRatio`;
- tau moves gradually with `advanceToward(...)` instead of snapping;
- collision uses a softer collision tau;
- startup fade reduces force and starts angular authority weaker through `fadeStartAngularRatio`.

Important:

- These rules are close to the HIGGS-style finite-force/mass/angular/collision behavior the user wants.
- They are currently not the ordinary one-hand loose-object authority path.
- They are currently not guaranteed to be written in the same solver step that consumes the constraint rows.

### ROCK current ordinary one-hand path

In `Hand::grabSelectedObject(...)`:

- peer-held / joining another hand uses `createConstraintGrabDrive(...)`.
- ordinary single-hand loose object / loose weapon sets `_heldDriveMode = HeldObjectDriveMode::NativeMouseSpring`.
- ordinary single-hand creates `_nativeGrab` with native mouse-spring tuning.

In `Hand::updateHeldObject(...)`:

- native path computes the desired body transform from raw hand and `_grabFrame.rawHandSpace/bodyLocal`;
- applies adaptive lead only for native mouse spring;
- queues native target through `_nativeGrab.queueTarget(...)`;
- the target is later flushed in `Hand::flushPendingHeldNativeGrab(...)` from whole-pre-step.

Therefore:

- ordinary one-hand loose object/weapon is still native mouse-spring authority;
- custom finite-force motors are present but only active for `SharedConstraint`;
- visual hand lag is downstream of object lag and cannot create weight if the native action follows too strongly.

### Updated parity gap

The key gap is no longer "ROCK lacks finite-force motor math."

The current evidence says ROCK lacks a correctly phased ordinary one-hand custom-authority path that combines:

- generated/raw root-flattened hand frame as body-A source;
- hidden/no-contact solver-visible proxy or equivalent body-A authority;
- between-collide-and-solve proxy and constraint target/motor writes;
- existing contact/palm non-COM grab frame;
- existing `GrabMotionController` finite-force/mass/angular/collision behavior;
- ordinary one-hand loose-object and loose-weapon routing, separate from equipped/two-hand weapons and actor ragdoll.

### Updated open questions

- Can direct body setter notifications on `hknpWorld +0x510/+0x538` cause unsafe reentrancy during the between listener phase, or are they ordinary body-change callbacks that remain safe under the hknp write guard?
- If not safe enough, can ROCK construct local command-list entries for command `6` transform and command `9` velocity and append them to `local_128`?
- Should future motor target/motor writes happen exclusively in the between callback, or should game update prepare a pending target snapshot that the between callback applies?
- Does current `updateConstraintGrabDriveTarget(...)` depend on live generated hand-body transform being already current? If so, a future hidden proxy must replace that dependency for ordinary one-hand grab.
