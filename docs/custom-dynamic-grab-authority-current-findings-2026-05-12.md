# Custom Dynamic Grab Authority - Current Findings

Date: 2026-05-12

This file supersedes `docs/custom-dynamic-grab-authority-tracker-2026-05-12.md` where the two conflict. The older tracker remains useful as historical context, but this file records the newer Ghidra/source audit findings from the current pass.

This is research only. No implementation has been made from these findings.

## Current Framing

HIGGS is the behavioral reference for polished dynamic grab feel, not a 1:1 implementation target. Fallout 4 VR is on a newer hknp/Havok stack than Skyrim VR, so the correct ROCK design should use FO4VR-native functions, wrappers, step phases, and constraint/action mechanisms where they give better control or stability.

The target remains ordinary dynamic one-hand grab for loose objects and loose non-equipped weapons. Equipped weapons, two-hand equipped weapon behavior, actor ragdoll grab, and HIGGS keyframed grab are outside this pass unless a shared infrastructure function must be understood.

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

## Current Parity Gap Summary

What ROCK already has:

- contact/mesh/palm grip capture;
- non-COM pivot storage;
- hand-relative object relation;
- held visual relation/deviation scaffolding;
- custom motor constraint path;
- mass cap policy in `GrabMotionController`;
- loose weapon tuning surfaces;
- generated root-flattened hand bodies;
- player-space compensation and release logic.

What is missing or suspect:

- trustworthy custom constraint runtime layout;
- physics-phase-safe custom target writes;
- dedicated grab proxy body;
- no-contact proxy filter/layer policy;
- unified authority for proxy + constraint update before solve;
- source tests that enforce custom one-hand authority instead of native one-hand authority;
- possible FO4VR-native malleable wrapper integration, if verified useful.
- native `hknpEaseConstraintsAction` is no longer a likely custom-grab smoothing candidate because it handles only native type `2` and type `7`.

## No-Code Implementation Implications

Before any custom one-hand replacement can be implemented:

1. Finish the custom runtime layout audit.
   - Confirm final solver result count.
   - Confirm final runtime size.
   - Confirm ragdoll and linear motor offsets.
   - Confirm addInstance slot behavior and whether zeroing full runtime is sufficient.

2. Finish step-phase integration design.
   - Add planned architecture for a `betweenCollideAndSolve` callback.
   - Ensure proxy and constraint writes happen in the same phase.

3. Finish proxy design.
   - Choose no-contact filter/layer.
   - Choose tiny/minimal shape.
   - Decide whether existing generated keyframed drive can be reused or needs a proxy-specific drive.

4. Continue FO4VR-native feature research.
   - `hknpMalleableConstraintData` solver input semantics;
   - `hknpBreakableConstraintData`.

5. Only then create implementation branch from `develop`.
   - Suggested branch remains `feature/custom-dynamic-grab-authority`.

## Open Questions

- Does FO4VR's hknp bridge require doubled runtime memory for custom hkp constraints, as HIGGS did, or can it use exact runtime size safely?
- Is ROCK's current custom add-instance callback ever called during actual constraint insertion, and if yes, what exact FO4VR call signature reaches vtable `+0xA0`?
- Can `hknpMalleableConstraintData` scale a custom motorized hkp constraint in a useful way?
- What exactly does the malleable wrapper scale at solver/build input offset around `+0x24`?
- Is `FO4_LAYER_NONCOLLIDABLE` safe for a constrained keyframed proxy body, or should ROCK add a dedicated no-contact layer/matrix row?
- Should proxy drive use `CollisionObject_DriveToKeyFrame`, direct velocity, or a specialized between-collide-and-solve keyframe update?
- Should loose weapon dynamic grab add lever-length-aware angular force scaling beyond mass cap, using grip-to-COM only as weight data?

## Immediate Next Research Actions

- Inspect custom constraint addInstance/runtime usage deeper in FO4VR.
- Inspect `hknpMalleableConstraintData` solve behavior and fields.
- Inspect no-contact layer behavior and generated body constraints.
- Inspect direct hknp body creation or no-shape alternatives for a non-contact authority proxy.
- Inspect native APIs for changing/limiting dynamic body motion properties, velocity caps, and inertia safely during held state.
- Update the old tracker only to point at this file as superseding if needed.
