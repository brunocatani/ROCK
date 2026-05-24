# Grab Authority Frame Simplification Plan

Project: ROCK
Date: 2026-05-24
Status: implementation plan
Scope: normal close/object grab authority from convergence through `TouchHeld`
Out of scope: far grab, authored grab nodes as a special authority path, weapon-specific authored handling

## Source Authority

- User direction for this task.
- Current local ROCK source and tests.
- Current local ROCK implementation notes:
  - `docs/implementation-notes/grab-drive-simplification-2026-05-21.md`
  - `docs/implementation-notes/grab-force-authority-redesign-2026-05-21.md`
  - `docs/implementation-notes/GRAB_SUPPORT_MODEL_2026-05-21.md`
- Approved local HIGGS source review from `F:/fo4dev/skirymvr_mods/source_codes/higgs`.

No external web source was used. No Ghidra or FO4 Mods MCP verification was used for this plan.

## Problem

ROCK currently has too many concepts that can influence grab authority:

- mesh point selection;
- palm pocket selection;
- pinch pocket selection;
- grip support model selection;
- contact patch samples;
- contact patch normals;
- position-only pivot flags;
- seated palm-pocket promotion;
- held support refresh;
- motor scaling from pivot confidence;
- visual hand evidence;
- release safety.

Some of these are useful as evidence, diagnostics, visual hand support, or release safety. They are not all valid solver authority.

The failure mode is that a small patch, narrow triangle, bad normal, or partial seated update can still be allowed to participate in active authority. When that happens, the grab can technically be "held" while the solver gets a weak pivot or weak angular target. The object then has poor lever behavior even when the user hand motion is correct.

The fix is not another patch rule. The fix is to reduce active authority to one coherent frame:

```text
resolved point world
-> frozen body-local pivot B
-> frozen desired body-in-proxy relation
-> proxy constraint drives that relation
-> hknp mass, COM, and inertia produce lever behavior
```

Evidence either selects a coherent point for that frame, or it fails and the resolver tries the next candidate. Evidence must not become a second motor truth.

## Non-Negotiable Invariant

The selected grab point and all frozen grab relations must be captured from the same authority event.

When freezing a grab, these values must be derived together from the same selected point, same body transform, same proxy/palm frame, and same desired object/body transform:

- `pivotAWorld`
- `pivotBBodyLocalGame`
- `pivotBConstraintLocalGame`
- `bodyLocal`
- `rootBodyLocal`
- `ownerBodyLocal`
- `grabPivotWorldAtGrab`
- `gripPointWorldAtGrab`
- `rawRotationProxyHandSpace`
- `rawRotationProxyBodyHandSpace`

Do not change one of these values without rebuilding the full frozen frame.

Do not retarget only `pivotBConstraintLocalGame`.

Do not retarget only `rawRotationProxyHandSpace`.

Do not refresh only metadata that later feeds motor authority.

Do not convert contact patch, support refresh, finger pose, or normal confidence into a post-freeze motor authority change.

If a pre-`TouchHeld` seated retarget is accepted, rebuild the entire frozen frame atomically from one new selected point and one new desired relation. If it is not accepted, keep the original frozen frame.

After transition to `TouchHeld`, the authority frame is immutable until release.

## HIGGS Reference Logic

The relevant HIGGS behavior is not a contact patch system. It is a frozen relation system:

- HIGGS chooses one point, `ptPos`.
- HIGGS shifts the desired object/node transform so the chosen point aligns with the palm.
- HIGGS stores the desired transform in hand space as `desiredNodeTransformHandSpace`.
- HIGGS computes constraint pivots A and B with body-local transforms.
- HIGGS updates the held object by recomposing the desired transform from the live hand transform and the frozen relation.
- HIGGS gets lever behavior from Havok through the body-local pivot offset from COM, plus inertia normalization and motor force tuning.

Local reference points from the approved HIGGS source review:

- `src/hand.cpp:1326` selects `ptPos` from the closest collision point.
- `src/hand.cpp:1432` searches graphics geometry near the palm line.
- `src/hand.cpp:1446` can replace `ptPos` with the graphics triangle point.
- `src/hand.cpp:1546` shifts desired transform by `palmPos - ptPos`.
- `src/hand.cpp:1547` stores `desiredNodeTransformHandSpace`.
- `src/hand.cpp:1592-1598` computes local pivot A/B.
- `src/hand.cpp:1603-1605` builds the desired Havok body relation and local transform B.
- `src/RE/havok.cpp:126-148` creates the custom constraint and activates motors.
- `src/constraint.cpp:90-93` stores local transforms in the constraint atoms.
- `src/hand.cpp:3917-3926` updates the held target from the frozen relation.
- `src/hand.cpp:1648-1669` saves and normalizes inverse inertia.
- `src/hand.cpp:3968-3986` applies force, angular force ratio, mass cap, and collision tau behavior.

ROCK should not copy HIGGS one-for-one. ROCK already has a proxy authority architecture, hFRIK provider integration, richer mesh evidence, and better cleanup/restore structure. The useful HIGGS invariant is the single frozen body-local pivot plus frozen desired relation.

## ROCK Mapping

```text
HIGGS ptPos
-> ROCK resolved grab authority point

HIGGS local pivot B
-> ROCK pivotBBodyLocalGame / pivotBConstraintLocalGame

HIGGS desiredNodeTransformHandSpace
-> ROCK rawRotationProxyHandSpace and rawRotationProxyBodyHandSpace

HIGGS hand body / constraint body A
-> ROCK hidden grab authority proxy body

HIGGS held constraint update
-> ROCK updateProxyConstraintGrabDriveTarget(...)

HIGGS inertia clamp
-> ROCK inertia normalization / restore policy
```

## What To Keep

Keep these systems, but route them through one authority resolver/freeze path:

- proxy constraint drive;
- body-set prep and restore;
- mesh extraction;
- pinch pocket;
- grip support model;
- palm-pocket mesh point;
- selection mesh snap;
- inertia normalization and restore;
- mass-capped motor force;
- angular force derived from linear force by ratio;
- contact softening for collision response;
- release safety;
- finger-pose solving as visual hand evidence;
- debug drawing and telemetry that do not feed motor authority.

## What To Delete Or Demote

Remove or demote these as active solver authority:

- contact patch as active pivot authority;
- patch sample count as active angular authority;
- patch or triangle normal as held angular authority;
- `positionOnlyPivot` weakening active held motor authority;
- `normalTrusted` weakening active held motor authority;
- held support refresh changing pivot or motor authority after capture;
- partial seated pivot blend;
- any duplicate "weak but active" grab mode;
- any config whose only purpose is to make active motor authority weaker because evidence was weak;
- tests that require weak held motor behavior from patch quality;
- stale comments/docs that describe patch-derived held authority as the intended model.

Contact patches can remain for:

- acquisition evidence;
- visual hand pose evidence;
- debug;
- release safety;
- contact softening;
- future diagnostics.

They must not own the held solver pivot or held angular target.

## Target State Model

Reduce active grab authority to these states:

```text
Invalid
FrozenAuthority
ConvergingVisual
TouchHeld
```

`FrozenAuthority` means the resolver found a coherent point and the freeze function created the body-local pivot and desired relation.

`ConvergingVisual` means the visual hand/acquisition state is still settling. The constraint still drives the same frozen authority frame.

`TouchHeld` means the object is held. The frozen authority frame is immutable until release.

There should not be separate active solver states for "position-only patch", "weak normal", "patch pivot", "support refresh authority", or "partial seated blend".

## New Authority Resolver

Add one resolver:

```cpp
ResolvedGrabAuthorityPivot resolveGrabAuthorityPivot(...);
```

The output must be narrow:

```cpp
enum class GrabAuthorityPivotSource : std::uint8_t
{
    None,
    PinchPocket,
    GripSupportModel,
    PalmPocketMesh,
    SelectionMeshSnap,
    CollisionFallback
};

struct ResolvedGrabAuthorityPivot
{
    bool valid = false;
    GrabAuthorityPivotSource source = GrabAuthorityPivotSource::None;
    RE::NiPoint3 pointWorld{};
    RE::NiPoint3 normalWorld{}; // visual/finger/debug only
    bool normalValid = false;
    RE::hknpBodyId bodyId{};
    const RE::NiAVObject* sourceNode = nullptr; // non-owning frame-local reference only
    const char* reason = "";
};
```

Candidate order:

1. `PinchPocket`
2. `GripSupportModel`
3. `PalmPocketMesh`
4. `SelectionMeshSnap`
5. `CollisionFallback`, only when config allows fallback

Rules:

- Authored nodes are not a special path for this refactor. If authored support is retained later, it must enter through this same resolver/freeze contract.
- Far grab is out of scope.
- A candidate that cannot provide an owner-matched body and finite world point fails.
- A normal can be carried for visual hand pose, but it cannot affect held motor authority.
- Contact patch data can help reject or score candidates, but it cannot be returned as a separate active pivot source unless it has become one of the named mesh-backed candidates above.
- The resolver returns one winner. No blending.

## New Freeze Function

Add one freeze function:

```cpp
FrozenGrabAuthorityFrame freezeGrabAuthorityFrame(...);
```

The output should contain the complete immutable authority packet:

```cpp
struct FrozenGrabAuthorityFrame
{
    bool valid = false;
    GrabAuthorityPivotSource source = GrabAuthorityPivotSource::None;
    RE::hknpBodyId bodyId{};

    RE::NiPoint3 pivotAWorld{};
    RE::NiPoint3 pivotBBodyLocalGame{};
    RE::NiPoint3 pivotBConstraintLocalGame{};

    RE::NiTransform bodyLocal{};
    RE::NiTransform rootBodyLocal{};
    RE::NiTransform ownerBodyLocal{};

    RE::NiPoint3 grabPivotWorldAtGrab{};
    RE::NiPoint3 gripPointWorldAtGrab{};

    RE::NiTransform desiredObjectWorld{};
    RE::NiTransform desiredBodyWorld{};
    RE::NiTransform rawRotationProxyHandSpace{};
    RE::NiTransform rawRotationProxyBodyHandSpace{};

    RE::NiPoint3 visualNormalWorld{};
    bool visualNormalValid = false;
};
```

The freeze function must:

1. Resolve `pivotAWorld` from the active proxy/palm authority frame.
2. Use the selected resolver point as the only world-space grip point.
3. Compute the visual object-to-body relation from the live object/body transforms.
4. Build `desiredObjectWorld` by aligning the selected grip point to the active pocket/proxy target while preserving the selected object relation.
5. Build `desiredBodyWorld` from `desiredObjectWorld * bodyLocal`.
6. Compute `pivotBBodyLocalGame` from the same selected world point and the same body transform used for body B.
7. Set `pivotBConstraintLocalGame` from the same selected world point and the same actual constraint body frame.
8. Build `rawRotationProxyHandSpace` from the same authority frame and `desiredObjectWorld`.
9. Build `rawRotationProxyBodyHandSpace` from the same authority frame and `desiredBodyWorld`.
10. Store debug/visual normal separately from motor authority.

The actual constraint body must own `pivotBConstraintLocalGame`.

Do not use a patch-local, visual-local, evidence-local, or COM-local point as active solver pivot B.

Do not keep separate BODY/MOTION authority paths for active held grabs. Motion/COM data can be telemetry and force/inertia input, not an alternate pivot relation.

## Constraint Creation And Held Update

Proxy constraint creation:

- Body A is the grab authority proxy.
- Body A origin is moved to `pivotAWorld`.
- Pivot A local is zero.
- Body B is the actual held object body.
- Pivot B is `pivotBConstraintLocalGame`.
- Angular target is seeded from `rawRotationProxyBodyHandSpace`.
- Motor tuning uses mass cap, force settings, angular ratio, fade, and inertia policy.

Held update:

- Resolve the live proxy authority frame.
- Recompose `desiredObjectWorld = liveAuthorityFrame * rawRotationProxyHandSpace`.
- Recompose `desiredBodyWorld = liveAuthorityFrame * rawRotationProxyBodyHandSpace`.
- Update transform B / angular target from the recomposed desired body relation.
- Keep `pivotBConstraintLocalGame` unchanged.
- Keep `rawRotationProxyHandSpace` unchanged.
- Keep `rawRotationProxyBodyHandSpace` unchanged.

The held update must not read contact patch quality, patch normal trust, support sample count, or position-only flags to reduce active angular authority.

## Convergence And Seated Retarget

Convergence is a visual/acquisition phase only. It must not create a weaker solver.

Rules:

- The proxy constraint drives the frozen relation during convergence.
- Visual hand publishing may wait for acceptable acquisition evidence.
- Contact and support evidence can decide visual readiness.
- Evidence cannot downgrade the motor target after freeze.

Allow one optional seated retarget before `TouchHeld`:

- It must be bounded by local delta from the existing body-local pivot.
- It must come from a higher-priority or clearly better resolver candidate.
- It must rebuild the full frozen frame atomically.
- It must update all frozen relation fields together.
- It must never blend old and new pivots.
- It must never update only support metadata or only contact patch samples.

If the seated candidate is rejected, keep the original frozen frame. Do not keep waiting forever unless a hard required invariant is missing.

After `TouchHeld`, seated retarget is disabled.

## Motor And Inertia Policy

Active held motor authority should depend on:

- configured linear force;
- configured angular-to-linear force ratio;
- shared-hand force scale;
- fade-in state;
- mass cap;
- minimum effective motor mass;
- inertia normalization;
- collision tau softening;
- hard safety clamps for release and failure cases.

Active held motor authority should not depend on:

- contact patch sample count;
- contact patch normal trust;
- `positionOnlyPivot`;
- grip support confidence;
- single-triangle patch quality;
- visual hand pose confidence.

That does not mean evidence is ignored. Evidence chooses the frozen pivot or rejects a candidate. Once a valid frozen frame exists, the solver authority is the frozen frame plus body mass/inertia.

## Main Files

Primary implementation files:

- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/grab/GrabCore.h`
- `src/physics-interaction/grab/GrabContact.h`
- `src/physics-interaction/grab/GrabMotionController.h`
- `src/physics-interaction/grab/GrabConstraint.cpp`
- `src/physics-interaction/grab/GrabConstraintMath.h`

Primary test files to update or add:

- `tests/GrabAuthorityProxyFrameSourceTests.ps1`
- `tests/GrabMotionControllerPolicyTests.cpp`
- `tests/GrabContactEvidencePolicyTests.cpp`
- a new pure resolver test, for example `tests/GrabAuthorityPivotResolverTests.cpp`
- a new pure freeze invariant test, for example `tests/GrabAuthorityFrameFreezeTests.cpp`

## Implementation Phases

### Phase 1 - Add Authority Contract Tests

Add source-boundary tests before changing behavior:

- reject active held motor reads of patch sample count;
- reject active held motor reads of `positionOnlyPivot`;
- reject active held motor reads of `normalTrusted`;
- reject post-`TouchHeld` mutation of `pivotBConstraintLocalGame`;
- reject post-`TouchHeld` mutation of `rawRotationProxyHandSpace`;
- reject post-`TouchHeld` mutation of `rawRotationProxyBodyHandSpace`;
- require one freeze function to write all frozen relation fields;
- require seated retarget to call the same freeze path instead of mutating fields directly.

This phase protects the most important invariant before cleanup starts.

### Phase 2 - Introduce Resolver Types

Add `GrabAuthorityPivotSource` and `ResolvedGrabAuthorityPivot`.

Implement `resolveGrabAuthorityPivot(...)` as a pure policy boundary where possible. The resolver may call existing mesh/support helpers, but the returned authority packet must be narrow.

Move candidate ordering into one place.

Remove duplicate ordering logic from grab-start paths after callers are moved.

Tests:

- pinch pocket beats all lower-priority candidates;
- grip support beats palm-pocket mesh;
- palm-pocket mesh beats selection mesh snap;
- selection mesh snap beats collision fallback;
- collision fallback is rejected when config disables it;
- invalid/finite/body-owner checks reject bad candidates;
- normal-only evidence never returns valid authority by itself.

### Phase 3 - Introduce Freeze Function

Implement `freezeGrabAuthorityFrame(...)`.

Move all writes of the frozen authority relation into this function:

- pivot fields;
- object/body local relation fields;
- world-at-grab telemetry;
- raw-rotation proxy relation fields;
- visual-only normal metadata.

Any caller that wants a new authority point must call this function and replace the whole frozen packet.

Tests:

- the selected world point round-trips through `pivotBConstraintLocalGame`;
- recomposed desired body places the selected grip point at the target proxy pocket;
- `rawRotationProxyHandSpace` and `rawRotationProxyBodyHandSpace` are derived from the same authority frame;
- changing the selected point requires a full new frozen packet;
- body-local and constraint-local pivots refer to the same selected point in their respective body frames.

### Phase 4 - Wire Grab Start To Resolver And Freeze

Replace grab-start authority construction with:

```text
resolveGrabAuthorityPivot(...)
-> freezeGrabAuthorityFrame(...)
-> createProxyConstraintGrabDrive(...)
```

The old construction code should be removed after the new path is wired. Do not leave a hidden fallback path.

Preserve useful telemetry by copying it into debug-only fields. Do not let telemetry feed motor authority.

Tests:

- source-boundary test requires grab start to call resolver then freeze;
- source-boundary test rejects direct writes of frozen relation fields outside freeze/atomic retarget helpers.

### Phase 5 - Demote Contact Patch Authority

Keep contact patch construction only where it has a useful non-authority role:

- acquisition evidence;
- visual hand pose support;
- debug overlay;
- contact softening;
- release safety diagnostics.

Remove or rename any fields whose names imply active held authority when they are now evidence-only.

Expected removals/demotions:

- `contactPatchUsedAsPivot` as active held authority;
- `contactPatchSampleCount` as held motor authority;
- `contactPatchNormalTrusted` as held motor authority;
- broad patch pivot selection paths;
- tests that expect patch quality to weaken active held angular authority.

If a mesh-backed contact result is good enough to be a point, it must enter as `PalmPocketMesh`, `SelectionMeshSnap`, or another named resolver candidate. It must not remain "patch authority".

### Phase 6 - Remove Held Support Refresh Authority

Audit `updateHeldSupportRefreshFromLivePalm()` and any seated/support refresh path.

After `TouchHeld`, this code may:

- update visual/finger/debug data;
- update contact softening evidence;
- update release safety evidence.

After `TouchHeld`, this code must not:

- mutate pivot B;
- mutate raw proxy relation;
- mutate motor force from support confidence;
- mutate authority source;
- mutate position-only or normal-trust fields that later affect motors.

If the support refresh has no remaining non-authority purpose, delete it.

### Phase 7 - Replace Partial Seated Blend

Retain one optional pre-`TouchHeld` retarget only if it follows the atomic freeze rule.

Remove:

- partial pivot blend;
- patch-only seated upgrade;
- support-only seated authority upgrade;
- waiting loops that keep the grab weak until a patch improves.

Keep:

- bounded local-delta check;
- better-candidate check;
- one atomic rebuild before `TouchHeld`;
- clear logging when accepted or rejected.

### Phase 8 - Simplify Held Motor Inputs

Update `GrabMotionController.h` and callers so active held motor inputs represent motor physics, not evidence quality.

Keep inputs for:

- configured force;
- mass;
- shared force scale;
- fade;
- collision softening;
- inertia/minimum mass policy;
- release clamp policy.

Remove or stop using active held inputs for:

- `positionOnlyPivot`;
- `normalTrusted`;
- `contactPatchUsedAsPivot`;
- `contactPatchSampleCount`;
- support confidence.

If release safety still needs those values, split release safety input from held motor input. Do not reuse the same struct for both if that makes active held authority ambiguous.

### Phase 9 - Cleanup Config, Logs, Docs, And Tests

Remove stale config keys only when they no longer have a runtime role.

Do not edit the production INI unless explicitly requested.

Update packaged config only if a removed key exists there.

Update docs that still describe patch-derived held authority as intended.

Keep logging focused:

- selected resolver source;
- selected world point;
- frozen pivot B;
- body id;
- accepted/rejected seated retarget;
- immutable `TouchHeld` transition;
- force/mass/inertia summary.

Avoid per-frame noisy logs.

## Validation Plan

Build and test commands for implementation work:

```bat
cd ROCK && cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
cd ROCK && ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%
cd ROCK && ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
```

When the user wants a deployable local plugin build, use the standard auto-deploying fast preset:

```bat
cd ROCK && cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m
```

Do not run a release package build unless explicitly requested.

Runtime validation matrix:

- bottle grabbed by tip;
- bottle grabbed by body;
- long handle/rifle/broom style object;
- small coin/cigarette style object;
- uneven low-poly prop corner;
- object grabbed near a wall with collision softening;
- two-hand/shared held object if the touched code overlaps shared grab;
- release throw after strong angular correction;
- rapid grab/release cleanup;
- provider/body loss fail-closed behavior.

Runtime success criteria:

- active held angular authority does not collapse because only one patch/triangle was found;
- object rotates around the selected body-local point with expected lever behavior;
- no pivot jump at `TouchHeld`;
- no post-`TouchHeld` authority mutation;
- inertia restore runs on release/cleanup;
- release clamp still prevents bad throws;
- visual hand may be imperfect during convergence but does not weaken the solver.

## Commit And Rollback Plan

Use focused commits:

1. tests/contracts for frozen authority invariants;
2. resolver/freeze data model;
3. grab-start wiring;
4. contact patch demotion;
5. seated/support refresh cleanup;
6. motor input cleanup;
7. config/docs cleanup.

Each commit should build and pass relevant tests before moving on.

Do not keep the old authority path as a fallback. Git is the rollback mechanism.

If unrelated dirty files appear, stop before committing and report them.

## Final Architecture Rule

The only active held solver truth is:

```text
live proxy frame * frozen desired body-in-proxy relation
with frozen body-local pivot B
```

Everything else is evidence, visuals, telemetry, softening, or safety.

The grab system breaks when the point is frozen but the references and relations are later changed independently. This refactor must make that impossible by construction.
