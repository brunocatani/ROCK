# 2026-06-20 - Pull-Catch Dynamic Grab Seating Plan

## Scope

Project: ROCK.

Scope is intentionally limited to dynamic pull-to-grab behavior.

Excluded:

- Authored grab nodes.
- Attach-point placement.
- Keyframed/legacy held-object behavior.
- HIGGS 1:1 porting.

Source authority used for this note:

- Current ROCK local source.
- Local HIGGS source at `F:\fo4dev\skirymvr_mods\source_codes\higgs`, as explicitly approved by the user for this research.

No web, Ghidra, or FO4 Mods MCP evidence was used.

## Local Research Summary

HIGGS dynamic pull does not simply put the object center/COM on the palm.

For pull transit, HIGGS stores the selected point as an offset from the rigid body motion position:

- `pulledPointOffset = selectedObject.point - hkObjPos`
- pull updates use `hkObjPos + pulledPointOffset`

That means the selected point is the pull transit authority, not necessarily COM.

On catch/grab, HIGGS does not blindly keep the original far ray point as final hand authority:

- It can catch with a fresh close-cast hit point.
- If normal close detection misses but the pulled object is inside the wide catch radius, it falls back to rigid body translation as the catch point.
- It applies `pulledGrabHandAdjustDistance` for pulled grabs before final geometry processing.
- It recomputes a visual geometry point near the palm line and uses that for finger pose and final relation.
- The final dynamic hold is maintained through a body-space grab constraint and desired hand/object relation, not by treating COM as the universal pivot.

The useful non-authored/non-keyframed behavior to preserve from HIGGS is:

```text
pull selected point -> catch with fresh near/wide evidence -> bias pulled object -> re-solve final seat near palm -> freeze dynamic relation
```

## Current ROCK Gap

ROCK already has a HIGGS-like pull transit point:

- It chooses a selection distance anchor.
- If a hit/contact point exists, that point wins.
- It stores `_pullPointOffsetHavok = selectedPointHavok - motion->position`.
- It drives `motion->position + _pullPointOffsetHavok` toward the hand.

The problematic step is pull arrival and grab capture:

- Arrival writes the pulled object point back into `_currentSelection.hitPointWorld`.
- Grab capture initializes `grabGripPoint` from `sel.hitPointWorld`.
- Dynamic seating uses `shiftObjectToAlignGripWithPocket(bodyWorld, pivotA, grabGripPoint)`.

So if the far ray selected a bad corner/end/surface point, that same point can become the frozen dynamic pivot unless pinch/support/loose-weapon authority replaces it.

This explains weird pull-to-grab placement:

```text
pull selected point -> arrival writes selected point -> final grip point = selected point -> object is translated so that exact point equals palm pocket
```

## Implementation Plan

Update after rollback to `dfefc20` and the follow-up research:

The implementation should not add another independent grab path. The missing
piece is a stricter invariant across the existing ROCK path:

```text
pull transit point stays object-local evidence
fresh palm-pocket/pinch/support evidence chooses the final seat
the frozen desired relation drives both physics and acquisition finger pose
```

The first implementation pass should therefore:

- preserve the pulled surface point in `PullCatchIntent` as both world and
  primary-body-local evidence;
- make wide reacquire restore that preserved point instead of replacing it with
  body origin;
- prefer fresh palm-pocket mesh surface over transit/ray evidence for
  pull-catch final seating;
- publish a conservative acquisition grab pose from the frozen desired object
  relation while the dynamic body converges to `TouchHeld`;
- keep pinch pocket, authored nodes, loose weapon attach, and ordinary close
  grabs on their existing authority path.

### 1. Split Pull Transit Point From Final Seat Point

Do not let pull arrival turn the transit point into final grip authority by default.

Add or extend pull-catch state with explicit fields:

- `arrivalTransitPointWorld`
- `arrivalPrimaryBodyId`
- `arrivalBodyWorld`
- `arrivalMotionWorld`
- `transitPointSource`
- `transitPointIsFinalGripAuthority = false`

`_currentSelection.hitPointWorld` may remain for diagnostics/distance, but grab capture must be able to identify that it came from pull transit and is not final seat authority.

### 2. Add A Pull-Catch Dynamic Seat Resolver

Add a small resolver, likely in `GrabThreePhase.h` or a nearby grab helper.

Inputs:

- live palm pocket frame
- live hand/proxy authority frame
- live body transform
- motion/COM position
- pull transit point
- object body id
- target kind
- available touch/contact/pocket evidence
- mesh/contact patch/support evidence already computed by capture
- `rockPulledGrabHandAdjustDistanceGameUnits`

Output:

- `seatPointWorld`
- `seatNormalWorld`
- `seatPointTrusted`
- `normalTrusted`
- `pivotAuthoritySource`
- `phase`
- `requiresSettledVisualRelation`
- `reason`

Core invariant:

```text
The pull transit point is evidence only. It is not final dynamic pivot authority unless fresh catch-time evidence validates it.
```

### 3. Re-Solve Final Seat At Catch Time

When `grabbedFromPullCatch` is true, run the resolver before freezing `_grabFrame`.

Candidate order:

1. Fresh stable palm/pocket contact evidence.
2. Pinch/support/contact patch evidence already available from the capture pass.
3. Closest current object surface near the palm pocket or palm ray.
4. Body transform or motion/COM position as fallback seed only.
5. If no trustworthy seat exists, start as `NearConverging` and require seated promotion.

Do not use the old far ray point as final authority merely because it exists.

### 4. Move Pulled Adjustment Before Final Freeze

ROCK already has `rockPulledGrabHandAdjustDistanceGameUnits`, but the current late application translates an already chosen point.

For HIGGS-like behavior, apply the pull-catch adjustment during resolver candidate evaluation, before selecting/freezing the seat point.

The adjustment should influence which surface/support point is selected, instead of translating a bad frozen pivot after the fact.

### 5. Freeze Dynamic Relation From The Resolved Seat

After the resolver returns a usable dynamic seat:

- set `grabGripPoint = seatPointWorld`
- set `gripNormalWorld = seatNormalWorld` when trusted
- compute `selectedPivotBBodyLocalGame` from `seatPointWorld`
- compute `desiredBodyWorld = shiftObjectToAlignGripWithPocket(grabBodyWorldAtGrab, grabPivotAWorld, seatPointWorld)`
- freeze `_grabFrame` from this resolved seat

If the resolver only has weak evidence:

- set capture phase to `NearConverging`
- mark `requiresSettledVisualRelation = true`
- keep using existing seated palm-pocket promotion to retarget once the object settles.

### 6. Keep Existing Good Paths Stable

Do not disturb:

- instant touch-held objects already inside palm or pinch pocket
- valid pinch/support authority
- loose weapon primary attach handling
- current dynamic constraint/body-frame convention
- current hknp BODY frame authority rule

The change should only affect pull-catch grabs where the active pivot would otherwise come from pull/ray selection.

### 7. Add Focused Tests

Add unit/policy coverage for the resolver:

- awkward far ray point does not become final pivot on pull-catch
- stable pocket/contact evidence allows immediate `TouchHeld`
- weak or missing normal starts `NearConverging`
- body/motion position is fallback seed only, not trusted surface authority
- pulled adjustment affects pre-freeze seat selection
- non-pull immediate grabs preserve current behavior

### 8. Add Runtime Diagnostics

Add one clear runtime log around pull-catch seat resolution:

```text
PULL CATCH SEAT: transit=(...) seat=(...) source=... phase=... reason=... delta=... adjust=...
```

Include:

- distance from transit point to final seat
- distance from final seat to palm pocket
- whether final seat came from fresh pocket/contact/support/fallback
- whether `TouchHeld` was allowed immediately
- whether settled visual relation was required

## Validation Plan

For implementation:

1. Check `git status` before editing.
2. Implement narrowly in ROCK dynamic grab/pull-catch code.
3. Run relevant unit tests.
4. Run the required auto-deploying fast preset:

```bat
cd ROCK && cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m
```

5. Inspect diff and final `git status`.
6. Commit code and this explicitly ordered implementation note.

## Implementation Pass - 2026-06-20

Source used: current ROCK local source and local HIGGS source already approved in the conversation for comparison. No web, Ghidra, or FO4 Mods MCP evidence was used.

Implemented invariant:

- pull start preserves the selected transit point as world and body-local evidence;
- pull arrival and wide close reacquire restore that preserved point instead of silently replacing it with BODY origin;
- dynamic pull-catch final seating prefers fresh palm-pocket mesh evidence when available;
- preserved transit evidence is fallback evidence only, not final dynamic pivot authority;
- pre-touch pull-catch finger posing may publish a conservative acquisition pose from the frozen desired relation;
- ordinary near/far dynamic grabs still defer mesh finger probe publication until `TouchHeld`;
- pinch pocket, instant touch-held, loose weapon attach, authored-node removal policy, and current BODY-frame drive authority remain outside this change.

Validation targets:

- pure policy tests for pull-catch seat source selection;
- source-boundary tests proving transit preservation, fresh palm-pocket preference, and the pull-catch-only finger pose exception;
- `custom-tests` build plus `ctest`;
- required `custom-fast` ROCK build, which auto-deploys `ROCK.dll`/`ROCK.pdb` to the configured mod path;
- post-commit regression check.
