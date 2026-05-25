# Hand Contact Layer Split And Visual Stop Plan

Date: 2026-05-25
Project: ROCK
Scope: generated hand colliders, hand-hand contact, hand-world visual stopping, soft-contact visual authority, grab pivot authority boundaries
Source used: current local ROCK source on the checked-out branch
Confidence: design plan; implementation still needs local build/tests and runtime validation

## Problem

ROCK currently has generated hand collider bodies, but both hands use the same collision layer:

- `ROCK_LAYER_HAND = 43`
- `ROCK_LAYER_WEAPON = 44`
- `ROCK_LAYER_BODY = 47`
- matrix rows `48..63` are available ROCK-owned extended rows

`buildRockHandExpectedMask()` removes `ROCK_LAYER_HAND` from the hand mask. That is correct for the current single-layer design because enabling layer 43 against itself would make the palm/finger bodies inside the same hand collide with each other.

The result is that native Havok cannot produce clean right-hand-vs-left-hand contact from the actual generated hand colliders. `SoftContactRuntime` therefore builds a separate simplified visual-only hand model from live skeleton points:

- one palm-ish capsule from raw hand to grab anchor;
- finger capsules from the root-flattened finger snapshot;
- hardcoded visual radii.

That gives deterministic visual contact, but it duplicates hand shape truth and can make the visual hand twitch when the best capsule/normal changes frame to frame.

The desired result is smoother visual hand self-interaction using the same generated collider model that ROCK already trusts for physics, while avoiding same-hand self-collision and avoiding direct raw solver jitter in the rendered hands.

## Strategy

Split generated hand colliders into side-specific ROCK-owned layers:

```text
43 = reload/tool compatibility layer, not live generated hand bodies
44 = ROCK weapon generated bodies
47 = ROCK body bone colliders
48 = ROCK right hand generated colliders
49 = ROCK left hand generated colliders
50..63 = reserved ROCK extended capacity
```

Then configure the collision matrix so:

- right hand collides with left hand;
- right hand does not collide with right hand;
- left hand does not collide with left hand;
- both hands collide with weapon, body, actors, dynamic props, and static world according to current config gates;
- body colliders collide with both hand layers, like they currently collide with the single hand/tool layer;
- layer 43 remains only for reload/tool compatibility where the code intentionally uses it.

Native contact then becomes the primary evidence source for hand-hand and hand-weapon contact. Visual motion still goes through ROCK soft-contact authority, not directly through raw hknp solver displacement.

## Why Not Just Enable Layer 43 Against Itself

That would create same-hand self-collision:

- palm face vs palm back;
- palm vs thumb pad;
- adjacent finger segments;
- finger tips vs other same-hand bodies.

These generated bodies are keyframed from the tracked hand. If they collide internally, Havok receives contradictory keyframed bodies pushing into each other every physics step. That is exactly the solver noise the current single-layer mask avoids.

Side-specific layers are the clean split: the left hand can hit the right hand, but each hand's own bodies stay invisible to each other.

## Architecture After Conversion

### Layer Policy

`CollisionLayerPolicy.h` becomes the single authority for ROCK generated layers.

Planned constants:

```cpp
ROCK_LAYER_RELOAD_TOOL = 43;
ROCK_LAYER_WEAPON = 44;
ROCK_LAYER_BODY = 47;
ROCK_LAYER_RIGHT_HAND = 48;
ROCK_LAYER_LEFT_HAND = 49;
```

Compatibility naming must be cleaned up rather than hidden. The old ambiguous `ROCK_LAYER_HAND` should not continue to mean "live generated hands" if live hands no longer use layer 43. Either remove it or rename it to the reload/tool role. Do not leave a fallback path where generated hand bodies may silently use either layer 43 or side-specific layers.

New helper policy:

- `handLayerForSide(bool isLeft)`;
- `isRockHandLayer(layer)`;
- `isRockGeneratedToolLayer(layer)`;
- `buildRockRightHandExpectedMask(...)`;
- `buildRockLeftHandExpectedMask(...)`;
- `buildRockHandExpectedMaskForSide(isLeft, ...)`.

The expected masks should deliberately include the opposite hand layer and exclude the same hand layer.

### Generated Hand Body Creation

`HandBoneColliderSet` should create each hand's hknp bodies with a side-specific filter:

```text
right hand bodies -> layer 48
left hand bodies  -> layer 49
```

The current local constant in `Hand.h`:

```cpp
ROCK_HAND_LAYER = 43
```

should be removed or replaced by a call into `collision_layer_policy`. `HandBoneColliderSet.cpp` should not hardcode the filter layer.

### Contact Routing

Contact routing already identifies hand bodies primarily by body ID metadata, not only by layer. That is good and should remain the source of side truth.

The layer split still requires updates in:

- prefilter and debug logs that mention `ROCK_LAYER_HAND`;
- object body filtering that treats ROCK-generated bodies as non-grabbable;
- native player collision suppression policy;
- body/hand/weapon matrix drift checks;
- tests that assert ROCK hand layer preservation.

`isRockOwnedReusableLayer()` must include both new hand layers and the reload/tool layer where appropriate. Any code that means "actual generated hand body" should use `isRockHandLayer()` instead of the broader reusable-layer predicate.

### Body Colliders

`ROCK_LAYER_BODY` should include both hand layers in its expected mask. Body colliders should not depend on the old single hand layer.

Expected body pair checks should become:

```text
body <-> right hand: enabled
body <-> left hand: enabled
body <-> weapon: enabled
body <-> body: existing policy
body <-> static/animstatic/clutter: existing policy
body <-> query/controller: disabled
```

This makes body interaction demonstrate the same side-specific contact model as hands.

### Soft Contact Runtime

The old simplified hand-hand capsules should be removed as the primary hand-hand path.

`SoftContactRuntime` should consume native contact evidence from the side-specific generated hand bodies. For hand-hand contact, the visual solver should use:

- source hand side from `HandColliderBodyMetadata`;
- source role, finger, segment;
- target hand side from metadata;
- target role, finger, segment;
- contact point and normal from native evidence when available;
- sampled generated-body velocity when available;
- fallback query only if native contact is unavailable for world surfaces, not as a duplicate hand-hand implementation.

Do not keep the old visual capsule hand-hand solver as a fallback. If native side-specific hand evidence is unavailable, hand-hand soft contact should simply be inactive and log/debug the missing evidence when diagnostics are enabled.

World soft contact may still keep query/cached-plane support because world contact has a different problem: once the hand is visually stopped at a wall, the visible hand may no longer penetrate enough to keep native contacts alive every frame. That query path should be documented as the wall-stop persistence/probe path, not a legacy hand-hand fallback.

## Visual Stop Model

The important distinction:

- raw controller/FRIK sample is the requested hand target;
- generated hknp hand bodies are contact sensors and interaction bodies;
- the rendered hand receives a clamped visual target;
- stronger systems such as grab, support grip, and held-object authority override soft contact.

### Contact State Per Hand

Each hand should own a fixed-size, allocation-free contact state:

```text
rawHandWorld
requestedGeneratedColliderFrames
activeContactManifold
latchedWorldPlane/contact body
smoothedNormal
smoothedCorrection
clampedVisualHandWorld
enter/exit counters
release decay timer
stronger-owner suppression state
```

The visual stop should be stateful. It must not pick a totally new point/normal every frame and immediately apply it.

### Hand-Hand Stop

For hand-hand self-interaction:

1. Native contact callback records side-specific generated body contact.
2. Soft-contact runtime groups records by hand pair and semantic role.
3. Solver builds a small contact manifold instead of a single strongest contact.
4. Opposite-hand correction is computed with compliance:
   - shallow overlap: soft pushback;
   - deep overlap: ramps to hard stop;
   - same-frame normal flips are rejected or smoothed;
   - release decays over a few frames.
5. FRIK external hand transform receives the clamped visual target.

Expected visual result:

- hands stop gently when touching each other;
- fingers/palm do not buzz between capsule approximations;
- hand-hand contact does not make one hand's own fingers fight its own palm;
- the hand can slide along the other hand instead of snapping away from it.

### Wall And Counter Stop

For walls/counters, the goal is that the hand appears to stop at the surface instead of following the raw tracked hand through it.

Detection sources:

1. native generated hand vs static/animstatic contact evidence;
2. cached plane from the last valid world contact;
3. swept/rest query probes only for world surfaces when native contact drops out.

Stop logic:

1. When a generated hand body contacts a world surface, latch the target body, surface point, normal, and source hand role.
2. Convert the raw hand target into a corrected/clamped visual target by removing only the illegal inward component along the surface normal.
3. Preserve tangential motion so the hand slides along walls/counters.
4. Keep the cached plane active while the raw hand target is still trying to move into the same surface and tangent drift remains within the configured limit.
5. Release only after penetration is gone for a small exit window or the raw hand moves away from the surface.

This means the visible hand stops, while raw tracking is still remembered. The raw sample remains necessary to know whether the player is still pushing into the wall or has pulled away.

### Should The hknp Hand Bodies Stop Too?

For a pure visual stop, the generated hknp bodies can continue to be driven from the raw tracked hand and only provide contact evidence. This is simple and reliable, but debug overlays may show bodies penetrating while the rendered hand is clamped.

For a more coherent "what you see is what collides" result, the later implementation should drive generated hand bodies from the same clamped target whenever soft contact owns the hand. To keep contact from disappearing when the bodies are held outside the wall, ROCK must retain the raw hand sample and cached-plane/query evidence separately.

Recommended staged behavior:

1. First implementation: native side-specific hand layers, native hand-hand evidence, smoothed visual clamp only.
2. Second implementation: optional clamped generated-body drive for world stop, with raw probe/cached-plane persistence.
3. Only promote clamped generated-body drive to default after runtime testing proves it does not break grab detection, haptics, or hand-world contact continuity.

No hidden fallback should remain: if staged behind a config flag during development, the flag needs a removal condition before release.

## Body And Arm Demonstration

The body/arm visual follow comes from FRIK hand authority:

1. ROCK computes a clamped external hand transform.
2. FRIK receives that hand transform through `FrikVisualAuthorityBridge`.
3. FRIK arm/body IK follows the stopped hand target instead of the raw penetrated target.
4. ROCK body bone colliders continue to follow the resulting skeleton/provider sample on the next frame.

Expected demonstration:

- push the right hand into a wall: rendered hand stops on the wall, arm visually compresses/follows through FRIK instead of the hand passing through;
- slide along a counter: rendered hand remains on the surface with stable normal, not buzzing in/out;
- touch left palm/fingers with right fingers: rendered hand response is soft and semantic, no same-hand self-collision;
- grab start while touching: grab authority clears lower-priority soft contact and uses the normal grab proxy/pivot path.

This is not full-body active ragdoll. ROCK should not pretend the whole player body is physically blocked by the hand unless a separate body reaction feature is implemented. For this work, "body follows" means FRIK visual arm/body posing follows the clamped external hand target and body colliders update from that provider pose.

## Grab Pivot Authority Impact

Dynamic grab pivot authority must keep its current ownership model:

- palm-anchor generated body frame remains the live physical source for proxy authority;
- raw-roll palm-pocket startup pivot remains the mesh/contact evidence source where currently required;
- BODY-local pivot B remains frozen once a grab commits;
- contact patches remain validation/pose evidence unless explicitly promoted by current policy.

Layer split affects grab pivot authority in two ways:

1. Palm anchor body filter layer changes from the old shared hand layer to the side-specific hand layer.
2. Any body classification or suppression code that special-cases `ROCK_LAYER_HAND` must recognize both side-specific hand layers.

The layer split must not reintroduce these old behaviors:

- raw controller fallback for proxy authority;
- sampled palm target as active body-A frame;
- generic contact-patch pivot replacement;
- duplicate hand-collider path for grab vs soft contact.

Grab authority proxy debug overlays should be updated to show:

- raw hand frame;
- live palm-anchor body frame;
- clamped soft-contact visual hand frame when active;
- proxy authority target frame;
- contact normal/manifold if soft contact is active.

That makes it obvious which system currently owns the hand.

## Cleanup Plan

Remove or rename the old ambiguous hand-layer concepts:

- remove `ROCK_HAND_LAYER = 43` from `Hand.h`;
- stop creating generated hand bodies with layer 43;
- rename layer 43 to a reload/tool-specific name if source compatibility allows it;
- remove old hand-hand capsule solver as the primary self-contact path;
- remove any fallback that silently chooses old simplified capsules when native side-specific hand evidence is missing;
- update tests so side-specific layers are required.

Keep only purposeful world query support:

- world query/cached-plane is not a legacy fallback;
- it exists because visual wall stops need persistent surface evidence even when native contacts naturally drop after the hand is clamped outside the wall.

## Implementation Steps

1. Add side-specific hand layer constants and policy helpers in `CollisionLayerPolicy.h`.
2. Update ROCK matrix registration:
   - apply right hand row;
   - apply left hand row;
   - preserve reload/tool row separately;
   - include both hand rows in body/weapon/contact policies where appropriate.
3. Update drift detection and registration logging to report both hand layers.
4. Change `HandBoneColliderSet` to create bodies using `handLayerForSide(isLeft)`.
5. Update object/body classification to treat the new hand layers as ROCK-generated, non-grabbable bodies.
6. Update native player/controller suppression policies to preserve both hand layers.
7. Update contact routing/tests for side-specific hand body layer expectations.
8. Refactor `SoftContactRuntime`:
   - consume native hand-hand evidence from generated hand bodies;
   - add fixed-size hand contact manifold state;
   - add smoothing, hysteresis, normal stability, and release decay;
   - remove simplified hand-hand capsule fallback.
9. Keep world cached-plane/query path, but document it as wall-stop persistence.
10. Add debug overlay state for raw, clamped, contact, and proxy authority frames.
11. Run targeted tests and source-boundary tests.
12. Build `custom-fast` only when asked, because that preset auto-deploys.

## Validation Plan

Policy/source tests:

- layer constants and masks;
- right hand does not collide with right hand;
- left hand does not collide with left hand;
- right hand collides with left hand;
- body collides with both hand layers;
- reload/tool layer remains intentionally separate;
- generated hand bodies do not use layer 43;
- old soft hand-hand capsule fallback is removed;
- pivot authority still resolves from live palm anchor and does not fall back to raw controller space.

Build:

```bat
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
```

Runtime validation:

- enable debug draw for hand bone colliders, soft contacts, and grab authority proxy;
- verify separate right/left hand layer IDs in contact logs;
- push hand into wall and counter;
- slide along wall/counter;
- touch hands palm-to-palm and finger-to-palm;
- start grab while touching object/wall/other hand;
- test equipped weapon and offhand support grip because those are stronger owners;
- test body colliders against hands.

Release/deploy:

- do not run `custom-fast` unless deploy is requested, because it auto-deploys;
- when release build is requested, verify deployed DLL/PDB metadata after build.

## Expected Result

The visible hand should no longer look like it is directly following every small collision normal change. It should:

- stop against the other hand using real generated hand collider evidence;
- stop against walls/counters with a stable cached surface;
- slide tangentially instead of popping away;
- release smoothly instead of snapping back on the first missing contact frame;
- avoid same-hand self-collision entirely;
- preserve grab, support grip, held-object, and weapon ownership boundaries.

The underlying design becomes cleaner: generated hand collider geometry is the contact source of truth, soft contact is the visual smoothing/authority layer, and legacy layer 43 no longer ambiguously means both "generated hand" and "reload/tool compatibility."
