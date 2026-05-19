# Held Object Feel: HIGGS Comparison Research

This note exists because long loose objects such as shovels and axes, and very small loose objects, are not failing in the same way as the palm/proxy snap issue. The current symptoms point at handling feel around the dynamic grab motor: object size, lever arm, mass budget, inertia, release velocity, and connected-body ownership. The research below compares current ROCK against the approved local HIGGS source and keeps the recommendation FO4VR-native instead of copying old keyframed HIGGS behavior back into production.

## Scope

- Local ROCK source: `E:\fo4dev\PROJECT_ROCK_V2\ROCK`
- Local HIGGS source: `E:\fo4dev\skirymvr_mods\source_codes\higgs`
- Web was not used.
- Ghidra was not used.
- HIGGS keyframed object hold code was read only as context. Per workspace rule, production HIGGS behavior should be treated as `ForcePhysicsGrab=1`, so ROCK should not reintroduce a keyframed loose-object hold to fix this.

## Current ROCK Baseline

ROCK already has the main dynamic-grab pieces that matter for held-object feel:

- One-hand and shared loose-object grabs use the proxy-backed dynamic constraint path. The hidden proxy follows the corrected live palm anchor; rotation still uses the root-flattened raw hand/controller frame, which is the stable path after the snap investigation. See `src/physics-interaction/grab/README.md`.
- The grab motor has finite linear/angular motors, mass caps, fade-in angular force, adaptive tau, and loose-weapon multipliers. The INI authority is in `data/config/ROCK.ini` around the "Grab Drive Feel" section.
- `GrabMotionController.h` already has:
  - adaptive tau based on position/rotation error;
  - force capped by object mass;
  - angular force derived from linear force ratio;
  - long-object angular speed scaling from grip-to-extents lever length.
- `HandGrab.cpp` stores `_grabFrame.longObjectLeverGameUnits` from the local mesh and uses it when capping the direct angular velocity path.
- Release velocity already uses HIGGS-like ingredients: hand local velocity, held-object local velocity history, player-space velocity, and COM-relative angular swing.
- Multipart held body handling was recently tightened: mass summary, velocity writes, angular writes, activation, body flags, inertia, release, and nearby damping now use the accepted held body set instead of only the selected primary body.
- ROCK has contact patch and multifinger evidence. It can choose mesh-backed pivots and reject bad close grabs.

So the missing piece is probably not "make grab dynamic" or "make max force bigger." ROCK already has those. The remaining weirdness is more likely that every non-weapon loose object still receives one broad motor policy, with only a single long-object angular speed cap and loose-weapon multipliers.

## What HIGGS Does Around Held Objects

### 1. HIGGS classifies when an object needs physics grab

`Hand::ShouldUsePhysicsBasedGrab` returns true for forced physics grab, actors, and constrained objects such as books, skulls with jaws, wagons, and ragdolls. Source: `higgs/src/hand.cpp:1145`.

Even though ROCK is already dynamic for loose grabs, the important HIGGS concept is not the split into keyframed vs physics. It is that constrained or multipart objects get their own handling branch. ROCK should keep that idea as object feel classification, not as a return to keyframed holds.

### 2. HIGGS chooses the grabbed rigid body from visual geometry

`GetRigidBodyToGrabBasedOnGeometry` finds the closest point on graphics geometry and maps it back to a rigid body. For skinned geometry it accumulates bone weights, biased by distance to the actual grabbed point, and chooses the best collision parent. Source: `higgs/src/hand.cpp:1160`.

ROCK has `GeometryBodyResolver.h`, contact patch pivots, triangle-owner priority, and selected/nearest fallback. ROCK deliberately does not currently trust unverified skinned-weight ownership. That is correct for FO4VR safety, but it means some odd-shaped or skinned/multipart refs may still be using a less specific body authority than HIGGS had.

### 3. HIGGS collects the constraint-connected body component

HIGGS calls `CollectAllGrabbedRigidBodies` on grab, update, and release. That walks rigid bodies connected by constraints, skips true non-moveable bodies, and also includes downstream non-moveable bodies that follow the grabbed component. Source: `higgs/src/utils.cpp:1221`.

It then applies held-object logic across that component:

- ignored hand-collision list;
- contact listener registration;
- contact callback delay forced to zero;
- inverse inertia saved and normalized across all connected bodies;
- release restoration delayed if the other hand still owns the same connected component.

ROCK now has accepted body-set ownership across scene-tree bodies and owner-node fallback, but it does not yet have a verified hknp equivalent of HIGGS' constraint graph traversal. That matters for complex long objects and props with linked child bodies because the motor may be correct for the chosen body while the object behaves like a larger physical component.

### 4. HIGGS clamps inertia on held physics bodies

At physics-grab creation HIGGS saves each connected body's inverse inertia, clamps each axis to be within `grabbedObjectMaxInertiaRatio`, and enforces `grabbedObjectMinInertia`. Source: `higgs/src/hand.cpp:1639`.

ROCK already has grabbed inertia normalization and restoration, but long-object feel may need the next layer: use the same shape classification that identifies a long lever to choose angular motor policy and release policy, not only inertia safety.

### 5. HIGGS uses a startup/fade motor schedule

For physics grabs, HIGGS creates a custom constraint with linear motors and ragdoll angular motors. It updates target orientation and pivot B while held. Source: `higgs/src/constraint.cpp` and `higgs/src/hand.cpp:3914`.

Important HIGGS tuning concepts:

- `grabConstraintFadeInStartAngularMaxForceRatio = 100`
- `grabConstraintFadeInTime = 0.1`
- `grabConstraintAngularToLinearForceRatio = 12.5`
- `grabConstraintMaxForceToMassRatio = 500`
- `grabConstraintLinearMaxForce = 2000`
- `grabConstraintLinearMaxForceWeapon = 9000`
- collision tau uses `grabConstraintCollidingAngularTau` / `grabConstraintCollidingLinearTau`
- body/ragdoll tau starts very stiff then lerps to softer body tau

ROCK already implements much of this, including fade-in angular ratio, mass cap, loose-weapon max-force multiplier, collision tau, and adaptive tau. What ROCK does not yet have is a clean object feel profile that changes these values for "long lever", "tiny/light", "constrained/multipart", and "normal clutter".

### 6. HIGGS damps nearby clutter at grab start

`StartNearbyDamping` queries closest points around the selected collidable and dampens nearby moveable bodies if their linear and angular velocity are already low. Source: `higgs/src/hand.cpp:155`.

ROCK has `NearbyGrabDamping`, implemented as FO4VR hknp motion-property leases instead of direct hkp damping writes. This is already a better FO4VR-native version. It is not the likely missing piece for long/tiny feel unless the damping radius misses the body set or local clutter around a very long prop.

### 7. HIGGS tracks release velocity from multiple sources

On release HIGGS combines:

- hand velocity;
- player velocity;
- tangential velocity from hand angular velocity around the object's center of mass;
- held-object local linear velocity history for physics grabs;
- temporary post-throw hand collision ignore.

Source: `higgs/src/hand.cpp:2973`.

ROCK already mirrors this shape in `GrabHeldObject.h` and `HandGrab.cpp`. The gap is not "add tangential velocity." The gap is that release composition is not currently shape-profiled. A long shovel should not inherit angular/tangential throw energy the same way as a tiny bolt, and a tiny object should not get noisy object-history velocity blended in the same way as a heavy tool.

### 8. HIGGS gives held mass feedback to the player

HIGGS registers held mass every frame and uses total held mass to reduce player speed, with fade-out after release. Source: `higgs/src/main.cpp:253` and `higgs/src/main.cpp:657`.

This does not directly fix solver handling, but it is a real part of HIGGS' "weight feel." ROCK appears not to have an equivalent player movement/jump feedback loop for held loose objects.

### 9. HIGGS has container/player-space support

During `HeldBody`, HIGGS can find bodies contained by the held rigid body, register their mass, and register them for player-space movement compensation. Source: `higgs/src/hand.cpp:3844` and `higgs/src/physics.cpp:1000`.

This is not specifically "shovel and axe" handling, but it is one more surrounding system HIGGS uses to make moving composite physical situations feel less broken.

### 10. HIGGS two-handing is mostly equipped-weapon-specific

HIGGS has equipped weapon two-handing and support hand rotation logic. It also allows both hands to participate in physics-grab objects when the object is already in the physics-grab path. Source: `higgs/src/hand.cpp:1712`, `higgs/src/hand.cpp:2851`, and `higgs/src/hand.cpp:3524`.

ROCK's equipped weapon support solver is already more advanced than HIGGS here. For loose long tools, the useful concept is not copying HIGGS weapon two-handing. It is giving long loose objects a shape-aware handling profile, and optionally allowing a generic second-hand support grip later.

## Likely Cause For Long Object Weirdness

ROCK currently detects a long lever and lowers angular speed. That prevents the far end from sweeping too fast, but it does not fully change the object as a motor problem.

For a long shovel or axe, the hand grip is often far from the center of mass and far from the object's longest extent. The solver sees:

- a small held pivot error near the hand;
- a large far-end sweep;
- high inertia anisotropy;
- large tangential velocity from small hand rotations;
- collision contacts that happen far from the grip point;
- sometimes a multipart or constrained reference where the selected body is not the whole physical object.

Only reducing angular speed handles one symptom. The better policy is to classify the object once at grab commit and use that profile consistently for:

- angular velocity cap;
- angular motor max force;
- angular tau;
- release tangential velocity scale;
- hand visual lerp duration;
- collision tau response;
- second-hand support eligibility.

## Likely Cause For Tiny Object Weirdness

Very small objects suffer from the opposite problem:

- low mass makes `mass * forceToMassRatio` small;
- tiny mesh/contact patch gives noisy pivot/normal evidence;
- small COM-to-grip distance makes rotation visually noisy;
- object velocity history can dominate release behavior;
- collision/contact evidence may be intermittent because the object is smaller than the probe pattern.

Making the global grab stronger would help tiny clutter but hurt normal/light props. The better policy is a tiny-object profile that gives the motor a minimum effective force budget and a minimum grip evidence envelope without changing the object's actual mass or physical release identity.

## Recommended ROCK Direction

### 1. Add `GrabObjectFeelProfile`

Create a pure policy that runs at successful grab commit and stores the result on the grab frame.

Inputs:

- accepted held body set;
- aggregate mass and primary mass;
- mesh triangle bounds;
- grip point local;
- long lever length;
- approximate object radius/cross-section near grip;
- base form kind, especially loose weapon vs clutter;
- constrained/multipart evidence from existing body-set scan;
- current contact patch confidence.

Outputs:

- `Normal`, `LongLever`, `TinyLight`, `ConstrainedMultipart`, and combinations where needed;
- effective motor mass for force budgeting only;
- linear and angular force scales;
- angular velocity cap scale;
- release tangential scale;
- release object-velocity blend;
- hand-lerp timing scale;
- diagnostic labels.

This keeps the fix senior-quality because all downstream systems consume one decision instead of growing scattered special cases.

### 2. Use the profile in motor solve

Feed profile outputs into `GrabMotionController::MotorInput`.

Long lever:

- keep translation authority stable at the grip;
- reduce angular max force and angular velocity cap together;
- keep angular tau from becoming too aggressive when collision is on the far end;
- reduce release tangential scale as lever length grows.

Tiny/light:

- apply a minimum effective motor mass or minimum force floor for motor budgeting only;
- keep fade-in active so tiny objects do not snap;
- reduce noisy object-velocity blend on release;
- use a minimum contact/grip radius for evidence, not for the actual constraint pivot.

### 3. Add a shape report to grab debug

Log one compact line at grab commit:

`feelProfile`, mass, effective motor mass, mesh extents, long lever, grip eccentricity, force scales, release scales, body count, motion count.

This gives immediate evidence for "three good side grabs and three bad top/long/tiny grabs" without re-enabling thousand-line telemetry.

### 4. Do not blindly copy HIGGS connected-component traversal yet

HIGGS' `CollectAllGrabbedRigidBodies` is conceptually important, but hkp constraint graph traversal does not directly map to FO4VR hknp without verification. ROCK should keep the current accepted body-set path, then add hknp constraint-component expansion only after mapping the local hknp data safely.

### 5. Add held mass feedback as its own feature

Player speed/jump feedback is part of weight feel, but it should be a separate coherent feature from solver tuning. It can consume the same `GrabObjectFeelProfile` and aggregate held mass without changing object handling physics.

## Prioritized Findings

1. Highest value: add a first-class grab feel profile and wire it into motor force/tau/angular cap/release velocity.
2. High value for long shovels/axes: shape-aware angular force and release tangential scaling, not only angular speed cap.
3. High value for tiny clutter: motor effective-mass floor and grip evidence envelope floor, without mutating real mass.
4. Medium value: compact profile debug logging for grab commit.
5. Medium value: hknp constraint-connected component expansion, but only after mapping/verifying the FO4VR hknp data path.
6. Separate feature: held mass feedback to player movement/jump for weight feel.

## Implementation Guardrails

- Keep the raw-hand rotation hybrid stable path intact.
- Keep proxy/palm translation authority unchanged.
- Do not globally raise `fGrabConstraintMaxForce`; route strength through object profile.
- Do not use COM as the grip pivot.
- Do not change actual object mass for tiny objects; use effective motor mass only.
- Do not copy HIGGS keyframed hold behavior.
- Keep logs compact and off unless the existing grab debug config enables them.
