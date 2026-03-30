# HIGGS Dynamic Grab Investigation - 2026-05-11

## Why This Pass Exists

This pass compares production HIGGS-style dynamic grab quality against ROCK's current FO4VR-native active grab stack because grab feel is not controlled by one motor value. The alternatives considered were a 1:1 HIGGS constraint port, more native mouse-spring tuning, and a cleanup-only pass. The correct approach is to map the full lifecycle first: acquisition, body prep, pivot capture, drive authority, collision/contact handling, player-space compensation, visual hand correction, and release. ROCK already uses FO4VR-native spring drive for ordinary single-hand grabs and shared constraints for two-hand loose-object grabs, so any improvement should preserve the verified authority boundaries unless a later binary-verified architecture decision changes them.

No runtime code was changed in this investigation pass.

## Scope And Rules

- HIGGS was explicitly approved as the reference for this task.
- Web was not used.
- Ghidra was not used. Any claim requiring FO4VR binary layout or native function behavior remains behind the normal Ghidra approval gate.
- HIGGS keyframed object grab code was ignored as a production target because the workspace notes state shipped HIGGS uses `ForcePhysicsGrab=1`.
- ROCK production config was read from `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini`; it was not modified.

## HIGGS Dynamic Grab Quality Map

Production HIGGS dynamic grab is a whole interaction stack, not only a constraint:

- Decision: `Hand::ShouldUsePhysicsBasedGrab` returns true when forced, when grabbing actors, or when the object has internal constraints. In the shipped setup this is forced for all object grabs.
- Prep: selected keyframed bodies are converted to dynamic, debris quality is upgraded when needed, current mass is captured, nearby objects can be damped, and connected rigid bodies are collected.
- Contact and pose: HIGGS extracts skinned/static triangles, finds the closest geometry point, solves finger curl, supports alternate thumb curves, and stores `desiredNodeTransformHandSpace`.
- Constraint: dynamic grabs create a custom constraint from the hand body to the selected object body using a palm pivot on the hand and closest/object pivot on the object. The constraint uses ragdoll angular motors plus three linear motors.
- Startup: if the object starts from a custom alignment, HIGGS fades constraint force and lerps the rendered hand to avoid visible snap.
- Connected bodies: HIGGS adds entity contact listeners to the connected body set, sets callback delay to zero, and saves/restores original callback delay when released.
- Inertia: connected body inverse inertia is clamped by a maximum ratio and a minimum inertia so odd tensors do not dominate the solver.
- Held update: every `HeldBody` frame HIGGS derives `m_adjustedHandTransform = heldObjectWorld * inverse(desiredNodeTransformHandSpace)`, lerps it during startup, checks average deviation, publishes the adjusted hand transform, then updates constraint target orientation and transform-B pivot.
- Motor policy: actors, ragdolls, ordinary objects, weapons, colliding objects, and startup fade use different force/tau policy. Non-actor collision tau drops toward `0.01`; max force is capped by mass ratio.
- Player-space: held/connected/contained bodies are registered for player movement compensation so locomotion and room transforms do not become residual object velocity.
- Release: HIGGS computes release velocity from local hand/object velocity history, angular swing around COM, and player-space velocity, then removes constraints/listeners and temporarily suppresses hand-object collision.
- Pull catch: pulled objects transition into the same held path when caught.
- Equipped two-hand weapon support is separate from loose-object dynamic constraint grab; it stores/restores weapon offsets and wand transforms.

Important HIGGS source anchors:

- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:1145` - physics-grab decision.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:1301` - `TransitionHeld`.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:1592` - palm/object pivots for dynamic constraint.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:1636` - connected-body listener/callback-delay setup.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:1658` - inertia clamp.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:3809` - `HeldBody` update.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:3883` - visual hand back-solve from held object.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:3921` - constraint target update.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:3964` - non-actor force/tau policy.
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\config.h:220` - dynamic grab tau/force/inertia/player-space tuning.

## ROCK Current Authority Map

ROCK is not currently a direct HIGGS constraint clone:

- Ordinary single-hand loose object grabs use FO4VR's native `hknpBSMouseSpringAction`.
- ROCK wraps the native mouse spring at a verified boundary: native action/cinfo size, target transform/position setters, native update call, native body flags, game-to-Havok scale conversion, and the working rotation boundary conversion.
- Single-hand native grab tuning is exposed through native response scales:
  - `fGrabNativeMouseSpringLinearResponseScale = 1.35`
  - `fGrabNativeMouseSpringAngularResponseScale = 0.75`
  - `fGrabNativeMouseSpringAngularClampScale = 0.85`
- ROCK adds a bounded adaptive target lead around the native spring using hand/object velocity, angular velocity, error, and collision lead scaling.
- When the peer hand joins the same loose object, the original native grab is promoted to shared constraint drive and the joining hand creates its own constraint.
- Shared constraints use the HIGGS-like angular/linear motor policy surface: tau, damping, recovery, force, angular-to-linear force ratio, mass ratio cap, collision tau, adaptive force, and fade-in.
- ROCK keeps the visual hand update separate from object motor authority. The visual target is held-object-relative and published to FRIK as an external transform.
- ROCK's active grab point authority is mesh/authored-node/contact evidence driven. The canonical native BODY frame is kept as authority for the spring path, and tests explicitly prevent BODY/MOTION double counting.
- ROCK's actor equipment path detaches/drops equipment into the normal object path. Whole live actor body grabbing is intentionally blocked from the normal active grab path.
- ROCK's weapon domain is separate: generated weapon collision/support behavior is not HIGGS loose-object dynamic grab.

Important ROCK source anchors:

- `src/physics-interaction/hand/Hand.h:114` - `HeldObjectDriveMode`.
- `src/physics-interaction/hand/HandGrab.cpp:3936` - ordinary single-hand native mouse-spring creation.
- `src/physics-interaction/hand/HandGrab.cpp:3984` - native dynamic grab debug log.
- `src/physics-interaction/hand/HandGrab.cpp:4241` - adaptive native target lead.
- `src/physics-interaction/hand/HandGrab.cpp:4296` - shared constraint motor update.
- `src/physics-interaction/hand/HandGrab.cpp:4320` - visual hand target gating.
- `src/physics-interaction/hand/HandGrab.cpp:4345` - held-object-relative visual hand target.
- `src/physics-interaction/hand/HandGrab.cpp:4856` - controller/object/tangential/player-space release velocity composition.
- `src/physics-interaction/grab/GrabHeldObject.h:301` - single-hand native spring design note.
- `src/physics-interaction/grab/GrabHeldObject.h:447` - adaptive held lead helper.
- `src/physics-interaction/grab/GrabHeldObject.h:621` - held-object character-controller contact policy.
- `src/physics-interaction/core/PhysicsInteraction.cpp:1655` - held player-space frame sampling before grab update.
- `src/physics-interaction/core/PhysicsHooks.cpp:1121` - held grab character-controller intervention.

## HIGGS Vs ROCK By Scenario

| Scenario | HIGGS production behavior | ROCK current behavior | Investigation note |
| --- | --- | --- | --- |
| Ordinary loose object, one hand | Custom hand-object constraint | FO4VR native mouse spring with ROCK target shaping | Different drive authority. Tune native response/target shaping and visual hand correction before considering a constraint rewrite. |
| Loose object, two hands | HIGGS has physics grab and two-hand logic | ROCK promotes native single-hand to shared constraints | ROCK already has a stronger explicit peer-held loose-object architecture. |
| Held visual hand | Back-solves from live held object every `HeldBody` frame | Back-solves only in `TouchHeld` or bounded acquisition envelope | Most likely polish delta: HIGGS lets the rendered hand settle to the object continuously. ROCK may be too conservative before/around touch. |
| Collision while held | Per-connected-body listeners, callback delay zero, collision-aware tau | Contact endpoint classification, held-body contact freshness, CC generated-contact filtering, collision lead scaling | ROCK has the pieces, but telemetry needs to separate native collision lead from shared constraint collision tau. |
| Inertia | Clamp connected body inverse inertia | ROCK saves/restores and normalizes held body inertia | Conceptually covered. Verify behavior in game with compound clutter. |
| Player movement | Register held/connected/contained bodies | Held player-space sampling and compensation | Conceptually covered. Any binary-level compensation assumptions need Ghidra before changing native integration. |
| Throw/release | Object velocity history, hand velocity, COM swing, player velocity | Same categories: object, hand, tangential COM swing, player velocity | Conceptually covered. |
| Whole actors/ragdolls | HIGGS supports actor/ragdoll physics grab policy | ROCK blocks live actors from normal active grab; actor equipment becomes dropped ref | This is not a legacy bug. Whole actor interaction belongs to the actor/SCISSORS architecture. |
| Equipped weapons | HIGGS two-hand equipped weapon support stores/restores offsets | ROCK weapon domain owns generated weapon collision/support | HIGGS equipped weapon behavior is not a loose object dynamic grab target. |

## Legacy And Troubleshooting Audit

The active source does not show a hidden legacy held-object keyframed path controlling ordinary ROCK grabs:

- `HandGrab.cpp` creates ordinary single-hand grabs through `HeldObjectDriveMode::NativeMouseSpring` and `_nativeGrab.create`.
- `NativeMouseSpringGrab.cpp` is tested to drive a dynamic body by native action, not keyframing.
- Source tests reject `setBodyKeyframed` in held object grab code.
- Source tests reject removed surface/opposition/pinch/visual-authority state in the canonical grab frame and runtime path.
- Source tests require native BODY authority, body-local identity for the native path, and held-object-relative visual hand target.
- Generated keyframed body drive exists for generated collider bodies, not held object authority.

The main legacy risk found is diagnostic and configuration ambiguity:

- `HandGrab.cpp:3986` logs `constraintTau`, `constraintDamping`, and `constraintForce` during a `"native dynamic grab"` log line. Those values are not the single-hand native spring drive surface, so this can send troubleshooting toward shared-constraint knobs while the object is actually on the native mouse spring.
- `data/config/ROCK.ini:427` names the section `Grab Constraint Feel`, but the same section also contains native mouse-spring single-hand settings. The comments at `data/config/ROCK.ini:457` correctly say native mouse spring, but the section heading can still mislead.
- The production INI has the same mixed native/constraint tuning area around its active grab settings.
- `GrabConstraint.cpp:287` labels transform-B pivot as `[surface]` even though the current shared-constraint path uses the active body-local pivot. This is stale naming in constraint diagnostics.
- `GrabContact.h:30` uses reason `"legacyContactSources"` and `GrabContactQualityMode::LegacyPermissive` exposes `"legacyPermissive"` names. These are compatibility policy names, not evidence of old grab authority, but they pollute logs during debugging.
- `GrabConstraint.cpp:530` has a `legacyState` variable in inertia restore. That appears to be a migration fallback for older saved state layout, not active legacy drive behavior.

## Most Likely Polish Deltas

1. Visual hand settling is the clearest HIGGS quality difference.

   HIGGS updates the rendered hand from the live held object every `HeldBody` frame and only releases when average deviation becomes unsafe. ROCK already has the same held-object-relative math, but gates it to `TouchHeld` or a bounded acquisition envelope. If users feel the grab is physically working but not as polished as HIGGS, the visual/arm settle gate and lerp timing should be investigated before changing object motors.

2. Single-hand native telemetry is mixed with shared constraint telemetry.

   The current native grab log prints native response scales and also shared constraint values. For tuning quality, the log should be drive-mode specific: native target point, live BODY grip error, adaptive lead, collision lead scale, native response scales, native action debug state. Shared constraints should log tau/force/recovery separately.

3. Collision feel should be evaluated separately for native and shared-constraint drives.

   HIGGS directly changes constraint tau when colliding. ROCK's native path cannot change native tau the same way; it shapes the native target lead and uses contact freshness. This is valid, but debugging must make that difference obvious.

4. Contact freshness and player controller intervention are critical polish layers.

   HIGGS sets contact callback delay to zero on connected bodies. ROCK has a different FO4VR-native contact pipeline and character-controller filtering. If held object collision still feels inconsistent, investigate whether held-body contacts are being refreshed for compound body sets before changing motor strength.

5. Actor/ragdoll and weapon polish should stay out of the loose-object grab bucket.

   HIGGS actor/ragdoll handling and equipped two-hand weapon support are useful references, but ROCK's actor and weapon systems are separate architecture. Mixing those into loose-object grab tuning will make troubleshooting less clear.

## Recommended Next Changes

These are documentation/diagnostic and behavior-polish candidates, not changes made in this pass:

1. Split grab telemetry by drive mode.
   - Native log: `driveMode=nativeMouseSpring`, native response scales, native action pointer, target point, BODY grip error, adaptive lead, collision lead scale.
   - Shared constraint log: `driveMode=sharedConstraint`, tau/damping/force/recovery/fade/collision tau.

2. Rename stale diagnostic labels.
   - `GrabConstraint.cpp` log label `[surface]` should become `[body-local-pivot]` or `[active-pivot-b]`.
   - `legacyContactSources` can become `collisionAndPatchSources` or `compatContactSources`.
   - `legacyPermissive` can become `permissiveFallback` if config compatibility allows it.

3. Clarify INI headings and comments in both packaged and production INIs.
   - Use separate headings for shared constraint motor feel and native mouse-spring single-hand feel.
   - Keep production INI edits in place only.

4. Investigate visual hand polish using the existing held-object-relative path.
   - Do not make visual transform object motor authority.
   - Test earlier/continuous visual hand settle after grab creation using bounded deviation and lerp safeguards.
   - Keep release safety based on visual deviation.

5. Add or extend source tests to enforce the cleanup.
   - Native dynamic grab logs must not present shared constraint tau as the active native drive.
   - Constraint pivot diagnostics must not use stale `surface` terminology.
   - INI comments should expose native and shared constraint tuning as separate surfaces.

6. In-game validation checklist after any behavior change.
   - Small clutter pickup near shelves.
   - Heavy/compound object pickup.
   - Object colliding with wall while held.
   - Room-scale movement and snap turn while holding.
   - Throw release with wrist rotation.
   - Peer hand joining and leaving a held object.
   - Far pull catch into held state.

## Open Verification Gates

- Any change to native mouse-spring action internals, native function pointers, body flags, body-array layout, contact callback delay equivalents, or FO4VR binary offsets requires Ghidra approval first.
- Any actor/ragdoll grab expansion needs an explicit actor/SCISSORS architecture decision, not a loose-object grab tweak.
- Any weapon grab change must be checked against the weapon domain, because HIGGS equipped two-hand support is not the same thing as ROCK loose-object dynamic grab.

## Test Snapshot

Focused source tests run on 2026-05-11:

- `tests/HandGrabNativeBoundarySourceTests.ps1` - passed.
- `tests/SharedHeldObjectGrabSourceTests.ps1` - passed.
- `tests/ContactEvidenceOwnershipSourceTests.ps1` - passed.
