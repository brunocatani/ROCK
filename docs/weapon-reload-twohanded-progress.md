# ROCK Weapon Reload And Two-Handed Grip Progress

Last updated: 2026-04-27 08:35 -03:00

## Why This File Exists

The reload and two-handed weapon work touches weapon mesh collision, generated Havok bodies, contact routing, FRIK pose control, ammo state, input blocking, debug overlay, and future native FO4VR calls. Treating any one of those as an isolated patch is how older convention bugs accumulated. This file tracks the implementation as one coherent system, records which references have actually been checked, and marks where Ghidra verification is required before code can safely depend on native offsets or runtime behavior.

## User Goal

Build a ROCK-native physical weapon interaction system:

- Right hand always owns the equipped weapon.
- Left hand always handles support grip and reload interactions.
- Equipped weapon collision becomes a semantic weapon assembly, not anonymous hulls.
- Left-hand weapon contact routes to support grip, ammo removal/insertion, socket guidance, or action manipulation.
- Reload behavior eventually covers box magazines, break-action shells, cylinders, laser/fusion cells, lever/direct insert, and bolt/slide/latch/pump actions.
- Two-handed grip is part of v1 and uses semantic grip profiles plus FRIK finger poses.
- Two-handed grip must replace the currently suppressed FRIK offhand weapon grip behavior in function, not merely in visuals:
  - primary/right hand remains the fixed weapon owner and fixed grip anchor;
  - left hand grabs a real mesh-derived semantic support point;
  - weapon aiming is helped by solving from both the fixed primary grip and the left support grip;
  - hands/fingers bend to the actual mesh contact;
  - the same solved weapon transform must drive both the visual weapon node and ROCK's generated collision bodies so visual aim and collision/grab stay matched.
- Virtual Reloads is reference material only; ROCK owns its own data/config/API/runtime state.

## Hard Constraints

- Use HIGGS source as reference for logic and approach before implementation.
- Cross-check FO4VR native assumptions with Ghidra before using offsets, native wrappers, ammo APIs, sound loading, NIF clone behavior, projectile/reload internals, or weapon form layouts.
- Ask before every Ghidra operation.
- Do not use `.md` files as source of truth for implementation facts unless they are our own progress/design notes.
- No power armor work in this slice. Power armor needs a separate compatibility pass.
- No left-handed-mode mirroring for weapon roles in v1: right hand is weapon owner; left hand is interaction hand.
- No fallback weapon collision creator. Generated mesh collision is the active weapon collision path.

## Current ROCK State

Already implemented before this tracker:

- Generated weapon mesh collision from visible weapon TriShapes.
- Weapon fallback/native body mirroring removed.
- Weapon body cap is now `MAX_WEAPON_COLLISION_BODIES = 64`, with a unit test asserting the cap.
- Weapon mesh hulls are transformed as one package using the fixed column-basis convention.
- Weapon projectile/spell layer filtering exists.
- Hand collider conventions are fixed and verified by user runtime testing.
- Grab hand visual lerp and normal-hand mesh finger pose support are implemented.
- FRIK API v5 has custom priority finger/joint pose entry points.
- TwoHandedGrip no longer uses generic `_offhandTouchingWeapon`. Left-hand weapon contact now carries body ID, semantic part kind, reload role, support role, socket role, action role, and fallback grip pose.

## HIGGS References Checked

Source files inspected:

- `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\hand.h`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp`
- `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\higgsinterface001.h`

Important HIGGS patterns:

- HIGGS treats weapon collision as owned, keyframed body state on the hand.
- Two-handing is a structured state (`SelectedTwoHand`, `HeldTwoHanded`), not a stateless touch flag.
- It stores transforms needed to restore weapon offset/collision offset/wand nodes when two-handing ends.
- Two-handed motion computes desired weapon transform from the owning hand, then adjusts it using the offhand palm position and palm direction.
- It applies smoothing with `AdvanceTransform`.
- It updates weapon node, weapon collision offset node, and related weapon offset nodes together.
- It animates fingers from weapon mesh geometry when grabbing/two-handing.
- HIGGS API exposes weapon body, two-handing callbacks, collision filter callbacks, pre-physics callbacks, and finger values.

ROCK cannot copy this directly because FO4VR uses hknp/Havok 2014 conventions and different node/runtime layouts, but the state ownership and data-flow pattern are the right reference.

## FRIK Offhand Grip References Checked

Source files inspected:

- `E:\fo4dev\PROJECT_ROCK_V2\Fallout-4-VR-Body(FRIK)\src\weapon-position\WeaponPositionAdjuster.cpp`
- `E:\fo4dev\PROJECT_ROCK_V2\Fallout-4-VR-Body(FRIK)\src\weapon-position\WeaponPositionAdjuster.h`
- `E:\fo4dev\PROJECT_ROCK_V2\Fallout-4-VR-Body(FRIK)\src\skeleton\HandPose.cpp`

Important FRIK behavior to reproduce in ROCK terms:

- FRIK offhand grip is not just a hand pose. It rewrites the weapon transform every frame.
- It first applies the stored primary weapon offset, then checks whether offhand grip is active.
- Grip activation in FRIK is based on offhand-to-primary vector being close to the weapon barrel axis after applying the offhand offset matrix.
- While gripping, FRIK computes a world vector from primary hand to offhand, converts it into weapon-local space, applies the configured offhand rotation offset, and rotates the weapon so canonical weapon forward aligns with that adjusted vector.
- FRIK recomputes weapon local translation around a local primary grip pivot so the stock/grip does not fly away from the primary hand when the weapon rotates.
- FRIK rotates the primary hand so it remains coherent with the adjusted weapon orientation.
- FRIK uses `setOffhandGripHandPose()` to apply the offhand finger pose. ROCK should replace that fixed pose with semantic mesh-derived finger/joint poses.
- FRIK updates weapon transforms and then runs downstream muzzle/scope adjustments. ROCK must at least ensure generated weapon collision bodies are updated from the same solved transform used by the visible weapon node.

ROCK design consequence:

- TwoHandedGrip must become a weapon transform solver, not a visual-only hand snapper.
- The solver's output must be fed into `WeaponCollision` as the authoritative weapon assembly transform for that frame.
- `WeaponCollision` must be able to update generated bodies from a supplied solved weapon transform rather than always reading the unmodified weapon node world transform.
- Semantic grip profiles supply the left-hand support point and pose; the right-hand fixed grip remains the invariant anchor.

## Ghidra Verification Completed

Approved by user on 2026-04-27.

Binary sanity check:

- Loaded image base is `0x140000000` with `.text` ending near `0x142c40bff`, matching FO4VR executable layout.
- Imports include SteamVR/OpenVR symbols, matching `Fallout4VR.exe.unpacked.exe` expectations.

Verified native contact/body assumptions:

- `0x1403B9E50` is an `hkSignal2<hknpEventHandlerInput const&, hknpEvent const&>` member-slot subscription helper. It copies callback function/context into a 0x28-byte slot.
- FO4VR world setup at `0x1403AB950` calls the event-signal getter with event type `3`, then subscribes `FOCollisionListener` through `0x1403B9E50`. This matches ROCK subscribing to `RE::hknpEventType::kContact`.
- FO4VR collision listener callback at `0x14061B5C0` independently reads contact event body IDs from `event + 0x08` and `event + 0x0C`.
- The same callback indexes the hknp body array through `world + 0x20` using body stride `0x90`.
- The same callback reads `hknpBody::collisionFilterInfo` at body offset `+0x44`.
- `0x14153AF00` writes body filter info at body offset `+0x44` and uses the same body array stride `0x90`; `0x141DF5B80` wraps this function.
- `0x141E07990` returns the body collision object back-pointer from body offset `+0x88`.
- `0x141548170` returns an event signal through the world/event manager path and is used by FO4VR setup for contact event type `3`.

Conclusion:

- ROCK can safely replace `_offhandTouchingWeapon` with structured contact data based on body IDs from contact event `+0x08/+0x0C`.
- No new native contact hook is needed for the semantic weapon contact router.
- Body ID to generated weapon metadata lookup is the right boundary.

## Immediate Architecture Conclusion

The first production slice should build semantic weapon assembly scaffolding on top of the working generated mesh collision path:

- Classify every generated hull into a semantic `WeaponPartKind`.
- Store metadata beside each generated body.
- Publish body ID to metadata lookup.
- Replace `_offhandTouchingWeapon` boolean with structured left-hand weapon contact data.
- Replace the current TwoHandedGrip behavior with an FRIK-style two-anchor weapon transform solver:
  - right fixed grip anchor is preserved;
  - left semantic support grip/contact point drives aim assistance;
  - weapon visual transform and generated collision bodies consume the same solved transform;
  - left fingers use actual mesh contact;
  - primary hand alignment is adjusted only through the same solved weapon transform relationship, not through a disconnected visual snap.
- Keep reload behavior as explicit state scaffolding first, with no unverified native ammo mutation or spawned ammo objects until Ghidra verifies the required FO4VR calls/layouts.

This gives the router and debug/API a stable spine without pretending reload internals are verified.

## Planned File Additions

- `src/physics-interaction/WeaponSemanticTypes.h`
  - Enums: `WeaponPartKind`, `WeaponReloadRole`, `WeaponSupportGripRole`, `WeaponSocketRole`, `WeaponActionRole`, `WeaponInteractionKind`, `WeaponReloadState`, `WeaponReloadType`, `WeaponAmmoKind`, `WeaponGripKind`, `WeaponFingerPoseId`.
  - Structs: `WeaponPartMetadata`, `WeaponGripProfile`, `WeaponInteractionContact`, `WeaponRouterDecision`, `WeaponReloadRuntimeState`.

- `src/physics-interaction/WeaponPartClassifier.h`
  - Name/path based classifier.
  - No FO4VR native assumptions.
  - Unit tested heavily.

- `src/physics-interaction/WeaponHullBudget.h`
  - Category-budgeted selection policy that preserves gameplay parts before cosmetics.
  - Replaces generic balanced hull selection for weapon semantic use.

- `src/physics-interaction/WeaponGripProfiles.h`
  - Auto-generated fallback support grip profiles from detected semantic parts.
  - Finger pose defaults for barrel wrap, handguard clamp, vertical foregrip, angled foregrip, pump grip, magwell hold, receiver support.

- `src/physics-interaction/WeaponTwoHandedSolver.h`
  - Pure math solver for FRIK-style two-handed weapon manipulation.
  - Inputs: current weapon transform, fixed primary grip anchor, active support grip profile/contact, primary hand transform, left hand transform.
  - Output: solved weapon transform plus hand alignment targets.
  - Unit tested without Havok.

- `src/physics-interaction/WeaponInteractionRouter.h`
  - Pure decision logic: touched part + reload state + grip input -> support grip/reload/socket/action/passive.
  - Testable without Havok.

- `src/physics-interaction/WeaponReloadStateMachine.h`
  - Explicit reload state enum and legal transition helper.
  - Initial slice only scaffolds transitions and logging. Native ammo mutation waits for Ghidra verification.

## Planned File Modifications

- `src/physics-interaction/WeaponCollisionLimits.h`
  - Raise `MAX_WEAPON_COLLISION_BODIES` from 32 to 64.

- `src/physics-interaction/WeaponCollision.h/.cpp`
  - Add metadata to `GeneratedHullSource` and `WeaponBodyInstance`.
  - Store source node/path/name, local center, bounds, part kind, roles, enabled/hidden/detached state.
  - Publish metadata lookup by body ID.
  - Accept an optional externally solved weapon transform from TwoHandedGrip for the current frame so generated collision bodies follow the exact same transform as the visible weapon assembly.
  - Log category counts and selected semantic parts.

- `src/physics-interaction/PhysicsInteraction.h/.cpp`
  - Replace `_offhandTouchingWeapon` with structured left-hand weapon contact snapshot.
  - Resolve weapon contacts by body ID through `WeaponCollision`.
  - Feed `WeaponInteractionRouter` into `TwoHandedGrip` and reload scaffolding.

- `src/physics-interaction/TwoHandedGrip.h/.cpp`
  - Replace one-pose barrel behavior with semantic support grip profiles.
  - Preserve right-hand ownership.
  - Use left-hand contacts and profile metadata, not closest triangle alone.
  - Drive the weapon assembly with an FRIK-style two-anchor solver.
  - Apply mesh-derived FRIK v5 finger/joint poses for the active semantic grip.
  - Never leave weapon visuals and ROCK generated collision bodies on separate transforms.

- `src/api/ROCKApi.h/.cpp`
  - Bump API version after the internal state exists.
  - Add query functions for last touched weapon part, active support grip profile, reload state/type, held ammo kind, socket alignment state.

- `src/physics-interaction/DebugBodyOverlay.h/.cpp`
  - Add part-kind coloring for weapon bodies.

- `src/RockConfig.h/.cpp`
  - Add reload/support grip config schema only for features implemented in this slice.
  - Avoid adding fake knobs for unimplemented native behavior.

- `data/config/ROCK.ini`
  - Mirror config defaults that are implemented and hot-reloadable.

- `tests/TransformConventionTests.cpp`
  - Add unit tests for cap 64, classifier, semantic budget, body metadata lookup, router decisions, grip profile selection, two-handed solver pivot preservation, visual/collision transform handoff, finger pose lookup, reload transition legality.

## First Implementation Slice

Status: implemented and locally verified on 2026-04-27.

- [x] Add pure semantic types.
- [x] Add classifier tests and implementation.
- [x] Add 64 body cap test and implementation.
- [ ] Add separate semantic hull budget helper. Current generated hull selector still uses the existing balanced selector, but its categories now come from the semantic classifier.
- [x] Attach semantic metadata to generated weapon hulls.
- [x] Add body ID -> metadata lookup for contact-thread use.
- [x] Convert contact storage from boolean to `WeaponInteractionContact`.
- [x] Add FRIK-style two-handed solver tests: fixed right grip stays fixed, left support point pulls aim vector, primary grip pivot is preserved.
- [x] Route two-handed grip through semantic support profiles and the two-anchor solver.
- [x] Feed solved weapon transform into generated weapon collision updates for the same frame.
- [x] Add debug/log output for semantic touched part and generated body part roles.
- [x] Build and run tests.

Implemented files:

- `src/physics-interaction/WeaponSemanticTypes.h`
- `src/physics-interaction/WeaponPartClassifier.h`
- `src/physics-interaction/WeaponInteractionRouter.h`
- `src/physics-interaction/WeaponTwoHandedSolver.h`

Implemented runtime behavior:

- Right hand is the fixed equipped-weapon owner for weapon collision suppression.
- Left hand is the weapon interaction hand for support grip contact.
- Contact events read the left-hand touched weapon body ID and resolve it through `WeaponCollision::tryGetWeaponContactAtomic`.
- Two-handed grip activates only through router decisions where the touched part is a support-grip part.
- The active support pose is selected from semantic part metadata, then overridden by mesh-derived finger values when the mesh solver succeeds.
- The two-handed solver preserves the right-hand primary grip anchor and pulls the weapon assembly toward the left-hand support point.
- The solved visual weapon transform is immediately passed to `WeaponCollision::updateBodiesFromWeaponRootTransform`, so generated collision bodies use the same weapon-root transform as the visual weapon for that frame.

Verification run:

- `cmake --build build --config Release --target ROCKTransformConventionTests`
- `build\Release\ROCKTransformConventionTests.exe`
- `cmake --build build --config Release`

## Two-Handed Equipped Weapon Authority Follow-Up

Status: implemented and unit-tested on 2026-04-27. Full in-game validation is still required.

Why this slice is being implemented this way:

FRIK's suppressed two-handed firearm grip already solved the part ROCK should not replace: aiming from the fixed right-hand weapon grip toward the left-hand support point while preserving the primary pivot. HIGGS supplies the better state model: select the support point from real mesh/contact data, store weapon-local relationships, compute hand/finger pose from mesh, then drive the weapon and hands as a keyframed authority state. This slice keeps FRIK's aiming convention but moves ROCK's active two-handed weapon path from point-only hand snapping to full weapon-local hand frames, so visual hands, generated weapon collision, and semantic support anchors can agree.

Controlling behavior:

- Right hand remains the equipped weapon owner.
- Left hand remains the support/reload interaction hand.
- Active equipped-weapon two-handed grip is separate from dynamic object two-handed grab.
- Layer 43 vs 44 contact/probe is needed before activation so the left hand can find weapon parts.
- During active support grip, hand-vs-weapon collision is suppressed elsewhere to avoid solver fighting; both bodies are keyframed, so penetration must be solved by transform authority and hand posing.
- One-handed firearms use the actual touched semantic mesh/body point.
- Fixed two-handed categories should prefer semantic grip profiles first, then fall back to touched mesh points.

Implemented in this slice:

- Added `solveTwoHandedWeaponTransformFrikPivot()` as the named FRIK-style aiming contract.
- Added solver `rotationDelta` output for right-wrist/primary-hand twist consumers.
- Added focused unit coverage for the FRIK-style contract:
  - fixed primary grip stays on the right-hand target;
  - support point pulls the weapon as one package;
  - rotation delta maps weapon forward toward the two-hand vector.
- Updated `TwoHandedGrip` to store right-hand and left-hand transforms in weapon-local space at activation.
- Updated active grip runtime to apply full hand world targets from the solved weapon transform through FRIK API v6's skeleton-only arm-chain service instead of translating wrist nodes directly.
- Added locked support-target math so support controller motion along the captured primary/support axis preserves the grabbed mesh point while lateral motion still pivots the weapon.
- Kept the generated weapon collision handoff through `WeaponCollision::updateBodiesFromWeaponRootTransform`, so colliders continue to follow the solved weapon root as one package.

Review fixes applied after the first implementation:

- Mesh-selected support points now adjust the stored support-hand frame so the hand grab pivot lands on the exact touched weapon mesh point.
- Equipped-weapon support grip can no longer start or continue while the left hand owns a normal dynamic-object grab.
- Normal left-hand grab processing is skipped while ROCK equipped-weapon support grip is active, preserving the separation between weapon support and dynamic object manipulation.
- Releasing equipped-weapon support grip publishes the restored weapon-root transform for the same frame, so generated weapon collision bodies do not lag behind the visual weapon restore.
- Right-hand normal object grabbing is suppressed while the right-hand weapon is drawn/equipped; if a normal object is already held when weapon ownership becomes active, ROCK releases that object instead of allowing dual right-hand ownership.

Required in-game validation:

- Visible gun must rotate around the right-hand grip when left support grip is active.
- Generated weapon colliders must keep following the visible gun as one coherent package.
- Left hand should no longer visually pass through or slide along the selected grip point during support grip; FRIK now applies ROCK's requested hand target through the arm chain rather than accepting a direct hand-node overwrite.
- Release must restore FRIK offhand grip behavior without leaving ROCK poses active.
- Projectile/muzzle origin should remain coherent after two-handed manipulation; if it does not, the native first-person weapon offset/projectile node update path needs the next Ghidra-verified implementation slice.

## Ghidra Verification Needed Before Native Runtime Work

Ask user approval before each Ghidra operation.

## Vanilla-Stage Reload Integration Slice

Status: in progress on 2026-04-27.

Why this slice is being implemented this way:

ROCK needs to use the vanilla FO4VR reload path as the compatibility spine instead of forcing every weapon to be manually patched. Virtual Reloads shows the right methodology at a system level: observe vanilla weapon/ammo state, manipulate ammo/node ownership at the right lifecycle point, and keep modded weapon compatibility. ROCK is not copying Virtual Reloads offsets or its controller-driven flow. The safe implementation boundary is a verified native event observer feeding a ROCK-owned reload coordinator, with physical ammo/node mutation kept behind later Ghidra-verified wrappers.

Plan implementation state:

- [x] Compare Virtual Reloads methodology against ROCK needs.
- [x] Ghidra-check current FO4VR reload/ammo leads before using them.
- [x] Add pure observer/coordinator tests for vanilla reload event, ammo event, completion, and router interaction.
- [x] Add `WeaponReloadStageObserver.h` pure state logic.
- [x] Add native `WeaponReloadEventBridge` sink for verified FO4VR reload/ammo event sources.
- [x] Wire observer/coordinator into `PhysicsInteraction::update`.
- [x] Pass active reload runtime state into `TwoHandedGrip`/router so reload-owned parts are not treated as support grips while reload is active.
- [x] Add config keys:
  - `bReloadUseVanillaStageObserver=true`
  - `bReloadRequirePhysicalCompletion=true`
  - `bReloadAllowStageFallbacks=true`
  - `bReloadDebugStageLogging=false`
- [x] Add API queries for observed vanilla reload stage, reload source, active ROCK reload state, and last touched weapon part.
- [x] Update generated and active INI defaults.
- [x] Build and run focused tests.
- [x] Deploy DLL only after successful verification.
- [x] Fix review issues found after first implementation:
  - same-frame reload+ammo events no longer drop `AmmoCommitted`;
  - `bReloadRequirePhysicalCompletion=true` keeps the coordinator in the active reload path when vanilla completion arrives before physical insertion;
  - hot-disabling the observer resets bridge event sequence state;
  - over-budget weapon hulls use semantic-priority selection so gameplay parts are preserved before cosmetics;
  - API enum values are fully asserted against internal enum values.

Verified FO4VR native findings from Ghidra:

- `0x140ddf690` reads the actor equipped weapon state at `actor + 0x300` and returns current ammo count through the equipped item/ammo path.
- `0x140ddf790` writes current ammo count through the same equipped item path.
- `0x140ddf6c0` is current clip/clip percentage related, reached through equipped weapon data and ammo capacity.
- The older CommonFramework reload hook lead at `0x140f3027c` is invalid for the loaded FO4VR binary: no function starts there and no xrefs support using it.
- `PlayerWeaponReloadEvent` and `PlayerAmmoCountEvent` exist in FO4VR and are the safer first reload observer path.
- Verified register helpers:
  - `0x140a10cd0` registers a `PlayerWeaponReloadEvent` sink.
  - `0x140a10be0` registers a `PlayerAmmoCountEvent` sink.
- Verified global event source pointers:
  - `0x145ac1378` (`module offset 0x5ac1378`) points to `BSTGlobalEvent::EventSource<PlayerWeaponReloadEvent>*`.
  - `0x145ac1368` (`module offset 0x5ac1368`) points to `BSTGlobalEvent::EventSource<PlayerAmmoCountEvent>*`.

Current runtime limitation:

The observer/coordinator core is implemented and unit-tested, but physical magazine/ammo detach/clone/socket insertion is not live yet. Completion gating now preserves ROCK's active reload routing state when vanilla completes before physical insertion, but it still does not block or rewrite vanilla ammo by itself. The actual node clone/hide/restore and ammo mutation path must wait for the next Ghidra-verified wrapper slice.

Implemented files in this slice:

- `src/physics-interaction/WeaponReloadStageObserver.h`
- `src/physics-interaction/WeaponReloadEventBridge.h`
- `src/physics-interaction/WeaponReloadEventBridge.cpp`
- `src/physics-interaction/WeaponSemanticHullBudget.h`

Modified files in this slice:

- `src/physics-interaction/PhysicsInteraction.h/.cpp`
- `src/physics-interaction/TwoHandedGrip.h/.cpp`
- `src/RockConfig.h/.cpp`
- `src/api/ROCKApi.h/.cpp`
- `data/config/ROCK.ini`
- `tests/TransformConventionTests.cpp`

Verification run:

- `cmake --build build --config Release --target ROCKTransformConventionTests`
- `build\Release\ROCKTransformConventionTests.exe`
- `cmake --build build --config Release`

Required later checks:

- Equipped weapon form/ammo count mutation path for early-zero reload behavior.
- Vanilla reload/fire blocking path.
- FO4VR ammo form and current ammo APIs.
- NIF/3D clone/spawn path for ammo visuals/collision.
- Sound descriptor playback wrappers.
- Keyword lookup/plugin data access patterns.
- Contact listener/body hit layout if reload/action manipulation needs actual collision callbacks beyond current contact event data.
- Any Virtual Reloads-style hook or projectile/reload hook.

Not required for the first semantic classifier/router slice:

- Name-based mesh part classification.
- Body cap constant.
- Pure router decision tests.
- FRIK pose profile selection through existing v5 API.

## Open Questions For User

These are implementation questions, not blockers for the first semantic scaffolding slice:

- Should v1 treat all firearms as right-hand weapons even when FO4VR left-handed mode is enabled? Current requirement says yes.
- Should reload button be the same controller button currently used by vanilla reload, or a new ROCK config key?
- Should support grip be allowed during `ReloadRequested`, or should reload state always take priority over support grip on reload-capable parts?
- Should magazines/cells be physically removable in the first reload runtime slice, or should v1 first implement socket/action detection and debug visualization before ammo object spawning?

## Current Blocker

Before implementation that depends on FO4VR native reload/ammo/clone/sound behavior, Ghidra approval is required. The first semantic assembly/router slice can proceed without Ghidra because it is pure ROCK data modeling on already-created ROCK bodies and HIGGS-inspired state structure.
