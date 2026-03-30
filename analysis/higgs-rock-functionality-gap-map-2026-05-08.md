# HIGGS to ROCK Functionality Gap Map

Date: 2026-05-08

Canonical note path: `ROCK\analysis\higgs-rock-functionality-gap-map-2026-05-08.md`

## Decision Note

This map exists because the requested comparison is specifically HIGGS functionality versus current ROCK coverage. HIGGS is treated here as an inventory source, not as the design authority for future ROCK behavior. The comparison separates true missing functionality from FO4VR-native replacements, because forcing ROCK back toward one-to-one HIGGS behavior would erase systems that already fit Fallout 4 VR better, especially the bone-derived hand bodies, generated weapon collision, provider APIs, and weapon/body interaction logic.

No web or Ghidra was used for this pass. This is a source and local-documentation comparison only, so it does not make new binary-layout, address, or offset claims.

## Local Sources Read

- HIGGS source and docs:
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\README.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\analysis\HIGGS_COMPLETE_ANALYSIS.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\tasks\MASTER_PLAN.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\tasks\phases\phase_1_hand_collision.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\tasks\phases\phase_2_selection_highlight.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\tasks\phases\phase_3_grab_release.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\tasks\phases\phase_4_gravity_gloves.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\tasks\phases\phase_5_weapon_collision.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\tasks\phases\phase_6_polish.md`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\hand.h`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\config.h`
  - `E:\fo4dev\skirymvr_mods\source_codes\higgs\include\higgsinterface001.h`
- ROCK source and docs:
  - `ROCK\README.md`
  - `ROCK\src\physics-interaction\core\README.md`
  - `ROCK\src\physics-interaction\grab\README.md`
  - `ROCK\src\physics-interaction\hand\HandLifecycle.h`
  - `ROCK\src\physics-interaction\hand\HandInteractionStateMachine.h`
  - `ROCK\src\physics-interaction\hand\HandInteractionStateMachine.cpp`
  - `ROCK\src\physics-interaction\hand\Hand.h`
  - `ROCK\src\physics-interaction\hand\Hand.cpp`
  - `ROCK\src\physics-interaction\hand\HandGrab.cpp`
  - `ROCK\src\physics-interaction\config\RockConfig.h`
  - `ROCK\src\physics-interaction\config\RockConfig.cpp`
  - `ROCK\src\api\ROCKApi.h`
  - `ROCK\src\api\ROCKApi.cpp`
  - `ROCK\src\api\ROCKProviderApi.h`
  - `ROCK\src\api\ROCKProviderApi.cpp`
  - Targeted searches through `ROCK\src` and `ROCK\tests`

## Status Legend

- Missing: no runtime backing found in ROCK.
- Scaffolded: enum, event, config, or state shape exists, but no full gameplay/runtime path was found.
- Partial: ROCK has an equivalent class of behavior, but not the full HIGGS feature.
- Superseded: ROCK has a deliberate FO4VR-native replacement that should not be counted as missing without a separate design decision.

## Current ROCK Coverage

ROCK already covers the central physical hand interaction stack:

- Live hand collision bodies on ROCK hand layer 43, derived from FRIK/player skeleton evidence instead of HIGGS-style fixed hand boxes.
- Near and far object selection, including actor equipment visual-node-first selection and VATS-style highlighting.
- Single-hand dynamic loose-object grab and release, with lifecycle save/restore, inertia handling, collision suppression, release velocity, player-space compensation, and telemetry.
- Far pull and catch flow, including actor-equipment drop handoff into pull/catch.
- FRIK-facing pose publication and mesh/contact evidence used for hand interaction.
- Event haptics for lock/unlock, pull start/catch, grab commit, held impact, and soft-contact world interaction.
- Generated weapon collision, support-hand/two-hand grip systems, reload/weapon evidence integration, and provider APIs that exceed the HIGGS weapon baseline.
- State-machine scaffolding for HIGGS-like grab phases plus ROCK-specific stash and consume candidates.

## Missing Or Scaffolded HIGGS Functionality

| HIGGS functionality | ROCK status | Current evidence | Equivalent ROCK work would need |
| --- | --- | --- | --- |
| Shoulder stash to inventory | Scaffolded | `HandState::StashCandidate` and grab event reservations exist; no inventory commit path was found. | Shoulder-zone detection, held-object eligibility, inventory transfer/drop cleanup, haptics, events, config gates. |
| Mouth consume/use item | Scaffolded | `HandState::ConsumeCandidate` and `Consumed` event reservation exist; no use/consume gameplay path was found. | Mouth-zone detection, consumable eligibility, item use/activation executor, failure handling, haptics, events. |
| HIGGS rollover UI text/menu | Missing | ROCK has VATS/highlight behavior, but no HIGGS-style activate rollover placement/runtime was found. | FO4VR UI policy for object prompts, stable ownership with selection, and cleanup on deselect/grab. |
| HIGGS selection beam | Missing | No selection beam runtime was found; ROCK uses highlight/contact visuals instead. | Beam renderer or FO4VR-native equivalent tied to selection and pull lock states. |
| Papyrus API and events | Missing | ROCK exposes native API and F4SE messages, but no Papyrus registration equivalent to HIGGS was found. | Stable Papyrus surface, event dispatch policy, VM-safe data conversion, versioning. |
| Direct callback API parity | Partial | ROCK dispatches `kPhysMsg_OnGrabEvent` and has provider callbacks, but not HIGGS-style callback registration for every event type. | ABI decision for callbacks versus messages, callback lifetime rules, thread/frame guarantees. |
| Programmatic external object grab | Scaffolded | `GrabExternal` exists in the hand state machine; no public `GrabObject` equivalent was found. | Public request API, object validation, hand conflict resolution, transform seed, failure result reporting. |
| Get/set grab transform API | Missing | No HIGGS-equivalent `GetGrabTransform` or `SetGrabTransform` API was found. | Stable transform model, per-hand/per-object storage, caller ownership rules. |
| Grabbed node name API | Missing | No HIGGS-equivalent grabbed node-name API was found. | Captured selected node/source evidence exposed through API. |
| Finger value API | Missing or superseded internally | ROCK publishes pose/local transform evidence internally, but no HIGGS-style public finger-values API was found. | Public hand pose contract, finger indexing, update timing, invalid-state behavior. |
| Hand, weapon, and grabbed rigid-body handle API | Missing | No public equivalent to HIGGS hand/weapon/grabbed rigid-body handle access was found. | ABI-safe handle exposure and ownership/lifetime constraints. |
| Public weapon collision enable/disable/force API | Missing | Weapon collision exists internally; no HIGGS-style public control API was found. | Runtime control surface, per-hand/per-weapon authority, event and config interaction. |
| Collision-filter comparison callback | Missing | No HIGGS-style user collision-filter callback was found. | Hook point in collision filtering, callback safety, deterministic fallback behavior. |
| Public layer bitfield mutation API | Missing | ROCK has fixed layer policy and suppression logic; no public bitfield get/set equivalent was found. | Deliberate layer ABI, validation, compatibility with ROCK hand layer 43 and weapon bodies. |
| Pre-physics-step public callback | Missing | Provider/frame callbacks exist, but no HIGGS-equivalent pre-physics-step callback was found. | Frame ordering guarantee and safe callback context. |
| Dropped-object tracking after release | Partial | ROCK has release and suppression cleanup; no HIGGS-style dropped-object listener lifecycle was found. | Dropped-body registry, regrab cooldown, island deactivation cleanup, listener removal. |
| NPC sound from thrown/dropped objects | Missing | No HIGGS-style sound-level dispatch from dropped object impact was found. | Impact classification, actor perception event bridge, config gates. |
| Destructible damage from thrown/dropped objects | Missing | No HIGGS-style destructible impact damage path was found. | Contact impulse policy, damage routing, ownership rules, exclusions. |
| Physics impact sounds | Missing | No HIGGS `PlayPhysicsSound` equivalent was found. | Contact event classification and FO4VR sound dispatch. |
| Mass-based held-object movement speed penalty | Missing | No HIGGS-style player speed modification from held mass was found. | Total held mass accounting, actor value or movement modifier policy, restore guarantees. |
| Mass-based jump-height penalty | Missing | No HIGGS-style jump-height modification from held mass was found. | Same mass accounting plus safe jump parameter modification/restore. |
| Container carry propagation | Missing | No equivalent to HIGGS contained-rigid-body compensation was found. | Detect carried bodies inside/on held containers and apply player-space compensation coherently. |
| Nearby grab damping | Implemented | `NearbyGrabDamping` now leases copied FO4VR hknp motion-property entries and restores the original ID after all overlapping leases release. Config defaults it enabled after binary verification. | In-game tuning for radius/duration/damping feel. |
| Full player-space transform warp | Implemented for held bodies | Velocity compensation and runtime transform warp exist through the central held-player-space writer. On 2026-05-08 the default and active INI were changed to enable `bGrabPlayerSpaceTransformWarpEnabled`. | Still needs contained-body propagation so bodies resting in/on held containers receive the same room-space warp. |
| Whole actor/ragdoll body grab | Missing in ROCK | ROCK has external body/provider concepts and actor-equipment handling; object detection blocks whole dead actor body grabs. | Coordination with SCISSORS/external body provider, body ownership, constraints, release safety. |
| Loot-from-other-hand runtime | Scaffolded | `LootOtherHand` state exists; no full HIGGS-style loot-from-other-hand runtime was found. | Cross-hand state ownership, inventory/actor transfer, input semantics, haptics/events. |
| Two-handed start/stop public events | Scaffolded | `TwoHandStarted`/`TwoHandStopped` are reserved in API assertions; support-grip systems exist, but dispatch was not found. | Event bridge from support grip state changes into public API/messages. |
| Continuous HIGGS haptic manager behavior | Partial | ROCK has event haptics and soft-contact haptics, but not the HIGGS dedicated haptic manager/thread with all selection/stash/consume patterns. | Decide which haptic patterns belong in FO4VR, then centralize runtime arbitration. |
| HIGGS exact finger curve-table animator | Superseded or partial | ROCK has mesh/contact/finger pose evidence; no one-to-one 201-point HIGGS curve-table animator was found. | Only needed if exact HIGGS-style finger curl semantics are desired; otherwise ROCK's FO4VR pose system should remain authoritative. |

## Partial Or Different Implementations

### Selection visuals

HIGGS combines shader highlighting, rollover text, and selection beam behavior. ROCK currently has selection and highlight behavior, including actor-equipment visual selection, but it does not reproduce the HIGGS UI presentation stack.

This is a feature gap only if ROCK wants equivalent user-facing prompt/beam behavior. It is not evidence that ROCK selection itself is absent.

### One-hand hold drive

HIGGS production grabs are dynamic, with constraint-based held-object drive. ROCK has dynamic held-object behavior, but its current single-hand hold path uses the FO4VR-native mouse-spring drive while HIGGS-style constraint drive appears focused on shared peer-held loose-object joins.

This should be treated as an implementation difference, not a missing grab system. If full HIGGS-style one-hand constraint authority is desired, it needs a separate design pass because it touches held-object stability, release velocity, collision suppression, player-space compensation, and two-hand handoff.

### Far pull and actor equipment

ROCK has far pull, catch, and actor-equipment drop handoff. HIGGS also has broader actor equipment looting semantics and other-hand looting states. The remaining gap is not "far pull"; it is the wider inventory/loot interaction surface around actors and cross-hand ownership.

### Player-space compensation

ROCK compensates held objects for player movement and now enables the runtime transform-warp path for held bodies by default. HIGGS still has broader contained-rigid-body propagation. ROCK remains partial only for bodies resting in/on held containers, not for the primary held-body room-rotation warp.

### Weapon collision and two-handing

ROCK exceeds HIGGS in generated weapon collision, weapon part evidence, reload/body interaction, and support-hand authority. The gap is the HIGGS public API/event parity around weapon collision control and two-hand start/stop notifications, not the core weapon collision feature.

## ROCK Systems That Supersede HIGGS

These should not be treated as missing merely because they differ from HIGGS:

- Bone-derived hand collider architecture instead of HIGGS fixed hand box assumptions.
- ROCK hand layer 43 policy instead of HIGGS layer 56.
- Generated weapon collision and weapon-part evidence.
- FRIK/provider-facing frame snapshots, external body/contact publication, and diagnostic overlay/input suppression APIs.
- Mesh contact patch and multi-finger validation systems.
- FO4VR-specific actor-equipment far-pull handoff.
- Soft-contact world haptics.

## Dependency Shape For Future Implementation

This is not an implementation order. It is the dependency shape implied by the gap map.

- Stash and consume are inventory/use systems layered on top of held-object ownership, release cleanup, haptics, and public events.
- Dropped-object sounds, damage, and cleanup need a persistent post-release body registry and contact listener ownership model.
- Public HIGGS API parity requires an ABI decision: native callback table, F4SE messages, Papyrus events, or a deliberate combination.
- Whole body/ragdoll grabbing should be designed with SCISSORS/external body ownership instead of copied directly from HIGGS.
- Nearby damping and full transform warp should not be enabled in production until the FO4VR hknp writes they rely on are verified.
