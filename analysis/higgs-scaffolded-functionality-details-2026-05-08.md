# HIGGS Scaffolded Functionality Details

Date: 2026-05-08

Canonical note path: `ROCK\analysis\higgs-scaffolded-functionality-details-2026-05-08.md`

## Decision Note

This note captures the HIGGS-derived gameplay/API systems that ROCK already has state-machine or ABI scaffolding for, but does not yet fully execute at runtime. It is split out from the wider HIGGS gap map so future implementation work can align on behavior, ownership, events, and cleanup before code changes. HIGGS is used here as a functionality inventory because the current task explicitly requested it; ROCK's FO4VR-native architecture remains authoritative for final implementation choices.

No web or Ghidra was used for this pass. This note does not make new binary-layout, address, or offset claims.

## ROCK Scaffolding Already Present

- `HandState::GrabExternal`
- `HandState::LootOtherHand`
- `HandState::StashCandidate`
- `HandState::ConsumeCandidate`
- `HandState::SelectedTwoHand`
- `HandState::HeldTwoHanded`
- `HandInteractionEvent::BeginExternalGrab`
- `HandInteractionEvent::BeginLootOtherHand`
- `HandInteractionEvent::CompleteLoot`
- `HandInteractionEvent::BeginStashCandidate`
- `HandInteractionEvent::CommitStash`
- `HandInteractionEvent::BeginConsumeCandidate`
- `HandInteractionEvent::CommitConsume`
- `HandInteractionEvent::BeginTwoHandSelection`
- `HandInteractionEvent::BeginTwoHandHold`
- `HandInteractionEvent::EndTwoHandHold`
- API-visible reserved grab events: `TwoHandStarted`, `TwoHandStopped`, `StashCandidate`, `ConsumeCandidate`, `Stashed`, `Consumed`, `LootStarted`, `LootCompleted`

## 1. Shoulder Stash To Inventory

### HIGGS Behavior

While holding a playable object, HIGGS checks whether the hand is settled near either shoulder. If the held object qualifies and the hand is inside the shoulder zone, HIGGS treats it as a stash candidate, pulses haptics while the object remains in that zone, and on release activates or picks up the object so it enters the player inventory.

HIGGS also fires stashed callbacks and Papyrus events before the held reference is consumed by activation/pickup, because after commit the physical reference may no longer exist.

### Relevant HIGGS Evidence

- `src\hand.cpp:1908` - `IsObjectDepositable`
- `src\hand.cpp:3108` - shoulder release commit path
- `include\config.h:39` - shoulder velocity threshold
- `include\config.h:342` - right/left shoulder HMD offsets
- `include\config.h:346` - shoulder radii
- `include\higgsinterface001.h:28` - stashed callback contract

### Current ROCK Status

ROCK has the state-machine shape but not the gameplay system:

- `HeldBody -> StashCandidate`
- `StashCandidate -> HeldBody` on candidate cancel
- `StashCandidate -> Idle` on `CommitStash`
- `StashCandidate` remains a holding/exclusive state
- `StashCandidate` and `Stashed` are ABI-visible event reservations

No current ROCK runtime path was found that detects shoulder zones, validates stash eligibility, emits candidate haptics, transfers the item into inventory, or dispatches the final stashed event.

### Required ROCK Runtime Work

- FO4VR-native shoulder zone source, likely from HMD/body/provider data rather than copied HIGGS offsets.
- Config surface for enablement, shoulder radii, zone offsets, velocity gate, and haptic strength/duration.
- Eligibility policy for playable loose objects, actor equipment, quest objects, books/notes, containers, moveable statics, and excluded refs.
- Candidate detector that enters `StashCandidate` only after the held object is stable enough to avoid accidental commits.
- Candidate cancellation when the hand leaves the zone, the object becomes invalid, the world changes, or the other hand takes ownership.
- Commit transaction ordering:
  - dispatch candidate/final event while identity is still available,
  - transfer/activate/pick up the item,
  - release held physics ownership,
  - restore collision/filter/body state,
  - clear player-space and haptic state.
- Tests for state transitions, config parity, event reservation, and source-boundary checks around inventory commit.

### Alignment Questions Before Implementation

- Should shoulder stash be enabled by default or opt-in while tuning?
- Should it apply to all playable loose objects, or start with a restricted set such as aid/ammo/misc/books?
- Should actor-equipment items that ROCK/PAPER handed off be stashable immediately?
- Should books/notes open/read, or always go directly to inventory?
- Should commit happen on release only, or after dwelling in the zone for a configured time?

## 2. Mouth Consume / Use Item

### HIGGS Behavior

HIGGS checks whether a held consumable is close to a mouth position derived from the HMD. It pulses haptics while the object is in the mouth zone, and on release it activates/equips/consumes the held item. HIGGS supports Skyrim potions, ingredients, and books, blocks poisons unless configured, and emits consumed callbacks/Papyrus events before the physical reference is removed or transformed by activation.

### Relevant HIGGS Evidence

- `src\hand.cpp:1945` - `IsObjectConsumable`
- `src\hand.cpp:3085` - mouth release commit path
- `include\config.h:40` - mouth velocity threshold
- `include\config.h:344` - mouth HMD offset
- `include\config.h:348` - mouth radius
- `include\higgsinterface001.h:33` - consumed callback contract

### Current ROCK Status

ROCK has the state-machine shape but not the item-use executor:

- `HeldBody -> ConsumeCandidate`
- `ConsumeCandidate -> HeldBody` on candidate cancel
- `ConsumeCandidate -> Idle` on `CommitConsume`
- `ConsumeCandidate` remains a holding/exclusive state
- `ConsumeCandidate` and `Consumed` are ABI-visible event reservations

No current ROCK runtime path was found that classifies FO4VR consumables, detects a mouth zone, performs item use, or dispatches consumed events.

### Required ROCK Runtime Work

- FO4VR item classification for aid, food, drink, chem, holotape/book-like refs, and any excluded types.
- Safe "use one item from held ref" executor that respects inventory count, activation side effects, and object disappearance.
- Mouth zone source from HMD/body/provider data.
- Anti-accidental-consume policy: release-only commit, velocity gate, dwell time, or both.
- Haptic candidate and final haptic events.
- Event dispatch before activation/removal invalidates the reference.
- Failure handling if activation fails, the item cannot be used, the ref disappears, or another hand owns the same body.

## 3. Programmatic External Object Grab

### HIGGS Behavior

HIGGS exposes `GrabObject(object, isLeft)` through its native/Papyrus API. The call records an external grab request on the chosen hand, and the normal hand update later attempts to process that request through the grab system.

### Relevant HIGGS Evidence

- `src\pluginapi.cpp:250` - `HiggsInterface001::GrabObject`
- `include\pluginapi.h:52` - native API method
- `src\papyrusapi.cpp:289` - Papyrus `GrabObject`

### Current ROCK Status

ROCK has the state-machine shape:

- `Idle/selection -> GrabExternal`
- `GrabExternal -> HeldInit` when the spawned/requested item is ready
- `GrabExternal` is treated as a gameplay scaffold and release/object/world invalidation candidate

No public ROCK API was found that accepts an external object-grab request, validates it, queues it safely, and drives the normal held-object commit path.

### Required ROCK Runtime Work

- Public request API, likely native first and Papyrus later only if explicitly chosen.
- Per-hand request queue or single pending request with overwrite/reject policy.
- Main-thread/frame processing so external callers cannot mutate hand/physics state directly.
- Validation policy: target ref exists, collision/physics body exists, object is grabbable, hand is available, world is valid, no conflicting owner.
- Integration with the normal `CommitGrab` path so save/restore, collision suppression, player-space compensation, haptics, and events stay consistent.
- Structured result reporting for accepted, rejected, already held, invalid target, no physics body, hand busy, and world unavailable.

## 4. Loot-From-Other-Hand Runtime

### HIGGS Behavior

HIGGS enters `LootOtherHand` when one hand tries to interact with an actor/equipment-related object that the other hand is already holding. While in that state, pulling away from the held object past a velocity threshold commits a loot-style transfer, while pushing toward it can convert into a physical grab/transfer.

### Relevant HIGGS Evidence

- `src\hand.cpp:2848` - selection branch that enters `LootOtherHand`
- `src\hand.cpp:3317` - runtime pull-away/push-in gesture handling

### Current ROCK Status

ROCK has the state-machine shape and selection context:

- `SelectedClose/SelectionLocked -> LootOtherHand`
- `LootOtherHand -> PreGrabItem` on `CompleteLoot`
- `LootOtherHand` is an exclusive object-selection state
- `OtherHandSelectionContext` distinguishes exclusive peer refs from shareable held refs
- API-visible events reserve `LootStarted` and `LootCompleted`

No complete ROCK runtime path was found for loot gesture classification, inventory/equipment transfer, cross-hand rollback, or public event dispatch.

### Required ROCK Runtime Work

- Define what FO4VR "loot from other hand" means:
  - actor equipment only,
  - actor equipment plus dropped actor items,
  - any loose object held by the other hand,
  - or a stricter ROCK/PAPER-specific subset.
- Cross-hand ownership transaction so one hand cannot duplicate, delete, or leave stale physics state in the other hand.
- Pull-away and push-in gesture classifier with thresholds, leeway timing, and haptic feedback.
- Integration with actor-equipment drop handoff and possible PAPER/SCISSORS ownership.
- `LootStarted` and `LootCompleted` event dispatch with failure/abort cleanup.
- Tests for exclusive selection, shareable held refs, two-hand conflict, and invalidation rollback.

## 5. Two-Handed Start/Stop Public Events

### HIGGS Behavior

HIGGS fires native callbacks and Papyrus events when weapon two-handing starts or stops. These events are public notifications over behavior HIGGS already owns.

### Relevant HIGGS Evidence

- `src\pluginapi.cpp:97` - add start-two-handing callback
- `src\pluginapi.cpp:103` - add stop-two-handing callback
- `src\pluginapi.cpp:192` - trigger start-two-handing callbacks/events
- `src\pluginapi.cpp:200` - trigger stop-two-handing callbacks/events
- `src\hand.cpp:1880` - start event call site
- `src\hand.cpp:3739` - stop event call site
- `src\papyrusapi.cpp:319` - Papyrus start event registration
- `src\papyrusapi.cpp:322` - Papyrus stop event registration

### Current ROCK Status

ROCK already has substantial two-hand behavior, especially generated weapon collision and support-hand authority. The missing part is event bridge parity, not core behavior.

Existing scaffolding includes:

- `SelectedTwoHand`
- `HeldTwoHanded`
- `BeginTwoHandSelection`
- `BeginTwoHandHold`
- `EndTwoHandHold`
- `EnterTwoHandAuthority`
- `ExitTwoHandAuthority`
- API-visible `TwoHandStarted` and `TwoHandStopped`

No complete bridge was found that emits public `TwoHandStarted`/`TwoHandStopped` events exactly once as the real weapon/support-grip or loose-object two-hand authority changes.

### Required ROCK Runtime Work

- Central edge detector for two-hand authority became active/inactive.
- Coverage for weapon support-grip authority and loose-object two-hand authority.
- Exactly-once event dispatch across normal stop, release, weapon swap, visual loss, object invalidation, and world invalidation.
- Event payload policy: hand, source kind, ref/form/body id where available.
- Tests that prevent duplicate start spam and guarantee stop emission on abort paths.

