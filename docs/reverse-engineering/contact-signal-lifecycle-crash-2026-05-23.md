# Contact Signal Lifecycle Crash - 2026-05-23

Project: ROCK

Source authority: local crash logs, ROCK source, hFRIK source, approved Ghidra MCP inspection of Fallout4VR.exe

External web: not used

Confidence: high for crash vector, medium for exact cross-thread timing

## Finding

The 2026-05-23 cell/teleport crash sequence points at ROCK mutating FO4VR's native hknp contact hkSignal slot list during FRIK skeleton teardown/reinitialization.

Evidence:

- `crash-2026-05-23-22-25-13.log` faults in `Fallout4VR.exe+1725BB0`, inside the native hkSignal unsubscribe traversal. The stack reaches that site from `PhysicsInteraction::unsubscribeContactEvents`.
- `crash-2026-05-23-22-44-51.log` faults while executing through the contact event dispatch path. The stack is in `hknpEventMergeAndDispatcher`, `hknpSimulationContext::dispatchCommands`, `hknpWorldEx::preSolve`, and `bhkWorld::Update`.
- The same runtime window has FRIK dispatching `kSkeletonDestroying`, ROCK destroying `PhysicsInteraction`, then FRIK dispatching `kSkeletonReady` and ROCK creating a new `PhysicsInteraction`.

Ghidra-verified FO4VR functions:

- `0x1417F7060`: hkSignal dispatch loop used by hknp contact event dispatch. It calls through the slot vtable and callback pointer while iterating the signal list.
- `0x141725B70`: native signal unsubscribe traversal/removal. The earlier crash occurred at the traversal compare, consistent with a stale or corrupt slot pointer.
- `0x1403B9E50`: native subscribe helper used by ROCK. It allocates and inserts a `hkSignal2<hknpEventHandlerInput const &, hknpEvent const &>::MemberSlot<FOCollisionListener,...>` style slot.
- `0x1403C7160`: member-slot dispatch thunk. It loads the userData at slot `+0x10`, adjusts it by slot `+0x20`, and jumps through the callback pointer at slot `+0x18`.

## Implementation Rule

ROCK must not explicitly call the native signal unsubscribe function for contact callbacks during skeleton, cell, teleport, power armor, or provider lifecycle teardown.

The contact callback registration is now treated as a native slot owned by the hknp world. ROCK keeps a stable bridge as the native userData and atomically activates or deactivates the current `PhysicsInteraction` instance behind that bridge.

On shutdown:

- clear the active ROCK instance from the bridge;
- remember every subscribed world/signal pair so later reinitialization can reuse retained native slots instead of creating duplicates;
- keep bridge world/signal identity even when ROCK cannot prove the world is live, because the native slot may still exist and cannot be safely removed;
- rely on hknp world cleanup for native slot destruction.

On callback:

- require the callback userData to be ROCK's stable bridge;
- require an initialized active `PhysicsInteraction`;
- require the callback's hknp world to match the bridge's current world;
- reject missing or mismatched world data. Do not fall back to the subscribed world.

## ROCK Creation Gate

ROCK must not create Havok bodies directly from FRIK's synchronous `kSkeletonReady` callback. The callback only requests creation. The ROCK frame loop owns the actual create/recreate decision after:

- at least one ROCK frame has elapsed since the ready event;
- ROCK's local menu/input gate is clear, including loading-menu tracking;
- FRIK's skeleton API still reports ready;
- the current player bhk/hknp world pair has remained stable for consecutive frames.

This keeps ROCK independent of FRIK's incomplete menu aggregate and prevents same-callstack destroy/ready transitions from immediately touching hknp state.

## hFRIK Interaction

hFRIK legitimately dispatches `kSkeletonDestroying` when its root node is released, loading menus open, power armor state changes, or the skeleton otherwise becomes invalid. It can dispatch `kSkeletonReady` again in the same frame after reinitialization. That timing amplifies ROCK's old unsubscribe/resubscribe race, but the direct native memory corruption evidence is in ROCK's contact signal lifecycle handling.

Future hFRIK work may still choose to defer same-frame reinitialization after root-node loss, but that is not required for this ROCK fix.
