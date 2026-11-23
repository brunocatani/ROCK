# devbench: live access to ROCK in the game

[devbench](https://github.com/ArthurHub/devbench) is a separate F4SE plugin that runs a local MCP and REST server inside Fallout 4 VR. ROCK registers one tool with it, `rock`, so an agent or a script can read ROCK's state, override its settings for the session and read its profiler while the game runs. devbench is a development tool: without it ROCK logs one line and runs as always.

## Calling the tool

Every call names its operation in the required `action` argument.

- **MCP**: register devbench's bridge once per machine, from this repository: `claude mcp add --scope local devbench-fo4vr -- <path to devbench-bridge.exe> --game fo4vr`. It is available from the next session.
- **REST**: `POST http://127.0.0.1:8931/api/tool/rock` with the arguments as a JSON object. Events: `GET /api/events?since=N`.

```powershell
irm http://127.0.0.1:8931/api/tool/rock -Method Post -ContentType application/json -Body '{"action":"state"}'
(irm http://127.0.0.1:8931/api/tool/rock -Method Post -ContentType application/json -Body '{"action":"perf","format":"text"}').text
```

An answer is a JSON object with `"ok": true`, or `{ "ok": false, "error": "..." }` with no other keys.

## Actions

They follow F4VR-CommonFramework's generic actions (FRIK's and Mods Config's tools), so the same calls work across these mods. `health` reports the tool's `contract` number, bumped when an action changes its arguments or its answer's shape.

| Action | Arguments | Thread | Answer |
| --- | --- | --- | --- |
| `health` | | listener | ROCK and devbench builds, `contract`, `armed`, `hasSnapshot`, the latest `liveness`, the actions, and `unsupported` (`input`, `snapshot`) |
| `state` | | listener | The latest frame's state; see [State](#state) |
| `hand` | `hand`: `left`, `right` or `both` (default) | frame end | One hand in detail; see [Hand](#hand) |
| `config` | `section`, `key` | game | The option as ROCK reads it from this frame (`value`), its `file` and `fileValue`, and `overriddenBy` (`devbench`, `modsconfig` or null) |
| `set` | `section`, `key`, `value` | game | Overrides the option for the session, applied from this frame; the file is unchanged. A value its type cannot hold sets nothing |
| `clear` | `section`, `key`, or `all: true` | game | Drops devbench's own overrides; Mods Config's live preview stays |
| `overrides` | | game | Every session value, each with its `owner` and whether it `applies` |
| `perf` | `reset`, `format` | listener | The profiler window; see [Perf](#perf) |

- `section` is the option's TOML table (`Grabbing`, `FiringHand.Left`) and is required: ROCK has many tables and no main one.
- **listener** actions answer at once on devbench's thread, so they reply while the game is stalled. **game** actions are queued and run at ROCK's next FrameBegin, before that frame's settings apply; while FRIK's frame is not running (main menu, loading) they go to devbench's main-thread queue. **frame end** actions run at ROCK's next FrameEnd, after the frame's interaction work, and fail while FRIK's frame is not running. The caller waits up to 2 seconds.
- Session values are the same layer as Mods Config's live preview, each kept by its owner (`PreviewOwner` in `src/RockConfig.h`). Of two owners' values of one option the last set applies, and each owner drops only its own.

## State

Published at every FRIK FrameEnd once the tool is armed, and read without waiting for the game thread. A call before the first snapshot waits up to 2 seconds for it.

- `liveness`: `frame` (snapshots since the tool armed), `ageMs` (a growing value means FRIK's frame stopped), `publishedAtMs` (steady clock, shared with other mods' snapshots), `providerGeneration`, `skeletonGeneration`.
- `frik`: `bound`, `apiVersion`.
- `config`: `revision` (every apply, previews included) and `fileRevision` (reloads of the files that changed a value).
- `interaction`: null until FRIK's skeleton is ready, then:
  - `playerAvailable`, `weaponDrawn`, `menuBlocking`, `inputMenuBlocking`, `loading`.
  - `hands.left` / `hands.right`: `state` (the hand's state machine, `HandState`), `held` (`none`, `object`, `loose-weapon`, `equipped-weapon`, `weapon-part`), `heldObject` and `selection` (`formId`, `name`; the selection also `kind` and `far`).
  - `weapons`: `twoHandedGrip` (`TwoHandedState`), `primary` (equipment index 0) and `secondary` (ROCK's second weapon, index 1), each `formId`, `name`, `firingHand`, or null.

The snapshot holds plain values copied on the frame thread (`src/devbench/RockDevBenchState.h`); the hands and weapons come from `PhysicsInteraction::captureDevBenchState` (`core/interaction/DevBenchState.cpp`). A new field goes in both, then in `stateJson` in `src/devbench/RockDevBench.cpp`.

## Hand

`hand` reads one hand or both at FrameEnd, after the frame's grab work, from the same sources the debug overlay draws: the frame context (`buildFrameContext`) and the hand's grab debug snapshots. Positions are world, in game units; a transform is `pos`, `rot` (the rotation matrix by rows) and `scale`. Without a `PhysicsInteraction` it answers `interaction: null`.

- `state`, `holding`, `worldReady` (the player's Havok world is the one ROCK built against; the poses and solve below need it).
- `poses`: `rawHand` (FRIK's raw hand), `grabAnchor`, `palmNormal`, `pointing`, `disabled`.
- `selection`, or null: the reference (`formId`, `baseFormId`, `name`), `kind`, `far`, `distance`, `bodyId`, and when known `hitPoint`, `hitNormal`, `score`, `hmdConeDot`.
- `grab`, or null while not holding:
  - `object`, `kind`, `looseWeapon`, `liveLimb`, `actorBodies`, `bodyId`, `heldBodyIds`, `inverseMass`, `trace` (the `trace=` of this grab's lines in ROCK.log).
  - `constraint`: `id`, `angularAuthority`, `tau`, `maxForce`, `targetMaxForce`; `clock`: `physicsHz`, `sourceIntervalSeconds`; `fingerCurls` (null when the hold never ran the mesh finger solve).
  - With the world: `heldRoot`, `pivot` (hand and object pivots, `errorGameUnits`), `solve` (acquisition phase, pivot authority source, grab point modes, pivot, tracking and rotation errors, live and desired body), `supportFrame`, `palmAuthority`.

The dump is `PhysicsInteraction::describeHandForDevBench` (`core/interaction/DevBenchState.cpp`); a new field goes there.

## Events

Published as `rock.<topic>` while devbench is installed. They share one small ring with every mod's events, so ROCK emits on a change, never every frame.

| Event | When | Payload |
| --- | --- | --- |
| `sessionLoaded` | A save loaded or a new game started | `newGame` |
| `skeleton.ready` | FRIK's skeleton is ready | `skeletonGeneration` |
| `skeleton.destroying` | FRIK's skeleton is going away | |
| `interaction.created`, `interaction.destroyed` | `PhysicsInteraction` came or went | |
| `hand.grab`, `hand.release` | A hand started or stopped holding an object | `hand`, `formId`, `name`, `kind` |
| `weapon.equipped`, `weapon.unequipped` | An equipment index gained or lost a weapon (a switch is both) | `slot`, `formId`, `name`; `firingHand` when equipped |
| `config.reloaded` | A reload of the files changed a value | `fileRevision` |
| `config.override` | devbench's `set` overrode a key it did not before, or `clear` dropped one or all | `owner`, `section`, `key`, `value` (null when cleared), or `all` |

The state events come from comparing each snapshot with the one before, in `publishState`; the lifecycle events are emitted where ROCKMain handles the message. Emit from any thread with `devbench::emit(topic, makePayload)`, which builds nothing while devbench is absent.

## Perf

`perf` reads `PerformanceProfiler`'s devbench window: every scope, counter and value metric since the window started, all threads together. ROCK's scopes do not nest, so `format: tree` (the default, for parity) answers as `flat`; `text` is a table sorted by total time.

- Arming the tool turns the profiler's collection on whatever `PerformanceProfilerEnabled` says; with the tool armed, that setting adds only `ROCK_Profiler.log` and its intervals.
- The window starts after the profiler's warmup frames, and again on `reset: true` or a change of the profiler's settings.
- While the log is on, a reset cannot clear the counts the log's interval still owns, so it subtracts them instead; a scope's `maxMs` may then predate the reset by up to one log interval, and the answer says so in `note`.

## Code

| File | Contents |
| --- | --- |
| `src/devbench/RockDevBench.cpp` | The tool: registration, the action table, the game-thread queue, state snapshots, events |
| `src/devbench/RockDevBenchState.h` | The state snapshot's plain values |
| `src/physics-interaction/core/interaction/DevBenchState.cpp` | Hands and weapons for the snapshot, and the hand action's dump |
| `src/devbench/vendor/` | devbench's MIT client API, from devbench 1.36.0 (`bd152d4`) |

- ROCKMain registers the tool at `kGameLoaded`, before FRIK's API is bound, so `health` answers even when ROCK disables itself. Registering arms the tool: the profiler collects and FrameEnd publishes state.
- A new action is one row in `actions()` in `RockDevBench.cpp`: a game action for anything that reads or changes game or ROCK state, a frame end action for what reads `PhysicsInteraction` after the frame's work (its handler finds it in `s_frameEndInteraction`), a listener action only for reads of atomics or data guarded for any thread. Start its argument descriptions with the actions that read them.
- The tool uses only `GetBuildNumber`, `RegisterTool` and `EmitEvent`, which every devbench has. A later slot of `IDevBenchInterface001` must be gated on `GetBuildNumber()`.
- The vendor files are devbench's `include/` copied byte for byte; `.clang-format` there keeps them unformatted. To update, copy all three files over them and record the version and commit here.
