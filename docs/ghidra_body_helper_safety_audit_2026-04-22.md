# Ghidra Audit — Body Helper Safety Contract — 2026-04-22

## Why this audit exists

The source review found that ROCK still exposes raw helper paths that index:

- `world + 0x20 + bodyId * 0x90`
- `world + 0xE0 + motionIndex * 0x80`

with only sentinel or heuristic checks. This audit was done to settle the native FO4VR safety contract before changing those helpers:

1. where the authoritative body bounds live
2. what the native motion-slot validity contract actually is
3. whether stock engine wrappers do more validation than ROCK currently does
4. which runtime paths are safer than direct raw indexing

## Wave items and verdicts

| Item | Raw identifier | Existing interpretation | Blind verdict |
|---|---|---|---|
| 1 | `0x141DF60C0` | world-side body validation / wake-notify helper | **CONFIRMED** |
| 2 | `0x141539F30` | body linear+angular velocity setter | **CONFIRMED** |
| 3 | `0x14153A250` | body linear velocity setter | **CONFIRMED** |
| 4 | `0x14153A350` | body angular velocity setter | **CONFIRMED** |
| 5 | `0x141546350` | motion allocation path | **CONFIRMED** |
| 6 | `0x141E0C460` | physics-system body-id lookup helper | **CONFIRMED** |
| 7 | `0x141E07A10` | `bhkNPCollisionObject::Getbhk(bhkWorld*, bodyId)` | **CONFIRMED** |

## Detailed findings

### Item 1 — `0x141DF60C0`

Blind read:

- locks the world at `world + 0x690`
- rejects body id `0x7fffffff`
- rejects `bodyId >= (*(world + 0x2C) & 0x3FFFFFFF)`
- rejects bodies whose `body+0x40 & 3 == 0`
- rejects bodies whose `body+0x6C == -1`

Conclusion:

This is the first clean blind witness in this wave that FO4VR really stores the authoritative body upper bound at:

- `world + 0x2C`, masked with `0x3FFFFFFF`

Important implication:

`bodyId != sentinel` is **not** sufficient for a safe generic helper. The native contract can require:

1. sentinel rejection
2. body-count bounds check
3. active/usable body flags
4. valid secondary body state at `body+0x6C`

### Item 2 — `0x141539F30`

Blind read:

- locks the world
- indexes body directly as `world+0x20 + bodyId*0x90`
- checks `body+0x40 & 2`
- reads `body+0x68` as motion index
- indexes motion directly as `world+0xE0 + motionIndex*0x80`
- does **not** perform a local `bodyId < worldCount` check
- does **not** perform a local motion-count check

Conclusion:

This confirms an important distinction:

- some stock world paths assume they were called with a trusted body id
- the native count validation is **not** universal at every setter/read site

Important implication:

ROCK cannot claim safety merely because “the engine also indexes raw arrays”. The safe contract is split:

- trusted call sites may index directly
- generic reusable helpers should not

### Item 3 — `0x14153A250`

Blind read:

- same world-lock pattern
- direct body lookup from `world + 0x20`
- direct motion lookup from `world + 0xE0`
- no explicit body-count check
- no explicit motion-count check

Conclusion:

The plain linear velocity setter follows the same trusted-caller pattern as the broader velocity setter.

### Item 4 — `0x14153A350`

Blind read:

- same world-lock pattern
- direct body lookup from `world + 0x20`
- direct motion lookup from `world + 0xE0`
- no explicit body-count check
- no explicit motion-count check

Conclusion:

The angular velocity setter also assumes the body id and motion slot are already valid.

### Item 5 — `0x141546350`

Blind read:

- allocates motion through a manager rooted at `world + 0xD8`
- writes the motion payload into `world + 0xE0 + motionId * 0x80`
- also grows/updates a companion table rooted at `world + 0x4A0`
- ensures `*(world+0x4A0 + 0x10) >= motionId + 1`
- uses `*(world+0x4A0 + 0x14) & 0x3FFFFFFF` as capacity during growth

Conclusion:

This is the strongest blind witness in this wave for the motion-side contract:

- the motion array at `world + 0xE0` is real
- its bookkeeping is managed through dedicated motion-manager state, not a simple “public motion count” checked everywhere

Important implication:

The current ROCK/CLib-style rule:

- `motionIndex == 0` reject static
- `motionIndex > 4096` reject garbage

is still only a **heuristic guard**, not the native authoritative contract.

### Item 6 — `0x141E0C460`

Blind read:

- reads a physics-system instance pointer
- returns sentinel `0x7fffffff` if the instance is missing
- otherwise indexes a body-id array at `instance + 0x20`
- caller-provided element index comes from `param_3`
- no local bounds check was visible in the decompiled slice

Conclusion:

The instance path already contains the real body-id array and count layout:

- body-id array at `instance + 0x20`
- count at `instance + 0x28`

but this helper itself is not a “fully safe accessor” in the generic sense.

### Item 7 — `0x141E07A10`

Blind read:

- validates world and sentinel body id
- locks the world
- resolves the live body row
- returns `*(body + 0x88)` as the collision object

Conclusion:

For body -> collision object -> owner resolution, the engine-backed path remains stronger than a generic raw helper.

## Questions answered

### 1. Where is the authoritative body count stored for `hknpWorld`?

Confirmed witness in this wave:

- `world + 0x2C`, masked with `0x3FFFFFFF`

This field is used by `0x141DF60C0` to reject out-of-range body ids.

### 2. What is the authoritative motion array bounds contract?

What is confirmed:

- motion data lives at `world + 0xE0`, stride `0x80`
- motion allocation/management is rooted at `world + 0xD8`
- companion bookkeeping/table growth is managed through `world + 0x4A0`
- `*(world+0x4A0 + 0x10)` is maintained to cover at least `motionId + 1`

What is **not** confirmed:

- no single public “motion count” field that the audited stock setters consult before indexing

Practical conclusion:

The native runtime appears to trust engine-assigned `body.motionIndex` once the body is in the correct active state. That means ROCK's current hardcoded `motionIndex > 4096` guard is not the authoritative FO4VR rule; it is only a defensive heuristic.

### 3. Is `bodyId != sentinel` ever enough for safe helper use?

No.

The strongest native guard seen in this wave requires more than the sentinel check:

- `bodyId != 0x7fffffff`
- `bodyId < (world+0x2C & 0x3FFFFFFF)`
- `(body+0x40 & 3) != 0`
- `body+0x6C != -1`

### 4. Which wrapper or engine path is safer than direct body-array indexing for owner lookup and COM queries?

For owner lookup:

- `bhkNPCollisionObject::Getbhk(bhkWorld, bodyId)` is the safer engine-backed path
- it uses the real body -> collision object contract (`body + 0x88`) under the world lock

For COM/motion reads:

- no dedicated high-level read wrapper was proven in this wave
- the safer contract is to mirror the native validation sequence before reading:
  - sentinel reject
  - body-count bounds reject
  - body active reject
  - body secondary-state reject
  - then read `body+0x68` / `world+0xE0`

This means a generic helper that does only:

- `bodyId != sentinel`
- `motionIndex != 0`
- `motionIndex <= 4096`

is still weaker than the strongest native validation pattern seen in the binary.

## Repair implication

Wave 2 confirms the source review finding:

- generic raw helper paths in `PhysicsUtils.h` are stale and under-validated
- direct `world->GetBodyArray()[bodyId]` helper usage should not be exposed as the default read contract
- owner lookup should prefer `Getbhk(...)`
- COM/motion helpers should either:
  - be retired behind stronger validated wrappers, or
  - adopt the native validation sequence proven in `0x141DF60C0`

## Net result

**Wave 2 is complete.**

The FO4VR binary does expose a real body-count bound and a real motion-management contract, but stock setters do not universally enforce those bounds at every site. Therefore ROCK's current generic raw helpers remain too weak to be treated as authoritative utility paths.
