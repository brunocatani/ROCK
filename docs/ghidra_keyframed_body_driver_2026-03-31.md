# Ghidra RE: Keyframed Body Driver Functions (2026-03-31)

Blind reverse engineering of three functions involved in driving keyframed bodies
to target transforms every frame in Fallout 4 VR (Fallout4VR.exe).

---

## Item 1: 0x141e086e0 — bhkRigidBody::DriveToKeyframedTarget

### What it is
This is the **per-body keyframed driver**. It takes a single bhkRigidBody wrapper,
a target NiTransform (as 4x4 matrix passed as pointer), and a deltaTime float.
It computes the velocities needed to reach the target pose, checks against velocity
limits from a quality table, and either applies those velocities OR teleports if
the velocities would exceed the limits.

### Proposed name
`bhkRigidBody::DriveToKeyframedTarget`

### Exact signature
```cpp
// Returns bool in AL (true = success, false = no world or deltaTime <= 0)
// param_1 = bhkRigidBody* (this pointer, Bethesda wrapper around hknpBody)
// param_2 = NiTransform* (4x4 row-major matrix: 3x4 rotation + translation, as 64 bytes)
// param_3 = float deltaTime (frame delta in seconds)
bool bhkRigidBody_DriveToKeyframedTarget(bhkRigidBody* body, NiTransform* targetTransform, float deltaTime);
```

### Detailed behavior (step by step)

1. **TLS guard**: Reads TLS offset `+0x9C0`, saves old value, writes `0x41` (marks "in keyframe driver")
2. **Early exit**: Gets hknpWorld from `body->physicsSystem->world` via `GetHknpWorld(body+0x20)`. If world is null OR deltaTime <= 0, restores TLS and returns false.
3. **Rotation-to-quaternion**: Calls `FUN_141722c10` which is `RotationMatrixToQuaternion(out_quat, targetTransform)` — converts the 3x3 rotation part of the NiTransform into a quaternion. Output stored in local `local_58[32]`.
4. **Get body ID**: Calls `bhkPhysicsSystem::GetBodyId(body+0x20, &outId, body+0x28)` to get the hknpBodyId.
5. **Body array lookup**: `bodyEntry = world->bodyArray[bodyId * 0x90]` — gets the body's entry from the world.
6. **Quality table lookup**:
   - Reads `body->qualityIndex` from `bodyEntry+0x68`
   - Gets quality entry from `world->qualityTable (world+0xE0)` at `qualityIndex * 0x80`
   - Gets deactivation data from `world+0x5D0`, reads quality table pointer at `+0x28`, indexes at `qualityIndex * 0x40`
   - **Velocity limits** are at quality entry offsets `+0x10` (maxLinearVelocity) and `+0x14` (maxAngularVelocity)
7. **Compute velocities**: Calls `hknpWorld::computeHardKeyFrame(world, bodyId_from_entry+0x60, targetTransform+0x30, targetQuat, deltaTime, &linVelOut, &angVelOut)`
8. **Velocity limit check**:
   - `linVelSqMag = linVelOut.x^2 + linVelOut.y^2 + linVelOut.z^2`
   - `angVelSqMag = angVelOut.x^2 + angVelOut.y^2 + angVelOut.z^2`
   - `maxLinSq = maxLinearVelocity^2`
   - `maxAngSq = maxAngularVelocity^2`
   - If `linVelSqMag > maxLinSq` OR `angVelSqMag > maxAngSq` → **TELEPORT path**
9. **TELEPORT path** (velocity too high):
   - Calls `setBodyTransform_or_defer(world, bodyId, targetTransform, 1)` — `FUN_141df55f0` — teleports the body directly. The `1` means "activate after teleport".
   - Sets both velocity pointers to `DAT_1438642b0` which is the **zero vector {0,0,0,0}**
   - Falls through to velocity setter with zeros
10. **NORMAL path** (velocity within limits):
    - Uses the computed `linVelOut` and `angVelOut` as-is
11. **Set velocity**: Calls `setBodyVelocity_or_defer(world, bodyId, linVel, angVel)` — `FUN_141df56f0`
    - This function also checks if both velocities are near-zero, and if so calls `FUN_141df60c0` to deactivate/wake the body
12. **Restore TLS**: Restores old TLS value at `+0x9C0`
13. **Return**: Returns `0x901` (which is `true` in AL due to `MOV AL, 0x1`) on success

### Does it handle deferred mode?
**No, not directly.** The function itself does NOT check `TLS+0x1528`. However, the functions it calls (`FUN_141df55f0` = setBodyTransform and `FUN_141df56f0` = setBodyVelocity) DO check `TLS+0x1528`:
- If `TLS+0x1528 == 0` → immediate Havok call (`hknpWorld::setBodyTransform` / `hknpWorld::setBodyVelocity`)
- If `TLS+0x1528 != 0` → deferred via `DeferredPhysicsWorld::AddCommand`

So the deferred handling is transparent to this function.

### Key evidence
- TLS `+0x9C0` guard write of `0x41`
- Quality table at `world+0xE0`, stride `0x80`, velocity limits at `+0x10` and `+0x14`
- Zero vector constant at `0x1438642b0`
- Returns `0x901` (byte 0x01 = true) on success vs `uVar5 & 0xFFFFFFFFFFFFFF00` (byte 0x00 = false) on early exit
- Assembly confirms: `MOV AL, 0x1` at `0x141e08958` (success) vs `XOR AL, AL` at `0x141e0895c` (failure)

---

## Item 2: 0x141e18f90 — bhkWorld::DriveKeyframedBodies_Recursive (NiNode tree walker)

### What it is
Recursive NiNode tree walker that drives ALL keyframed bodies in a scene graph
to their target world transforms. This is the function called by the engine each
frame to update all keyframed collision bodies in a node hierarchy.

### Proposed name
`bhkWorld_DriveKeyframedBodies_Recursive`

### Exact signature
```cpp
// param_1 = NiAVObject* (current node being processed, cast to NiNode for children)
// param_2 = NiTransform* (parent's accumulated world transform, 4x4 matrix as 64 bytes)
// param_3 = DeferredWorldState* (pointer to pair: {hknpWorld* worldPtr, int lockMode})
void bhkWorld_DriveKeyframedBodies_Recursive(NiAVObject* node, NiTransform* parentWorldTransform, DeferredWorldState* worldState);
```

### Detailed behavior

1. **Null check**: If `node == nullptr`, return immediately.
2. **Transform computation**:
   - Checks bit `(node[0x21] >> 0x2D) & 1` — this is a flag in NiAVObject (likely "use local transform" or "has collision object")
   - If flag is CLEAR: calls `NiTransform::Multiply(parentTransform, &result, node->localTransform)` at `FUN_1401a8d60` — concatenates parent * local to get world transform
   - If flag is SET: copies parentTransform directly (node is at world origin / uses parent transform as-is)
3. **Copies result** into `local_98` (the accumulated world transform)
4. **Gets bhkRigidBody**: Calls `FUN_141e07a90(node)` which does `node->collisionObject->vtable[0x0D8/8]()` — this is `NiCollisionObject::GetRigidBody()` via virtual dispatch at `node+0x100`
5. **Validation checks**:
   - `FUN_141e09530(rigidBody)` — checks if the body is allocated/valid (reads motion type bits from `bodyEntry+0x40`, checks `(motionType >> 2) & 0xFFFFFF01`)
   - `FUN_141e09310(rigidBody)` — checks if the body's quality entry maps back to this bodyId (consistency check)
6. **World transform preparation**: Masks rotation/translation components of the accumulated transform (clears W components of rotation rows, scales translation by some factor from `DAT_1465a4f30..3c`)
7. **Deferred world management**:
   - Gets hknpWorld via `FUN_141e07fa0(rigidBody)`
   - Checks `TLS+0x1529`:
     - If `== 0` (not deferred): calls `FUN_141df5fb0(world)` to begin batch, sets lockMode = 1
     - If `!= 0` (deferred mode): sets world to null, lockMode = 2
   - Manages world pointer transitions (if world changed from last iteration, unlocks old, locks new)
8. **Calls Item 1**: `FUN_141e086e0(rigidBody, &worldTransform, DAT_1465a3d84)` where `DAT_1465a3d84` is the **global physics deltaTime** (a float global)
9. **Recurse into children**:
   - Gets NiNode children via vtable call `(*node->vtable[0x20/8])(node)` — this is `NiNode::GetChildren()` or similar
   - Children array at `result+0x168`, count at `result+0x172`
   - For each child: recursively calls `FUN_141e18f90(child, &accumulatedTransform, worldState)`

### Key evidence
- Recursive self-call at end of function
- Children at `+0x168` (VR offset), count at `+0x172` (VR offset) — matches NiNode VR layout
- Transform concatenation via `FUN_1401a8d60` (NiTransform multiply)
- Calls Item 1 with global deltaTime `DAT_1465a3d84`
- Manages deferred world lock/unlock pairs

---

## Item 3: 0x14153a6a0 — hknpWorld::computeHardKeyFrame

### What it is
**CONFIRMED**: This is `hknpWorld::computeHardKeyFrame`. The function computes the
linear and angular velocities needed to move a body from its current position/rotation
to a target position/rotation within one timestep.

### Proposed name
`hknpWorld::computeHardKeyFrame` (already labeled in Ghidra)

### Exact signature
```cpp
// param_1 = hknpWorld* this
// param_2 = int bodyId (NOT hknpBodyId struct — just the raw int from bodyEntry+0x60)
// param_3 = float* targetPosition (4 floats: x, y, z, w — Havok position from world transform)
// param_4 = float* targetQuaternion (4 floats: x, y, z, w — target orientation)
// param_5 = float invDeltaTime (1/dt, NOT deltaTime — see evidence below)
// param_6 = float* linVelOut (output: 4 floats, linear velocity)
// param_7 = float* angVelOut (output: 4 floats, angular velocity)
void hknpWorld_computeHardKeyFrame(
    hknpWorld* world,
    int bodyId,
    float* targetPosition,    // 4 floats
    float* targetQuaternion,  // 4 floats
    float invDeltaTime,       // 1/deltaTime
    float* linVelOut,         // output, 4 floats
    float* angVelOut          // output, 4 floats
);
```

### CRITICAL: param_5 is invDeltaTime (1/dt), NOT deltaTime

**Evidence from the assembly at `0x14153a6ea-0x14153a710`:**
```asm
MOVSS   XMM1, [RSP+0x120]     ; param_5 loaded
SHUFPS  XMM1, XMM1, 0         ; broadcast to all 4 lanes
RCPPS   XMM0, XMM1            ; XMM0 = 1/param_5 (approximate reciprocal)
MULPS   XMM1, XMM0            ; XMM1 = param_5 * (1/param_5) ≈ 1.0
SUBPS   XMM14, XMM1           ; XMM14 = 2.0 - 1.0 = 1.0 (Newton-Raphson step)
MULPS   XMM14, XMM0           ; XMM14 = 1.0 * (1/param_5) = refined 1/param_5
```
This is a **Newton-Raphson RCPPS refinement** computing `1/param_5`. The result (`XMM14`) is used as a MULTIPLIER on the position delta: `(targetPos - currentPos) * (1/param_5)`.

If `param_5` were `deltaTime`, this would compute `displacement / dt = velocity` — **CORRECT physics**.
If `param_5` were `1/deltaTime`, this would compute `displacement / (1/dt) = displacement * dt` — **WRONG, gives displacement scaled by time, not velocity**.

**WAIT**: Let me re-examine. The caller (`0x141e086e0`) passes `param_3` directly as the 5th parameter to computeHardKeyFrame. But Item 1 receives `param_3` as `float deltaTime`. Let me recheck...

Looking at the assembly of Item 1 at `0x141e08866-0x141e08870`:
```asm
MOVSS   dword ptr [RSP + 0x20], XMM6    ; param_5 = XMM6 = param_3 of caller = deltaTime
CALL    0x14153a6a0                       ; computeHardKeyFrame
```

And the tree walker passes `DAT_1465a3d84` which is the global physics deltaTime (seconds per frame, e.g., 0.01111 for 90fps).

So **param_5 of computeHardKeyFrame receives deltaTime directly**. The function internally computes `1/deltaTime` via RCPPS+Newton-Raphson, then multiplies the displacement by `1/deltaTime` to get velocity.

**CORRECTION: param_5 = deltaTime (seconds). The function internally computes invDeltaTime.**

### Detailed computation

**Linear velocity:**
1. Gets body's current position from body array: `currentPos = world->bodyArray[bodyId*0x90]` (the column-major position at float[12,13,14] mapped differently here)
2. Gets quality entry position from `world->qualityTable[qualityIdx * 0x80]`
3. Applies quaternion rotation formula to compute the position delta accounting for rotation:
   - `delta = quat_rotate(targetQuat, currentBodyRow) * 2 + targetPos - qualityPos`
4. `linVelOut = delta * invDeltaTime`
5. Zeroes out W component of linVelOut

**Angular velocity:**
1. Extracts current body rotation as quaternion via `RotationMatrixToQuaternion`
2. Computes relative rotation: `relQuat = targetQuat * inverse(currentQuat)`
3. Normalizes the relative quaternion
4. Checks if rotation magnitude > threshold (`DAT_1438644d0`)
5. If below threshold: `angVelOut = {0,0,0,0}` (no rotation needed)
6. If above threshold:
   - Calls `FUN_141723ad0` to compute rotation angle (likely `acos` or `atan2` based)
   - If angle < small threshold: clamp to 0
   - Computes axis from normalized quaternion xyz
   - `angVelOut = axis * angle * invDeltaTime`
   - Flips sign if quaternion W < 0 (shortest path)

### Key evidence
- RCPPS + Newton-Raphson at top proves internal 1/dt computation
- Body array stride 0x90 confirmed
- Quality table stride 0x80 confirmed
- Lock/unlock pattern via `FUN_1415388f0`/`FUN_141538a30` (world+0x690 mutex)
- Quaternion shortest-path via W sign check
- Small angle threshold at `DAT_1438644d0`

---

## Final Answers

### 1. Can Item 1 be called standalone on a single body?

**YES.** Item 1 (`0x141e086e0`) is fully self-contained. It takes a bhkRigidBody pointer, a target transform, and deltaTime. It internally resolves everything it needs (world pointer, body ID, quality limits). The tree walker is just a convenience that recursively applies it to a whole scene graph.

To call it standalone:
```cpp
using DriveToKeyframed_t = bool(*)(bhkRigidBody* body, NiTransform* targetTransform, float deltaTime);
static REL::Relocation<DriveToKeyframed_t> DriveToKeyframedTarget{ REL::Offset(0x1e086e0) };

// Usage:
NiTransform targetTransform;
// ... fill targetTransform with desired 3x4 rotation + translation ...
bool success = DriveToKeyframedTarget(handRigidBody, &targetTransform, frameDeltaTime);
```

**Required state:**
- `body->physicsSystem` (at `body+0x20`) must be valid (non-null)
- The body must have a valid hknpWorld behind it
- `deltaTime` must be > 0.0
- The body should be a keyframed body (the function doesn't explicitly check motion type, but the quality table velocity limits are what make it work correctly for keyframed bodies)

### 2. Does it handle ALL of: teleport, velocity computation, velocity setting, velocity clamping?

**YES, ALL OF THEM.** This is a complete one-call solution:

| Feature | Handled? | How |
|---------|----------|-----|
| Velocity computation | YES | Calls `computeHardKeyFrame` internally |
| Velocity clamping | YES | Compares against quality table limits |
| Teleport fallback | YES | If velocity exceeds limits, calls `setBodyTransform` to teleport |
| Zero velocities after teleport | YES | Sets both linear and angular to `{0,0,0,0}` |
| Velocity application | YES | Calls `setBodyVelocity` with computed or zeroed velocities |
| Deferred mode support | YES (transparent) | Sub-functions check `TLS+0x1528` |
| Body activation | YES | `setBodyVelocity` internally calls activation if velocities are non-zero |

**You do NOT need separate calls.** This single function is the complete keyframed body driver.

### 3. Exact calling convention and C++ typedef

```cpp
// x64 Microsoft calling convention (standard for FO4VR):
// RCX = param_1 (bhkRigidBody*)
// RDX = param_2 (NiTransform*) — 64-byte 4x4 matrix (3 rotation rows as NiPoint4 + translate row)
// XMM2 = param_3 (float deltaTime) — third parameter is float, goes in XMM2 per MS x64
//
// Returns: bool in AL (1 = success, 0 = no world or bad deltaTime)

// For REL::Relocation:
using DriveToKeyframedTarget_t = bool(__fastcall*)(void* bhkRigidBody, void* targetNiTransform, float deltaTime);
static REL::Relocation<DriveToKeyframedTarget_t> DriveToKeyframedTarget{ REL::Offset(0x1E086E0) };
```

**NiTransform layout expected by this function** (64 bytes / 16 floats):
```
Offset 0x00: rot[0][0], rot[0][1], rot[0][2], rot[0][3]   // Row 0 (NiPoint4)
Offset 0x10: rot[1][0], rot[1][1], rot[1][2], rot[1][3]   // Row 1 (NiPoint4)
Offset 0x20: rot[2][0], rot[2][1], rot[2][2], rot[2][3]   // Row 2 (NiPoint4)
Offset 0x30: trans.x,   trans.y,   trans.z,   scale        // Translation + scale
```

The function reads `targetTransform+0x30` as the position (passed to computeHardKeyFrame param_3),
and converts the 3x3 rotation at `+0x00..+0x2C` to quaternion (passed as param_4).

---

## Helper Function Summary

| Address | Name | Purpose |
|---------|------|---------|
| 0x141e0c530 | GetHknpWorld | `physicsSystem->havokWorld->hknpWorld` (double deref: ptr+0x18 -> ptr+0x18) |
| 0x141e0c4e0 | GetHknpWorld_v2 | Same as above but identical implementation |
| 0x141e0c460 | bhkPhysicsSystem::GetBodyId | Maps body index to hknpBodyId from world mapping array |
| 0x141722c10 | RotationMatrixToQuaternion | Converts 3x3 rotation matrix to quaternion (Shepperd method) |
| 0x1401a8d60 | NiTransform::Multiply | Concatenates two 4x4 NiTransforms (parent * child = world) |
| 0x141df55f0 | SetBodyTransform_OrDefer | Teleport body; checks TLS+0x1528 for deferred mode |
| 0x141df56f0 | SetBodyVelocity_OrDefer | Set lin+ang velocity; checks TLS+0x1528 for deferred mode |
| 0x141df60c0 | WakeBody | Activates a body (signals constraint system via world+0x6D8) |
| 0x14153a6a0 | hknpWorld::computeHardKeyFrame | Core Havok function computing keyframe velocities |
| 0x141e07a90 | GetRigidBodyFromNode | Gets bhkRigidBody from NiAVObject collision object via vtable |
| 0x141e09530 | IsBodyValid | Checks body allocation/motion type bits |
| 0x141e09310 | IsBodyConsistent | Verifies quality entry maps back to correct bodyId |
| 0x141e07fa0 | GetWorldFromBody | Gets hknpWorld from bhkRigidBody's physics system |
| 0x141e07d80 | GetBodyEntry | Gets raw body array entry (world->bodyArray + bodyId*0x90) |
| 0x1415388f0 | LockMutex | Locks world mutex at world+0x690 |
| 0x141538a30 | UnlockMutex | Unlocks world mutex at world+0x690 |

## Constants

| Address | Value | Purpose |
|---------|-------|---------|
| 0x1438642b0 | {0.0, 0.0, 0.0, 0.0} | Zero vector (used for zeroing velocities on teleport) |
| 0x1438644d0 | small float | Angular velocity threshold below which angular velocity is zeroed |
| 0x1465a3d84 | float | Global physics deltaTime (seconds per frame) |
| 0x142dfba40 | {2.0, 2.0, 2.0, 2.0} | Newton-Raphson RCPPS refinement constant |
| 0x1438643b0 | {0.5, 0.5, 0.5, 0.5} | Quaternion rotation constant |
