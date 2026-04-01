# Ghidra RE: Tree-Level Motion & Collision Functions (2026-03-31)

Blind reverse engineering of bhkWorld tree-walking functions for ROCK grab system.

---

## 0x141df95b0 — bhkWorld::SetMotion

### What it is
Sets the motion type on ALL bodies in a NiAVObject tree by walking the scene graph
recursively and dispatching a command (ID=5) to each collision object's physics body.

### Exact Signature (from assembly)
```cpp
// RCX = NiAVObject* root       (the scene graph root to walk)
// EDX = uint32_t motionType    (hknpMotionPropertiesId::Preset value)
// R8B = bool activate          (whether to activate the body)
// R9B = bool forceUpdate       (if true, skip the "is physics initialized" check)
// [RSP+0x28] = byte recurse   (if nonzero, recurse into NiNode children)
typedef bool (*bhkWorld_SetMotion_t)(
    NiAVObject* root,           // RCX
    uint32_t motionType,        // EDX (0=DYNAMIC, 1=KEYFRAMED, 2=FIXED/STATIC, etc.)
    bool activate,              // R8B
    bool forceUpdate,           // R9B (skip bhkWorld init check)
    uint8_t recurse             // stack [RSP+0x28] -> stored at cmdStruct+0x18 (the +0x38 byte in walker)
);
```

### Command Struct Layout (stack-allocated)
```
Offset  Size  Field           SetMotion Value
+0x00   8     padding/ptr     0
+0x08   4     commandId       5
+0x10   4     motionType      param_2 (EDX)
+0x18   4     recurse         param_5 (stack byte, cast to uint)
+0x38   1     (unused here)   -
+0x58   1     activate        param_3 (R8B)
```

### Flow
1. Null-check on param_1 (NiAVObject root)
2. If !forceUpdate: calls `FUN_141d2dd80(root)` to get bhkWorld, checks `(bhkWorld+0x18) & 2` (physics initialized flag)
3. Loads function pointer from global `[0x1438ae888]` -> points to `0x141dfa5d0` (per-body SetMotion callback)
4. Calls `FUN_141dfa2b0(root, &cmdStruct, callback)` — the recursive tree walker
5. Returns true (1) on success, false (0) if null or physics not init

### Key Evidence
- `MOV dword ptr [RSP + 0x28],0x5` — command ID 5 = SetMotion
- `MOV R8,qword ptr [0x1438ae888]` — loads callback from global table
- `CALL 0x141dfa2b0` — calls the recursive tree walker
- The 5th parameter (stack byte at RSP+0x90 in caller frame) is the "recurse into children" flag

---

## 0x141df9940 — bhkWorld::EnableCollision

### What it is
Enables or disables collision on ALL bodies in a NiAVObject tree by walking the scene
graph recursively and dispatching a command (ID=9) to each collision object.

### Exact Signature (from assembly)
```cpp
// RCX = NiAVObject* root       (scene graph root to walk)
// DL  = byte enable            (nonzero = enable, 0 = disable)
// R8B = bool activate          (whether to activate the body after change)
// R9B = bool forceUpdate       (if true, skip bhkWorld init check)
typedef bool (*bhkWorld_EnableCollision_t)(
    NiAVObject* root,           // RCX
    uint8_t enable,             // DL (0 = disable collision, nonzero = enable)
    bool activate,              // R8B
    bool forceUpdate            // R9B
);
```

### Command Struct Layout
```
Offset  Size  Field           EnableCollision Value
+0x00   8     padding/ptr     0
+0x08   4     commandId       9
+0x10   4     enable          param_2 (DL, zero-extended)
+0x58   1     activate        param_3 (R8B)
```

### Flow
1. Null-check on param_1 (NiAVObject root)
2. If !forceUpdate: calls `FUN_141d2dd80(root)` for bhkWorld init check
3. Loads callback from global `[0x1438ae8a8]` -> points to `0x141dfa6b0` (per-body collision toggle callback)
4. Calls `FUN_141dfa2b0(root, &cmdStruct, callback)` — recursive tree walker
5. Returns true/false

### Per-Body Callback Behavior (0x141dfa6b0)
The callback at `FUN_141dfa630` (nearby, same pattern):
- Calls `FUN_141e095d0` which reads `bodyArray[bodyId * 0x90 + 0x40]`, checks bit 3
- This is the Havok body `isActive` / collision-enabled flag
- If enabling: increments a counter at cmdStruct+0x10, sets cmdStruct+0x38 = 0

### Key Evidence
- `MOV dword ptr [RSP + 0x28],0x9` — command ID 9 = EnableCollision
- `MOVZX ESI,DL` — enable flag from DL register (byte)
- Same tree-walker pattern as SetMotion
- NOT deferred/async — executes immediately via tree walker

---

## 0x141dfa2b0 — Recursive NiNode Tree Walker

### What it is
The core recursive function that walks a NiAVObject scene graph tree, calling a
function pointer callback on each node's collision object. Used by SetMotion,
EnableCollision, and at least 8 other bhkWorld operations.

### Exact Signature
```cpp
// RCX = NiAVObject* node        (current node being visited)
// RDX = void* commandStruct     (stack-allocated command data from caller)
// R8  = code* callback          (function pointer: void callback(bhkCollisionObject*, commandStruct*))
typedef void (*TreeWalker_t)(
    NiAVObject* node,           // RCX
    void* commandStruct,        // RDX
    void (*callback)(void* collisionObject, void* commandStruct)  // R8
);
```

### Tree Traversal Algorithm
```
TreeWalker(node, cmd, callback):
    if node == null: return

    collisionObject = GetCollisionObject(node)  // FUN_141e07a90

    if cmd->commandId == 0:
        // Special case: clear flag 0x10, set flags 0x6 on node->flags (+0x108)
        node->flags = (node->flags & ~0x10) | 0x6

    if collisionObject != null:
        callback(collisionObject, cmd)           // CALL RBP

    if cmd->recurse != 0:                        // byte at cmd+0x38
        niNode = node->IsNode()                  // vtable[0x20/8 = slot 4]
        if niNode != null:
            for i in 0..niNode->childCount:
                TreeWalker(niNode->children[i], cmd, callback)
```

### VR-Specific NiNode Offsets (CONFIRMED)
- `node->vtable[4]` (offset +0x20 in vtable): IsNode() — returns NiNode* or null
- `niNode + 0x168`: children data pointer (NiTArray<NiAVObject*>::data)
- `niNode + 0x172`: child count (uint16_t) — NOTE: 0x172, not 0x174 as some docs say
- Children accessed as: `*(children_data + i * 8)` — array of pointers

### GetCollisionObject (FUN_141e07a90)
```cpp
// Gets bhkNPCollisionObject from NiAVObject
bhkNPCollisionObject* GetCollisionObject(NiAVObject* node) {
    if (node && node->collisionObject != null) {  // +0x100
        // Calls virtual function at collisionObject->vtable[0xD8/8]
        return node->collisionObject->vfunc_0xD8();
    }
    return null;
}
```
- `NiAVObject + 0x100` = collisionObject pointer (NiCollisionObject*)
- Virtual call at vtable offset 0xD8 = slot 27: likely GetAsBhkNPCollisionObject()

### Key Evidence
- `CMP dword ptr [RSI + 0x8],0x0` — checks commandId at cmd+0x08
- `MOV R9,qword ptr [RBX + 0x108]` and `AND R9,-0x11; OR R9,0x6` — flag manipulation at NiAVObject+0x108
- `CALL qword ptr [RAX + 0x20]` — vtable slot 4 = IsNode
- `CMP BX,word ptr [RAX + 0x172]` — child count at NiNode+0x172
- `MOV RCX,qword ptr [RCX + RBX*0x8]` at `[RDI + 0x168]` — children array
- `CMP byte ptr [RSI + 0x38],0x0` — recurse flag at cmd+0x38
- Recursive self-call: `CALL 0x141dfa2b0`

---

## 0x141e09170 — IsBodyConstrained

### What it is
Checks whether the body referenced by a bhkNPCollisionObject is constrained
(i.e., has an active constraint in the physics system).

### Exact Signature
```cpp
// RCX = bhkNPCollisionObject*
typedef bool (*IsBodyConstrained_t)(bhkNPCollisionObject* collObj);
```

### Algorithm
```cpp
bool IsBodyConstrained(bhkNPCollisionObject* collObj) {
    bhkPhysicsSystem* physSys = collObj->physicsSystem;  // +0x20
    if (!physSys) return false;

    uint32_t constraintCount = GetConstraintCount(physSys);  // FUN_141e0c7a0
    uint32_t targetBodyIndex = collObj->bodyIndex;           // +0x28 (uint32_t)

    for (uint32_t i = 0; i < constraintCount; i++) {
        ConstraintEntry* entry = GetConstraintEntry(physSys, i);  // FUN_141e0c760
        // entry = physSys->internalData->constraintArray + i * 0x18
        if (entry->bodyIdField == targetBodyIndex) {  // entry+0x08
            return true;
        }
    }
    return false;
}
```

### bhkPhysicsSystem Constraint Access
- `GetConstraintCount(physSys)`: reads `physSys->internalData->field_0x58` where internalData = physSys+0x10
  - Full path: `*(uint32_t*)(*(physSys+0x10) + 0x58)`
- `GetConstraintEntry(physSys, i)`: returns `*(physSys+0x10)->field_0x50 + i * 0x18`
  - Full path: `*(*(physSys+0x10) + 0x50) + i * 0x18`
  - Each constraint entry is 0x18 bytes, with bodyId at entry+0x08

### What it works with
- Takes a `bhkNPCollisionObject*`, NOT a raw hknpBodyId
- Reads `collObj+0x28` as the body index to search for — this is the collision object's body index into the physics system, NOT the raw hknpBodyId
- Iterates the physics system's constraint list (from `collObj+0x20`, the bhkPhysicsSystem*)
- Compares constraint entry body IDs (at entry+0x08) against the collision object's body index

### Key Evidence
- `MOV RCX,qword ptr [RCX + 0x20]` — gets physicsSystem from collObj+0x20
- `MOV ECX,dword ptr [RSI + 0x28]` — reads body index from collObj+0x28
- `CMP dword ptr [RAX + 0x8],ECX` — compares constraint bodyId with target
- Constraint entry stride: 0x18 (from `FUN_141e0c760`: `param_2 * 0x18`)

---

## Command ID Table (bhkWorld tree operations)

| ID | Function Address | Operation |
|----|-----------------|-----------|
| 0  | 0x141df9170     | SetMotionAndActivate (complex, 7 params) |
| 1  | 0x141df9480     | SetCollisionLayer? (simple, callback via 0x1438ae868) |
| 2  | 0x141df9650     | ??? (callback via 0x1438ae870) |
| 3  | 0x141df96d0     | ??? (callback via 0x1438ae878) |
| 5  | 0x141df95b0     | SetMotion |
| 8  | 0x141df9810     | ??? (callback via 0x1438ae8a0) |
| 9  | 0x141df9940     | EnableCollision |

---

## Answers to ROCK Integration Questions

### Q1: Can bhkWorld::SetMotion (0x141df95b0) be called on a grabbed object's NiAVObject root to set ALL bodies to DYNAMIC?

**YES.** This is exactly what it does. The call would be:

```cpp
// Get the NiAVObject root from TESObjectREFR
NiAVObject* root = ref->GetCurrent3D();  // or specific node

// Call SetMotion to set all bodies to DYNAMIC
// motionType 0 = DYNAMIC (based on Havok hknpMotionPropertiesId::Preset)
// activate = true, forceUpdate = true, recurse = true
typedef bool (*SetMotion_t)(NiAVObject*, uint32_t, bool, bool, uint8_t);
auto SetMotion = (SetMotion_t)0x141df95b0;
SetMotion(root, 0 /*DYNAMIC*/, true /*activate*/, true /*force*/, 1 /*recurse*/);

// To set back to KEYFRAMED when releasing:
SetMotion(root, 1 /*KEYFRAMED*/, true, true, 1);
```

**IMPORTANT:** The 5th parameter (recurse) MUST be 1 if the object has multiple physics
bodies in its tree (e.g., multi-part objects, ragdolls). If 0, only the root node's
collision object is processed.

### Q2: Can bhkWorld::EnableCollision (0x141df9940) be used to disable collision on ROCK's hand body?

**IT DEPENDS on what "hand body" is.** This function requires a NiAVObject* root,
NOT a raw body ID or collision object. It works by:
1. Walking the NiAVObject scene graph tree
2. Getting the collision object from each NiAVObject node
3. Toggling collision on each body found

For ROCK's hand body:
- If the hand body is attached to a NiAVObject in the scene graph (has a collision object
  at node+0x100), then YES — pass that NiAVObject.
- If the hand body is a standalone Havok body NOT attached to any NiAVObject, then NO —
  you need to call the Havok-level function directly (hknpWorld::setBodyCollisionEnabled
  or similar).

**For disabling collision on a grabbed object (e.g., to prevent it colliding with
the hand while held):**
```cpp
// Disable collision on the grabbed object's tree
typedef bool (*EnableCollision_t)(NiAVObject*, uint8_t, bool, bool);
auto EnableCollision = (EnableCollision_t)0x141df9940;
EnableCollision(grabbedRoot, 0 /*disable*/, true /*activate*/, true /*force*/);
```

### Q3: Which functions are directly usable for ROCK's grab system?

| Function | Directly Usable? | Notes |
|----------|-----------------|-------|
| SetMotion (0x141df95b0) | **YES** | Perfect for switching grabbed objects between DYNAMIC/KEYFRAMED/FIXED. Pass TESObjectREFR's NiAVObject root with recurse=1. |
| EnableCollision (0x141df9940) | **YES for objects with NiAVObject** | Use to disable collision on grabbed objects. NOT usable for standalone Havok bodies (like ROCK's hand collider if it's not in the scene graph). |
| TreeWalker (0x141dfa2b0) | **YES but indirect** | Can be called directly with custom callbacks if you need tree-level operations beyond the predefined commands. |
| IsBodyConstrained (0x141e09170) | **YES** | Takes bhkNPCollisionObject*, checks if any constraint references its body. Useful for detecting if an object is already constrained before grabbing. |

### Adaptation Needed for ROCK Hand Body
ROCK's hand collider (layer 43) is likely a standalone body not in the scene graph.
For this body, collision toggle must use direct Havok body manipulation:
- Read body flags from `hknpWorld+0x20 + bodyId*0x90 + 0x40`
- Toggle bit 3 for collision enable/disable
- Or use the deferred command system if available

The scene-graph-based functions (SetMotion, EnableCollision) are ideal for **grabbed
objects** which always have a TESObjectREFR and thus a NiAVObject tree.
