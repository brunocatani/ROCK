# Ghidra RE: bhkNPCollisionObject Creation Pipeline (From Scratch)
**Date:** 2026-03-31
**Binary:** Fallout4VR.exe
**Auditor:** Ghidra MCP blind analysis

---

## Item 1: bhkNPCollisionObject Constructor (0x141e07710)

**What it is:** Constructor for bhkNPCollisionObject, the Bethesda wrapper around a Havok physics body.

### Signature
```cpp
bhkNPCollisionObject* bhkNPCollisionObject::ctor(
    bhkNPCollisionObject* this,  // RCX - pre-allocated memory
    uint32_t bodyIndex,          // EDX - index into physics system body array
    bhkPhysicsSystem* physSys    // R8  - the physics system owning this body
);
```

### Struct Size: 0x30 (48 bytes)
Allocation size confirmed from ALL callers using `FUN_141b91950(&DAT_14392e400, 0x30, 0)`.

### Struct Layout
```
Offset  Size  Field                      Description
------  ----  -----                      -----------
0x00    8     vtable*                    -> 0x142e835d8 (bhkNPCollisionObject vtable)
0x08    4     refCount (low16) / flags   hkReferencedObject::referenceCount + memSizeAndFlags
0x0A    2     memSizeAndFlags            hkReferencedObject high 16 bits
0x10    8     owner (NiAVObject*)        NiAVObject that owns this collision object (set by LinkObject)
0x18    2     flags                      Collision object flags (bit 6 cleared, bit 7 set by ctor)
0x20    8     physicsSystem*             bhkPhysicsSystem* (ref-counted, incremented in ctor)
0x28    4     bodyIndex                  Index into physicsSystem's body ID array
```

### Constructor Sequence
1. Calls parent ctor `FUN_142996d20` (NiCollisionObject base) which:
   - Calls `FUN_141c142f0` (bhkRefObject base) which:
     - Sets refCount to 0 at +0x08
     - Sets vtable to `PTR_FUN_142c83628` (hkReferencedObject)
     - Increments global NiObject count at `DAT_145c617b0`
     - Overwrites vtable with `PTR_FUN_142e574e8` (NiCollisionObject)
   - Sets `this+0x10` (owner) = 0
   - Sets vtable to `PTR__purecall_1430c5aa8` (abstract NiCollisionObject)
2. Clears flags at `this+0x18` to 0
3. Sets vtable to `PTR_FUN_142e835d8` (bhkNPCollisionObject final vtable)
4. Sets `this+0x20` = physicsSystem pointer
5. If physicsSystem != null: atomically increments refCount at physSys+0x08
6. Sets `this+0x28` = bodyIndex
7. Clears bit 6 of flags (`& 0xFFBF`)
8. Sets bit 7 of flags (`| 0x80`)

### Class Hierarchy
```
hkReferencedObject (vtable at PTR_FUN_142c83628)
  └─ NiObject (increments global count)
       └─ NiCollisionObject (vtable at PTR_FUN_142e574e8, owner at +0x10)
            └─ bhkNPCollisionObject (vtable at 0x142e835d8, physSys at +0x20, bodyIdx at +0x28)
                 └─ bhkNPCollisionObjectUnlinked (vtable at 0x142e838f8, same layout)
```

### Dependencies
- Pre-allocated 0x30 bytes from Bethesda allocator
- A bhkPhysicsSystem* (can be null initially, but must be set before CreateInstance)

---

## Item 2: bhkNPCollisionObject::CreateInstance (0x141e07ac0)

**What it is:** Instantiates the physics body in the Havok world. Takes the body description data from the physicsSystem's hknpPhysicsSystemData and creates an actual live body.

### Signature
```cpp
void bhkNPCollisionObject::CreateInstance(
    bhkNPCollisionObject* this,  // RCX
    bhkWorld* world              // RDX - param_2
);
```

### Preconditions
- `this+0x20` (physicsSystem) must be non-null
- `this+0x10` (owner NiAVObject) is read for initial transform (can be null = identity)
- `this+0x28` (bodyIndex) must be set

### Operation Sequence
1. **Guard check**: If `this+0x20` (physicsSystem) == 0, returns immediately
2. **Already-in-world check**: Calls `bhkPhysicsSystem::HasInstanceInWorld()` -- if already instantiated, skips to step 5
3. **Build initial transform**:
   - If `this+0x10` (NiAVObject owner) == null: uses identity/default transform from global data at `DAT_143864460`
   - If `this+0x10` != null: reads NiAVObject.world transform from owner+0x70..+0xAC:
     - Rotation matrix (3x4 floats at +0x70..+0x9C) masked with constants
     - Translation (4 floats at +0xA0..+0xAC) scaled by `DAT_1465a4ec0` constants (Bethesda→Havok scale)
     - Calls `FUN_1417215e0` to normalize/finalize the transform
4. **Creates instance**: Calls `bhkPhysicsSystem::CreateInstance(this+0x20, world, &transform)`
5. **Back-pointer setup**: Calls `FUN_141e07e30(this)` which:
   - Gets body ID via `bhkPhysicsSystem::GetBodyId(this+0x20, &out, this+0x28)`
   - Gets the hknpWorld from physicsSystem
   - Locks world mutex at world+0x690
   - Reads body array base at `world+0x20`
   - Computes body pointer: `bodyId * 0x90 + world+0x20`
   - **Sets `body+0x88 = this`** (back-pointer from Havok body to bhkNPCollisionObject)

### Key Insight: body+0x88
This is the critical back-pointer. When you have a raw Havok body pointer, `body+0x88` points back to the bhkNPCollisionObject that owns it. This is how Bethesda code resolves from physics collision results back to game objects.

The Getbhk function (0x141e07a10) confirms this:
```cpp
// Given hknpWorld* world and bodyId:
// body_ptr = bodyId * 0x90 + world->bodyArray (+0x20)
// return *(bhkNPCollisionObject**)(body_ptr + 0x88)
```

---

## Item 3: bhkPhysicsSystem::CreateInstance (0x141e0c320)

**What it is:** Creates the live Havok physics system instance from the template data (hknpPhysicsSystemData).

### Signature
```cpp
bool bhkPhysicsSystem::CreateInstance(bhkPhysicsSystem* this);
// Note: param_2 (world) and param_3 (transform) are passed from the caller
// but the decompiler shows them going to ChangeWorld when already instantiated
```

### bhkPhysicsSystem Layout (0x28 bytes)
```
Offset  Size  Field                          Description
------  ----  -----                          -----------
0x00    8     vtable*                        -> 0x142e83c88
0x08    4     refCount + memSizeAndFlags     hkReferencedObject
0x10    8     hknpPhysicsSystemData*         Template data (body cinfos, shapes, materials)
0x18    8     hknpPhysicsSystem* (instance)  Live instance (null before CreateInstance)
0x20    1     deactivated flag               Used by AddToWorld
```

### Operation Sequence
1. **Guard**: If `this+0x18` (instance) is already non-null AND `this+0x10` (data) exists:
   - Calls `bhkPhysicsSystem::ChangeWorld()` (remove from old world, re-create in new)
   - Returns 0
2. **If instance is null AND data exists**:
   - Calls virtual function at `vtable+0x160` on `this` (= `hknpPhysicsSystemData::createInstance()`)
   - This creates a live `hknpPhysicsSystem` from the template data
   - The result is a new `hknpPhysicsSystem*` (with body IDs, world pointer, etc.)
3. **Releases old instance** (if any) via atomic refcount decrement
4. **Stores new instance**: `this+0x18 = newInstance`
5. Returns 1 (success)

### Key: hknpPhysicsSystem Instance Layout (at this+0x18)
```
Offset  Size  Field
------  ----  -----
0x18    8     hknpWorld* (the world it belongs to)
0x20    8     bodyId array pointer (uint32_t[])
0x28    4     body count
```

This is confirmed by:
- `GetBodyId`: reads `*(uint32*)(instance+0x20 + index*4)`
- `HasInstanceInWorld`: checks `instance+0x18` == world+0x60
- `GetWorld` (FUN_141e0c4e0): returns `*(instance+0x18)`

---

## Item 4: bhkPhysicsSystem Constructor / hknpPhysicsSystemData

### bhkPhysicsSystem Constructor (0x141e0c2b0)
```cpp
bhkPhysicsSystem* bhkPhysicsSystem::ctor(
    bhkPhysicsSystem* this,              // RCX - pre-allocated 0x28 bytes
    hknpPhysicsSystemData* templateData  // RDX
);
```

**Sequence:**
1. Calls `FUN_141c142f0` (hkReferencedObject base ctor) - sets refCount, increments NiObject count
2. Sets vtable to `PTR_FUN_142e83c88` (bhkPhysicsSystem vtable)
3. Addref on templateData if non-null
4. `this+0x10` = templateData
5. `this+0x18` = 0 (no instance yet)
6. `this+0x20` = 0 (deactivated flag)

### hknpPhysicsSystemData Layout (0x78 bytes, from FUN_14005eab0)

This is a Havok container that stores body descriptions, material descriptions, and shape references. The constructor at `FUN_14005eab0` initializes it with 6 hkArray fields:

```
Offset  Size  Field                          Description
------  ----  -----                          -----------
0x00    8     vtable*                        -> PTR_FUN_142c83640
0x08    4     refCount + memSizeAndFlags     Initialized to 0xFFFF0001
0x10    8     bodyCinfos.data                hkArray<hknpBodyCinfo> data pointer
0x18    4     bodyCinfos.size                Current count
0x1C    4     bodyCinfos.capacityAndFlags    Capacity (low 30 bits) | flags (high 2 bits)
                                              Initialized with 0x80000000 (no-dealloc flag)
0x20    8     motionCinfos.data              hkArray<hknpMotionCinfo> data pointer
0x28    4     motionCinfos.size
0x2C    4     motionCinfos.capacityAndFlags
0x30    8     materialPalette.data           hkArray<material data>
0x38    4     materialPalette.size
0x3C    4     materialPalette.capacityAndFlags
0x40    8     bodyCinfoExtra.data            hkArray<extra body data> (stride 0x60)
0x48    4     bodyCinfoExtra.size
0x4C    4     bodyCinfoExtra.capacityAndFlags
0x50    8     contactData.data               hkArray<contact info>
0x58    4     contactData.size
0x5C    4     contactData.capacityAndFlags
0x60    8     shapeRefs.data                 hkArray<hkRefPtr<hknpShape>> (stride 8)
0x68    4     shapeRefs.size
0x6C    4     shapeRefs.capacityAndFlags
0x70    --    nameStringPtr                  hkStringPtr (via FUN_141536530)
```

### Can We Create One Without NIF Data? YES.

The phantom creation function at 0x140f0a340 proves this:
1. Creates hknpPhysicsSystemData via `FUN_14005eab0` (the raw constructor)
2. Manually populates the bodyCinfos array with `hknpBodyCinfo` structs
3. Manually populates the motionCinfos array
4. Manually adds shape references to the shapeRefs array
5. Wraps it in a bhkPhysicsSystem via `FUN_141e0c2b0`

---

## Item 5: bhkWorld::AddPhysicsSystem (0x141dfac30)

**What it is:** Registers a physics system with the Havok world, making its bodies active.

### Signature
```cpp
void bhkWorld::AddPhysicsSystem(
    bhkWorld* this,                  // RCX
    hknpPhysicsSystem* instance,     // RDX - the LIVE instance (bhkPhysicsSystem+0x18)
    bool deactivated                 // R8  - if true, bodies start deactivated
);
```

### Operation Sequence
1. Gets the hknpWorld from `this+0x60`
2. Checks thread-local state for physics threading mode:
   - Normal mode: locks the world via `FUN_141b93330(world+0x6D8)`
   - Threaded mode: skips lock
3. Calls `hknpPhysicsSystem::addToWorld(instance, worldStepId, !deactivated, deactivated)`
   - `worldStepId` comes from `this+0x15C`
4. Unlocks world mutex

### bhkPhysicsSystem::AddToWorld (0x141e0c580) - Higher-level wrapper
```cpp
bool bhkPhysicsSystem::AddToWorld(bhkPhysicsSystem* this);
```
- Gets instance at `this+0x18`
- Gets hknpWorld from `instance+0x18`
- Gets bhkWorld from `world+0x6D0`
- Calls `bhkWorld::AddPhysicsSystem(bhkWorld, instance, this+0x20)`

### State Before Calling
- The physics system must have a valid instance (`bhkPhysicsSystem+0x18 != null`)
- The instance must have an hknpWorld pointer set (`instance+0x18 != null`)
- Body IDs must be populated in the instance

---

## Item 6: Programmatic Creation - CreatePhantomBody (0x140f0a340)

**What it is:** Creates a complete bhkNPCollisionObject from scratch for a phantom/trigger volume. This is the GOLDEN REFERENCE for programmatic creation without NIF data.

### Complete Step-by-Step Sequence

#### Step 1: Create the Shape
```cpp
// FUN_1415ff4e0 creates an hknpShape (sphere in this case)
// Allocates 0x80 bytes via Havok TLS allocator
// param_1 = shape descriptor (position/radius data at DAT_1438642b0)
// param_2 = collision filter info (from DAT_1437ceeb0 * DAT_145b29178)
hknpShape* shape = FUN_1415ff4e0(&shapeDesc, collisionFilterInfo);
```

The shape creation function:
- Allocates 0x80 bytes from Havok TLS allocator: `TlsGetValue(DAT_145b63b20) -> vtable+8 call with size 0x80`
- Sets vtable to `PTR_FUN_142c92f38`
- Sets refCount = 0xFFFF0001
- Sets convex radius at +0x14
- Sets type/dispatch at +0x12 = 0x100
- Copies AABB extents from shape descriptor
- Marks as shared: sets bit 23 of refCount field (`| 0x800000`)

#### Step 2: Create hknpPhysicsSystemData
```cpp
// Allocate 0x78 bytes from Havok TLS allocator
hknpPhysicsSystemData* sysData = allocate(0x78);
FUN_14005eab0(sysData);  // Initialize all arrays to empty
```

#### Step 3: Populate Body Cinfo
```cpp
// Grow the bodyCinfos array if needed (via FUN_14155d820)
hknpBodyCinfo* cinfo = &sysData->bodyCinfos[sysData->bodyCinfos.size++];
hknpBodyCinfo__ctor(cinfo);  // Initialize at 0x141561dd0

cinfo->shape = shape;                    // +0x00
cinfo->bodyId = 0x7FFFFFFF;              // +0x08 (invalid = auto-assign)
cinfo->motionId = 0x7FFFFFFF;            // +0x0C (invalid = auto-assign)
cinfo->collisionFilterInfo = 5;          // +0x14 (motion type = KEYFRAMED phantom)
cinfo->materialId = sysData->materialPalette.size; // +0x12 (link to material entry)
```

#### Step 4: Populate Motion Cinfo
```cpp
// Grow motionCinfos array
hknpMotionCinfo* mcinfo = &sysData->motionCinfos[sysData->motionCinfos.size++];
FUN_141536cb0(mcinfo);  // Initialize motion cinfo (0x50 bytes)
// Sets default values: mass=1.0, angular damping, etc.
```

#### Step 5: Add Shape Reference
```cpp
// Grow shapeRefs array (stride 8, pointer array)
sysData->shapeRefs[sysData->shapeRefs.size++] = shape;
// Addref on shape
```

#### Step 6: Create bhkPhysicsSystem
```cpp
// Allocate 0x28 bytes from Bethesda allocator
// DAT_14392e400 = Bethesda memory pool
// DAT_14392e880 = initialization guard (must be == 2, or call FUN_141b91dd0 to init)
bhkPhysicsSystem* physSys = allocate(0x28);
FUN_141e0c2b0(physSys, sysData);  // bhkPhysicsSystem ctor
// Sets: vtable, refcount, data=sysData, instance=null, deactivated=0
```

#### Step 7: Create bhkNPCollisionObject
```cpp
// Allocate 0x30 bytes from Bethesda allocator
// Set TLS alloc type to 0x41 first:
//   *(uint32*)(TLS + 0x9C0) = 0x41;
bhkNPCollisionObject* collObj = allocate(0x30);
bhkNPCollisionObject__ctor(collObj, 0, physSys);  // bodyIndex=0
// Restores TLS alloc type after
```

#### Step 8: Set Motion Type
```cpp
bhkNPCollisionObject__SetMotionType(collObj, 2);  // 2 = KEYFRAMED
// Motion types: 0=STATIC, 1=DYNAMIC, 2=KEYFRAMED
```

#### Step 9: Link to NiAVObject
```cpp
// vtable slot 0x150 = LinkObject (FUN_142996cb0)
// This is the function that binds the collision object to a NiAVObject
collObj->vtable->LinkObject(collObj, niAVObject);
```

The LinkObject function (at 0x142996cb0):
1. Sets `collObj+0x10 = niAVObject` (owner back-pointer)
2. If niAVObject != null:
   - Reads old collision object from `niAVObject+0x100`
   - If different from this: increments this->refCount, stores `niAVObject+0x100 = this`, decrements old->refCount
   - If new collision object's owner (`collObj+0x10`) != niAVObject: recursively calls LinkObject

---

## hknpBodyCinfo Layout (0x60 bytes, ctor at 0x141561dd0)
```
Offset  Size  Field                  Description
------  ----  -----                  -----------
0x00    8     shape*                 hknpShape* (hkRefPtr)
0x08    4     bodyId                 0x7FFFFFFF = auto-assign
0x0C    4     motionId               0x7FFFFFFF = auto-assign
0x10    1     motionPropertiesId     0xFF = default
0x12    2     materialId             Index into material palette
0x14    4     collisionFilterInfo    Collision filter bits
0x18    4     (unused/padding)
0x1C    4     collisionLookAheadDist
0x20    48    transform              hkTransformf (3x4 rotation + translation)
0x28    8     userData               (at +0x28 within cinfo)
0x40    16    position               hkVector4f (from transform row 3)
0x50    8     flags
0x58    8     shape (hkRefPtr copy)  Duplicate shape pointer with ref management
```

---

## hknpMotionCinfo Layout (0x50 bytes, from FUN_141536cb0)
```
Offset  Size  Field                  Description
------  ----  -----                  -----------
0x00    8     (zeroed)
0x10    1     motionType
0x11    --    (encoded params via FUN_141725980)
0x12    2     linearDamping          half-float, default 0x3F00
0x14    2     angularDamping         half-float, default 0x3F00
0x18    2     flags                  default 0x0201
0x1A    2     timeFactor             bfloat16
0x1C    4     maxLinearVelocity      default 0x7F7FFFEE (~FLT_MAX)
0x20    4     mass                   default 0x3F800000 (1.0f)
0x24    1     (flag)
0x26    2     gravityFactor          half-float, default 0x3F80
0x38    2     (additional param)     bfloat16
```

---

## COMPLETE RECIPE: Creating bhkNPCollisionObject From Scratch

### Prerequisites
- A valid hknpShape* (sphere, box, convex hull, etc.)
- A target NiAVObject* to attach to (optional but recommended)
- Access to bhkWorld* (for world registration)

### Step-by-Step Code

```cpp
// ============================================================
// STEP 1: Allocator setup
// ============================================================
// Bethesda allocator at DAT_14392e400
// Guard at DAT_14392e880 - must be initialized (== 2)
// Call FUN_141b91dd0(&DAT_14392e400, &DAT_14392e880) if != 2
// Allocator function: FUN_141b91950(&DAT_14392e400, size, alignment)

// TLS alloc type at TLS+0x9C0 should be set to 0x41 for collision objects
// (phantom creation saves/restores this value)

// ============================================================
// STEP 2: Create hknpPhysicsSystemData (template)
// ============================================================
// Havok TLS allocator: TlsGetValue(DAT_145b63b20)->vtable[1](size)
void* sysDataMem = havokAlloc(0x78);
auto* sysData = FUN_14005eab0(sysDataMem);  // 0x14005eab0

// ============================================================
// STEP 3: Create shape (e.g., sphere)
// ============================================================
// For sphere: FUN_1415ff4e0(&sphereDesc, collisionFilterInfo)
// sphereDesc = {float radius} scaled appropriately
// The shape is allocated at 0x80 bytes from Havok TLS allocator
auto* shape = createShape(...);

// ============================================================
// STEP 4: Add body cinfo to systemData
// ============================================================
// Grow bodyCinfos array if full:
//   FUN_14155d820(&PTR_PTR_FUN_143866310, &sysData->bodyCinfos, 0x60)
auto* cinfo = &sysData->bodyCinfos[sysData->bodyCinfoCount++];
hknpBodyCinfo__ctor(cinfo);  // 0x141561dd0

cinfo->shape = shape;            // +0x00
cinfo->bodyId = 0x7FFFFFFF;      // +0x08 (auto-assign)
cinfo->motionId = 0x7FFFFFFF;    // +0x0C (auto-assign)
cinfo->collisionFilterInfo = 5;  // +0x14 (KEYFRAMED for phantom)
cinfo->materialId = 0;           // +0x12

// ============================================================
// STEP 5: Add motion cinfo
// ============================================================
// Grow motionCinfos if full
auto* mcinfo = &sysData->motionCinfos[sysData->motionCinfoCount++];
FUN_141536cb0(mcinfo);  // 0x141536cb0 - default motion init

// ============================================================
// STEP 6: Add shape reference
// ============================================================
// Grow shapeRefs array if full
sysData->shapeRefs[sysData->shapeRefCount++] = shape;
// addref shape

// ============================================================
// STEP 7: Create bhkPhysicsSystem
// ============================================================
void* physSysMem = FUN_141b91950(&bethesdaAlloc, 0x28, 0);
auto* physSys = FUN_141e0c2b0(physSysMem, sysData);  // 0x141e0c2b0
// Now: physSys+0x10 = sysData, physSys+0x18 = null (no instance yet)

// ============================================================
// STEP 8: Create bhkNPCollisionObject
// ============================================================
// Save TLS alloc type, set to 0x41
uint32_t oldAllocType = *(uint32_t*)(TLS + 0x9C0);
*(uint32_t*)(TLS + 0x9C0) = 0x41;

void* collObjMem = FUN_141b91950(&bethesdaAlloc, 0x30, 0);
auto* collObj = bhkNPCollisionObject__ctor(collObjMem, 0, physSys);  // 0x141e07710
// bodyIndex = 0 (first body in the system)

// ============================================================
// STEP 9: Set motion type
// ============================================================
bhkNPCollisionObject__SetMotionType(collObj, 2);  // 0x141e07300
// 0 = STATIC, 1 = DYNAMIC, 2 = KEYFRAMED

// Restore TLS alloc type
*(uint32_t*)(TLS + 0x9C0) = oldAllocType;

// ============================================================
// STEP 10: Link to NiAVObject (CRITICAL)
// ============================================================
// vtable slot 0x150 = LinkObject = FUN_142996cb0
// This sets:
//   collObj+0x10 = niAVObject (owner)
//   niAVObject+0x100 = collObj (collision object pointer)
collObj->LinkObject(collObj, targetNiAVObject);

// ============================================================
// STEP 11: Create instance in world
// ============================================================
// This calls bhkPhysicsSystem::CreateInstance which:
// 1. Calls hknpPhysicsSystemData::createInstance() (vtable+0x160)
// 2. Stores live instance at physSys+0x18
// 3. Sets body+0x88 back-pointer to collObj
collObj->CreateInstance(bhkWorldPtr);  // 0x141e07ac0
// OR manually:
//   bhkPhysicsSystem::CreateInstance(physSys, world, &initialTransform)
//   Then manually set body+0x88

// ============================================================
// STEP 12: Register with world (if not done by CreateInstance)
// ============================================================
// bhkPhysicsSystem::AddToWorld (0x141e0c580) handles this
// OR directly: bhkWorld::AddPhysicsSystem(bhkWorld, physSys->instance, deactivated)
```

### Critical Back-Pointers (ALL MUST BE SET)
1. `bhkNPCollisionObject+0x10` = NiAVObject* owner (set by LinkObject)
2. `bhkNPCollisionObject+0x20` = bhkPhysicsSystem* (set by ctor)
3. `NiAVObject+0x100` = bhkNPCollisionObject* (set by LinkObject)
4. `havokBody+0x88` = bhkNPCollisionObject* (set by CreateInstance back-pointer step)
5. `hknpPhysicsSystem+0x18` = hknpWorld* (set during addToWorld)

### Allocator Details
- **Bethesda allocator**: Singleton at `DAT_14392e400`, initialized via guard `DAT_14392e880`
  - `FUN_141b91950(allocator, size, alignment)` for allocation
  - `FUN_141b91dd0(allocator, guard)` for one-time init
- **Havok TLS allocator**: Via `TlsGetValue(DAT_145b63b20)` -> call vtable[1] with size
- **TLS alloc type**: At `TLS+0x9C0`, set to `0x41` for collision object allocations, `0x37` for general bhk objects

### Motion Type Constants
- 0 = STATIC (immovable, no velocity)
- 1 = DYNAMIC (fully simulated, affected by forces)
- 2 = KEYFRAMED (driven by code, pushes dynamic bodies)

### Key Function Addresses Summary
```
0x141e07710  bhkNPCollisionObject::ctor(this, bodyIndex, physSys)
0x141e07ac0  bhkNPCollisionObject::CreateInstance(this, world)
0x141e07300  bhkNPCollisionObject::SetMotionType(this, type)
0x141e07780  bhkNPCollisionObject::~dtor (clears body+0x88, releases physSys)
0x141e07e30  bhkNPCollisionObject::GetBodyPtr (resolves bodyId -> body array address)
0x141e07d80  bhkNPCollisionObject::GetBodyPtrLocked (same with mutex)
0x141e07a10  bhkNPCollisionObject::Getbhk (world, bodyId -> body+0x88 backptr)
0x141e07f30  bhkNPCollisionObject::GetShape (body+0x48 = shape ptr)

0x141e0c2b0  bhkPhysicsSystem::ctor(this, hknpPhysicsSystemData)
0x141e0c320  bhkPhysicsSystem::CreateInstance(this)
0x141e0c430  bhkPhysicsSystem::HasInstanceInWorld(this, world)
0x141e0c460  bhkPhysicsSystem::GetBodyId(this, out, index)
0x141e0c580  bhkPhysicsSystem::AddToWorld(this)
0x141e0c620  bhkPhysicsSystem::ChangeWorld(this, world, transform)

0x141dfac30  bhkWorld::AddPhysicsSystem(this, instance, deactivated)
0x141dfad00  bhkWorld::RemovePhysicsSystem(this, instance)

0x14005eab0  hknpPhysicsSystemData::ctor(this) -- initializes empty arrays
0x141561dd0  hknpBodyCinfo::ctor(this) -- 0x60 bytes
0x141536cb0  hknpMotionCinfo::init(this) -- 0x50 bytes

0x142996cb0  NiCollisionObject::LinkObject(this, niAVObject) -- vtable slot 0x150
0x1415ff4e0  CreateSphereShape(shapeDesc, filterInfo)
0x140f0a340  TESObjectREFR::CreatePhantomBody -- GOLDEN REFERENCE for from-scratch creation

0x14392e400  Bethesda physics allocator (singleton)
0x14392e880  Allocator init guard
0x145b63b20  Havok TLS allocator key (TlsGetValue)
```
