# Skinned Mesh Data Access Structures for BSGeometry in FO4VR

**Date:** 2026-03-30
**Binary:** Fallout4VR.exe
**Method:** Blind Ghidra reverse engineering via MCP
**Base address:** 0x140000000 (ImageBase)

---

## Item 1: skinInstance pointer and BSSkin::Instance layout

### CRITICAL DISCOVERY: FO4 uses BSSkin::Instance, NOT NiSkinInstance

Fallout 4 does NOT use NiSkinInstance for BSTriShape. It uses a completely different class hierarchy:
- `BSSkin::Instance` (0xC0 bytes) replaces `NiSkinInstance` (0x98 bytes)
- `BSSkin::BoneData` (0x28 bytes) replaces `NiSkinData` (0x60 bytes)
- `NiSkinPartition` still exists but is NOT used by BSTriShape; it's legacy for NiTriShape

### skinInstance pointer location

**BSTriShape offset 0x180** contains the `BSSkin::Instance*` pointer.

This is NOT the same as the BSGeometry-level `NiSkinInstance*` at offset 0x178 (legacy field for old NiGeometry format). BSTriShape in FO4 uses offset 0x180 exclusively.

**Evidence:** Assembly at 0x141cdaa10 (GetSkinnedVertex):
```asm
141cdaa25: MOV RBP, qword ptr [RCX + 0x180]   ; RBP = BSSkin::Instance*
```
And at 0x141db2be0 (ApplySkinningToGeometry):
```c
lVar20 = param_1[0x30];  // 0x30 * 8 = 0x180 = BSSkin::Instance*
```
And at 0x141db3f90 (MergeSkinInstances):
```c
lVar4 = *(longlong *)(param_2 + 0x180);  // BSSkin::Instance*
```

### BSSkin::Instance struct layout (0xC0 bytes)

| Offset | Size | Type | Field | Evidence |
|--------|------|------|-------|----------|
| 0x00 | 8 | ptr | vtable | `0x142e59a48` (ctor at 0x141c33670) |
| 0x08 | 4 | uint32 | refCount | NiObject base |
| 0x10 | 16 | NiTArray | bones (NiNode**) | Stream read at 0x141c34070 stores bone nodes here; resized via FUN_141c358d0 |
| 0x20 | 8 | -- | bones capacity/metadata | Part of NiTArray at 0x10 |
| 0x28 | 16 | NiTArray | boneWorldTransforms (NiTransform**) | Stream read: `bone + 0x70` stored here |
| 0x38 | 4 | uint32 | boneCount | Used as loop bound in ApplySkinning: `*(uint*)(lVar20 + 0x38)` |
| 0x3C | 4 | uint32 | (unknown/padding) | Set to 0 in ctor |
| 0x40 | 8 | ptr | BSSkin::BoneData* | Ref-counted. Confirmed: `*(lVar4 + 0x40)` in GetSkinnedVertex & MergeSkin |
| 0x48 | 8 | ptr | rootNode (NiAVObject*) | Stream read at 0x141c34070 resolves this |
| 0x50 | 8 | ptr | boneWorldTransformBuffer | Raw alloc, each entry 0x10 bytes; freed in dtor |
| 0x58 | 4 | uint32 | (unknown) | Set to 0 in ctor |
| 0x5C | 4 | uint32 | (unknown) | Set to 0 in ctor |
| 0x60-0x9C | 64 | NiTransform | skinToRootTransform | 3x3 rotation (0x60-0x83) + translation (0x84-0x8F) + padding + scale at 0x9C = 1.0f |
| 0xA0 | 8 | ptr | (data buffer 1) | Released via FUN_141ba3100 in dtor |
| 0xA8 | 8 | ptr | (data buffer 2) | Released via FUN_141ba3100 in dtor |
| 0xB0-0xB7 | 8 | -- | (unknown) | |
| 0xB8 | 4 | uint32 | (unknown) | Set to 0 in ctor |
| 0xBC | 4 | int32 | (sentinel) | Set to 0xFFFFFFFF in ctor |

**Key functions:**
| Address | Name |
|---------|------|
| 0x141c33670 | BSSkin::Instance constructor |
| 0x141c33740 | BSSkin::Instance destructor |
| 0x141c335e0 | BSSkin::Instance factory (NIF alloc, 0xC0 bytes) |
| 0x141c33b60 | BSSkin::Instance NIF stream create + load |
| 0x141c33050 | BSSkin::Instance LoadBinary (fields) |
| 0x141c34070 | BSSkin::Instance LoadBinary (bone linkage) |
| 0x141c33f50 | BSSkin::Instance::SetBoneWorldTransformBuffer |

**Pointer chain from BSTriShape to BSSkin::Instance:**
```
BSTriShape* triShape;
BSSkin::Instance* skinInst = *(BSSkin::Instance**)(triShape + 0x180);  // offset 0x180
```

---

## Item 2: BSSkin::BoneData layout (replaces NiSkinData and NiSkinPartition for BSTriShape)

### BSSkin::BoneData struct layout (0x28 bytes)

| Offset | Size | Type | Field | Evidence |
|--------|------|------|-------|----------|
| 0x00 | 8 | ptr | vtable | `0x142e598e8` (ctor at 0x141c32f30, 0x141c35140) |
| 0x08 | 4 | uint32 | refCount | NiObject base |
| 0x10 | 16 | AlignedArray | skinToBone transforms | Entries of 0x50 bytes each, aligned to 0x10. Initialized via FUN_141c357b0 |
| 0x20 | 8 | -- | (secondary container) | Initialized via FUN_141b93680 |

### Per-bone transform entry (0x50 bytes = 80 bytes)

Each entry in the `BSSkin::BoneData` transform array at offset 0x10 is 0x50 bytes:

| Sub-offset | Size | Type | Field | Evidence |
|------------|------|------|-------|----------|
| 0x00 | 12 | float[3] | boundingSphere center | Initialized to (0,0,0) in FUN_141c357b0 |
| 0x0C | 4 | uint32 | (padding/flags) | Set to 0 |
| 0x10 | 48 | float[12] | 3x4 skinToBone matrix (rotation 3x3 + translation) | Copied as 4x float4 blocks in MergeSkin |
| 0x40 | 16 | float[4] | scale/extra data | Includes scale at 0x4C = 1.0f |

**Evidence from MergeSkin (0x141db3f90):**
```c
lVar9 = *(longlong *)(*(longlong *)(local_78 + 0x40) + 0x10);  // boneData->transforms.data
// Then copies 0x50 bytes per bone:
// lVar9 + boneIdx * 0x50 + 0x00..0x0C  (bounding sphere)
// lVar9 + boneIdx * 0x50 + 0x10..0x2C  (rotation rows)
// lVar9 + boneIdx * 0x50 + 0x30..0x3C  (translation + padding)
// lVar9 + boneIdx * 0x50 + 0x40..0x4C  (scale + extra)
```

**Evidence from GetSkinnedVertex (0x141cdaa10):**
```asm
141cdabfe: MOV RAX, qword ptr [RBP + 0x40]        ; BSSkin::Instance->boneData
141cdac0b: MOV RAX, qword ptr [RAX + 0x10]         ; boneData->transforms.data
141cdac0f: SHL R8, 0x4                              ; R8 = boneIdx * 0x10 (was boneIdx * 5 * 16)
141cdac13: ADD RAX, 0x10                            ; skip first 0x10 bytes of each entry
141cdac17: ADD R8, RAX                              ; skinToBone = data + 0x10 + boneIdx * 0x50
```

### NiSkinPartition (legacy, for NiTriShape only)

NiSkinPartition still exists in FO4 but is NOT used for BSTriShape meshes. It's only used for legacy NiTriShape.

**NiSkinPartition struct (0x30 bytes):**
| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0x00 | 8 | ptr | vtable (`0x142e5daa8`) |
| 0x08 | 4 | uint32 | refCount |
| 0x10 | 4 | uint32 | numPartitions |
| 0x18 | 8 | ptr | partitions array |
| 0x20 | 8 | ptr | (unknown) |

Each partition is 0x58 bytes with per-partition vertex data.

**Key functions:**
| Address | Name |
|---------|------|
| 0x141c32f30 | BSSkin::BoneData factory |
| 0x141c35140 | BSSkin::BoneData constructor (basic) |
| 0x141c350e0 | BSSkin::BoneData constructor (with count) |
| 0x141c357b0 | BSSkin::BoneData::AllocateTransforms |

**Pointer chain from BSTriShape to bone transforms:**
```
BSTriShape* triShape;
BSSkin::Instance* skinInst = *(BSSkin::Instance**)(triShape + 0x180);
BSSkin::BoneData* boneData = *(BSSkin::BoneData**)(skinInst + 0x40);
void* transformData = *(void**)(boneData + 0x10);  // aligned array data pointer
// Per-bone skinToBone matrix at: transformData + boneIdx * 0x50 + 0x10
// Per-bone bounding sphere at:   transformData + boneIdx * 0x50 + 0x00
```

---

## Item 3: Bone transform access and skinning formula

### GetSkinnedVertex (0x141cdaa10)

**Signature:** `void GetSkinnedVertex(BSTriShape* shape, int vertexIndex, float* outPos, float* outNormal)`

**Algorithm:**
1. Get BSSkin::Instance from shape+0x180
2. Get shapeData via property lookup (FUN_141c16e20 with DAT_145c629c0)
3. Read half-float vertex position from `shapeData + 0x18` (position buffer)
4. Read half-float vertex normal from position buffer + vertexCount * 6 (normal buffer)
5. Read bone weights (3 half-floats) and bone indices (4 bytes) from `shapeData + 0x20` at vertexIndex * 12
6. 4th weight = 1.0 - (w0 + w1 + w2)
7. For each of 4 bones (breaks if weight == 0):
   - Get bone node from `skinInst->bones[boneIdx]` (offset 0x28)
   - Compute combined transform via FUN_1401a8d60: `combinedMatrix = boneWorldTransform * skinToBoneTransform`
     - boneWorldTransform = bone node pointer (param to FUN_1401a8d60)
     - skinToBoneTransform = `boneData->transforms[boneIdx]` at `*(skinInst+0x40)+0x10 + boneIdx*0x50 + 0x10`
   - Accumulate: `result += combinedMatrix * vertexPos * weight`

**FUN_1401a8d60 (MatrixMultiply4x4):** At 0x1401a8d60
```c
// result = param1 (4x4 bone world) * param3 (4x4 skinToBone)
// This is a full 4x4 matrix multiply stored as row-major float[16]
```

### ApplySkinningToGeometry (0x141db2be0)

**Signature:** `void ApplySkinningToGeometry(BSTriShape* shape, void* destBuffer, uint* normalDest, uint param4, void* mappedVertexData)`

**Algorithm:**
1. Get BSSkin::Instance from shape[0x30] (offset 0x180)
2. Get vertexDesc, determine vertex stride
3. Map GPU vertex buffer if not provided
4. **Phase 1: Build combined bone matrices (all at once)**
   - Read per-bone transforms from `BSSkin::Instance + 0xA0` (pre-computed bone world transforms, 0x30 floats = 12 floats per 4x4 row-major matrix, 0x30 stride)
   - Apply geometry local transform (negated translation from shape offsets 0xA0, 0xA4, 0xA8)
   - Multiply against constant matrices (DAT_142c91a10 etc. -- possibly inverse bind adjustments)
   - Store result in temp buffer: 0x40 bytes per bone (16 floats = 4x4 matrix)
5. **Phase 2: Per-vertex skinning**
   - For each vertex (count from shape+0x1A4):
     - Read vertex position as 4x half-float (8 bytes) from mapped vertex data
     - Read bone weights as 4x half-float from bone weight buffer offset
     - Read bone indices as 4x uint8 from bone index buffer (at weight offset + 8)
     - 4th weight = 1.0 - (w0 + w1 + w2)
     - For each bone (up to 4, stop when weight <= 0):
       - Access combined matrix at `tempBuffer + boneIdx * 0x40`
       - Accumulate weighted transform of position
     - Pack result into output buffer (bit-interleaved format for GPU)
     - Optionally transform normals similarly

**Key constant addresses:**
- `DAT_142c7f640` = 1.0f (used as "1.0 - sum of weights" reference)
- `FUN_1402883e0` = half-float to float conversion

**Pointer chain for skinning:**
```
BSTriShape* shape;
BSSkin::Instance* skin = *(BSSkin::Instance**)(shape + 0x180);

// Bone count
uint32_t boneCount = *(uint32_t*)(skin + 0x38);

// Pre-built bone world transforms (from last update)
void* boneWorldTransforms = *(void**)(skin + 0xA0);
// Each entry: 0x30 bytes (12 floats: 3 rows of 4x4 matrix + translation row)

// SkinToBone transforms (inverse bind pose)
BSSkin::BoneData* boneData = *(BSSkin::BoneData**)(skin + 0x40);
void* skinToBoneArray = *(void**)(boneData + 0x10);
// Each entry at: skinToBoneArray + boneIdx * 0x50 + 0x10 (3x4 matrix)

// Bone nodes (for per-vertex lookup)
void** boneNodes = *(void***)(skin + 0x10);  // NiTArray data
// Access: boneNodes[boneIdx] -> NiNode*, worldTransform at node + 0x70

// Bone world transform pointers (cached)
void** boneWorldTransformPtrs = *(void***)(skin + 0x28);  // points to bone+0x70 for each bone
```

---

## Item 4: BSDynamicTriShape detection

### BSDynamicTriShape struct layout (0x1C0 bytes, extends BSTriShape at 0x1B0)

**Vtable:** `0x142e5a488`
**NiRTTI object:** `DAT_145c62aa0`
**Allocation size:** 0x1C0 (confirmed at 0x141c370c0)

BSDynamicTriShape extends BSTriShape by 0x10 bytes:

| Offset | Size | Type | Field | Evidence |
|--------|------|------|-------|----------|
| 0x000-0x1AF | 0x1B0 | BSTriShape | base class | |
| 0x1A0 | 8 | ptr | pDynamicData (ref-counted object) | Ctor: `param_1[0x34] = 0`; Dtor releases as ref-counted |
| 0x1A8 | 2+2 | uint16[2] | frame counters | NIF read copies 0x1A8, 0x1AA |
| 0x1AC | 2+2 | uint16[2] | (vertex range?) | NIF read copies 0x1AC, 0x1AE |
| 0x1B0 | 4+4 | -- | (extended fields) | Zeroed in ctor |
| 0x1B8 | 4 | uint32 | (unknown) | Set to 0 in ctor |

**WARNING:** BSDynamicTriShape's offset 0x1A0 (pDynamicData) OVERLAPS with BSTriShape's field at the same offset. BSTriShape stores `numTriangles` (uint32) at 0x1A0 and `vertexCount` (uint16) at 0x1A4. In BSDynamicTriShape, offset 0x1A0 becomes a full 8-byte pointer. This means:
- **BSTriShape**: 0x1A0 = uint32 numTriangles, 0x1A4 = uint16 vertexCount
- **BSDynamicTriShape**: 0x1A0 = qword pDynamicData pointer (overwrites the numTriangles/vertexCount fields)

This implies BSDynamicTriShape stores its vertex count differently or in the pDynamicData object itself.

### How to detect BSDynamicTriShape at runtime

**Method 1: NiRTTI check (used by the engine)**

The engine at 0x141c5da60 checks against `DAT_145c62aa0` (BSDynamicTriShape NiRTTI):
```c
// Get RTTI chain from BSGeometry via virtual call
NiRTTI* rtti = shape->vtable[0x10/8](shape);  // GetRTTI
while (rtti != NULL) {
    if (rtti == DAT_145c62aa0) {  // BSDynamicTriShape NiRTTI
        // It's a BSDynamicTriShape
        break;
    }
    rtti = rtti->parent;
}
```

**Method 2: Vtable comparison**
```c
if (*(void**)shape == (void*)0x142e5a488) {  // BSDynamicTriShape vtable
    // It's exactly BSDynamicTriShape (not a subclass)
}
```

**Method 3: Check BSGeometry flags (set by constructor)**
```c
// BSDynamicTriShape ctor sets: param_1[0x21] |= 0x400000000
uint64_t flags = *(uint64_t*)(shape + 0x108);  // param_1[0x21] = offset 0x108
if (flags & 0x400000000) {
    // Has BSDynamicTriShape flag
}
```

**Method 4: Check type byte**
```c
// BSDynamicTriShape sets *(byte*)(shape + 0x198) = 1
// BSTriShape sets *(byte*)(shape + 0x198) = 3
uint8_t typeFlag = *(uint8_t*)(shape + 0x198);
if (typeFlag == 1) {
    // BSDynamicTriShape
} else if (typeFlag == 3) {
    // BSTriShape
}
```

### BSDynamicTriShape vertex data differences

BSDynamicTriShape sets vertexDesc to `0x840200002000031` via BSGeometry_SetVertexDesc.
This encodes a specific vertex format. The vertexDesc bitfield determines:
- Vertex stride (extracted via `(vertexDesc << 2) & 0x3C`)
- Position format
- UV/normal/tangent presence

The constructor stores vertex data differently -- `*(byte*)(shape + 0x198) = 1` vs BSTriShape's `= 3`, which controls how the vertex buffer is mapped (via `FUN_141d90640` vs `FUN_141d90620`).

**Key functions:**
| Address | Name |
|---------|------|
| 0x141c37930 | BSDynamicTriShape constructor (no params) |
| 0x141c37670 | BSDynamicTriShape constructor (with data) |
| 0x141c370c0 | BSDynamicTriShape factory (NIF alloc, 0x1C0 bytes) |
| 0x141c37190 | BSDynamicTriShape LoadBinary |
| 0x141c37a50 | BSDynamicTriShape destructor (with free) |
| 0x141c37720 | BSDynamicTriShape destructor (no free) |

---

## Summary: Complete BSTriShape layout (0x1B0 bytes)

| Offset | Size | Type | Field | Source |
|--------|------|------|-------|--------|
| 0x000 | 8 | ptr | vtable | `0x142e5a0b8` for BSTriShape, `0x142e5a488` for BSDynamicTriShape |
| 0x008 | 4 | uint32 | refCount | NiObject |
| 0x010-0x027 | 24 | -- | NiObjectNET fields (name hash, extra data) | |
| 0x028-0x02F | 8 | ptr | NiObjectNET name | |
| 0x030-0x10F | 224 | -- | NiAVObject fields (local transform, world transform, etc.) | |
| 0x0A0 | 4 | float | worldTransform.translate.x | Used in skinning |
| 0x0A4 | 4 | float | worldTransform.translate.y | Used in skinning |
| 0x0A8 | 4 | float | worldTransform.translate.z | Used in skinning |
| 0x100 | 8 | ptr | collisionObject | BSGeometry (set in NIF read at 0x141c22b00) |
| 0x108 | 8 | uint64 | flags | BSGeometry flags; BSDynamicTriShape sets bit 34 |
| 0x110-0x15F | 80 | -- | BSGeometry data (property arrays etc.) | |
| 0x160 | 4 | float | bounds.center.x | BSTriShape |
| 0x164 | 4 | float | bounds.center.y | |
| 0x168 | 4 | float | bounds.center.z | |
| 0x16C | 4 | float | bounds.radius | |
| 0x170 | 8 | ptr | shaderProperty (NiBSShaderProperty*) | BSGeometry, ref-counted |
| 0x178 | 8 | ptr | skinInstance_legacy (NiSkinInstance*) | BSGeometry level, NOT used by BSTriShape. Legacy for NiTriShape. |
| 0x180 | 8 | ptr | **skinInstance (BSSkin::Instance*)** | **THE actual skin pointer for BSTriShape** |
| 0x188 | 8 | ptr | rawVertexData | GPU vertex buffer; set via FUN_141c31da0 |
| 0x190 | 8 | uint64 | vertexDesc | Cached from rawVertexData[0]; bitfield encoding vertex format |
| 0x198 | 1 | byte | geometryType | 3 = BSTriShape, 1 = BSDynamicTriShape, 4 = BSSubIndexTriShape |
| 0x199 | 1 | byte | (flags) | |
| 0x1A0 | 4 | uint32 | numTriangles | BSTriShape (overwritten in BSDynamicTriShape!) |
| 0x1A4 | 2 | uint16 | numVertices | BSTriShape vertex count |
| 0x1A6-0x1AF | 10 | -- | padding/unused | |

---

## Key vtable addresses

| Class | Vtable | Alloc Size |
|-------|--------|------------|
| BSGeometry | 0x142e58338 | ~0x160 |
| BSTriShape | 0x142e5a0b8 | 0x1B0 |
| BSDynamicTriShape | 0x142e5a488 | 0x1C0 |
| BSSegmentedTriShape | 0x142e5c388 (via NiTriShape chain) | 0x1E0 |
| NiSkinInstance | 0x142e5de48 | 0x98 |
| BSSkin::Instance | 0x142e59a48 | 0xC0 |
| BSSkin::BoneData | 0x142e598e8 | 0x28 |
| NiSkinPartition | 0x142e5daa8 | 0x30 |
| NiSkinData | 0x142e5dc28 | 0x60 |

---

## Vertex data access for skinned meshes

### Step-by-step: Reading a skinned vertex position

```cpp
// 1. Get the skin instance
BSSkin::Instance* skin = *(BSSkin::Instance**)((char*)triShape + 0x180);
if (!skin) return; // Not skinned

// 2. Get bone data
BSSkin::BoneData* boneData = *(BSSkin::BoneData**)((char*)skin + 0x40);
uint32_t boneCount = *(uint32_t*)((char*)skin + 0x38);

// 3. Get vertex data (from shapeData via property lookup)
// shapeData is obtained via FUN_141c16e20(triShape, &DAT_145c629c0)
// Position data at shapeData + 0x18 (half-float array, 3 per vertex)
// Normal data at shapeData + 0x18 + numVertices * 6
// Bone weight/index at shapeData + 0x20 (12 bytes per vertex: 6 bytes weights + 2 unused + 4 bytes bone indices)

// 4. Get bone transforms
// Bone nodes: *(NiNode***)(skin + 0x10) -> array of NiNode*
// Bone world transforms: each bone node + 0x70 = NiTransform
// SkinToBone: *(void**)(boneData + 0x10) + boneIdx * 0x50 + 0x10 = 3x4 matrix

// 5. Apply skinning formula
// For each vertex:
//   Read 3 half-float weights, compute w3 = 1.0 - w0 - w1 - w2
//   Read 4 bone indices (uint8)
//   For each bone i (0-3, stop if weight[i] == 0):
//     combined = boneWorldTransform[boneIdx] * skinToBone[boneIdx]  (4x4 * 4x4)
//     result += combined * vertexPos * weight[i]
```
