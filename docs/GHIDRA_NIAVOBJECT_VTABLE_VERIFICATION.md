# NiAVObject Virtual Function & Layout Verification — FO4VR Binary
## Ghidra blind verification, 2026-03-30

### Vtable Addresses Found

| Class | VR Vtable Address | Size (alloc) | Constructor Chain |
|-------|------------------|--------------|-------------------|
| NiAVObject | `0x142e58338` | 0x160 (VR) / 0x120 (flat) | FUN_141c23ea0 |
| NiNode | `0x142e57a68` | 0x180 | FUN_141c17c50 (factory) -> FUN_141c23ea0 (NiAVObject ctor) |
| BSGeometry | `0x142e59668` | — | FUN_141c31380 -> FUN_141c23ea0 |
| BSTriShape | `0x142e5a0b8` | 0x1B0 | FUN_141c36ce0 (factory) -> FUN_141c31380 -> FUN_141c23ea0 |

### Item 1: IsTriShape() — Vtable Slot 0x0A

**CommonLibF4VR claims:** Slot 0x0A. Returns `BSTriShape*` (this) for BSTriShape, `nullptr` for base.

**Ghidra verification:**

| Vtable | Slot 0x0A Address | Function Pointer | Behavior |
|--------|-------------------|-----------------|----------|
| NiAVObject (0x142e58338) | +0x50 = 0x142e58388 | `0x1400382b0` | Returns nullptr (shared stub) |
| NiNode (0x142e57a68) | +0x50 = 0x142e57ab8 | `0x1400382b0` | Returns nullptr (same stub) |
| BSTriShape (0x142e5a0b8) | +0x50 = 0x142e5a108 | `0x141c370b0` | Override — returns `this` |

The override function `0x141c370b0` is referenced ONLY from BSTriShape and its derived vtables
(BSSubIndexTriShape, BSDynamicTriShape, etc.), confirming it's the "return this" override.

**Verdict: CONFIRMED** — Slot 0x0A, same in flat and VR.

---

### Item 2: IsNode() — Vtable Slots 0x03 (const) and 0x04 (non-const)

**CommonLibF4VR claims:** Slot 0x03 = `IsNode() const`, Slot 0x04 = `IsNode()`. Both return `NiNode*`.

**Ghidra verification:**

| Vtable | Slot 0x03 (+0x18) | Slot 0x04 (+0x20) |
|--------|-------------------|-------------------|
| NiAVObject | `0x140038230` (nullptr stub) | `0x140038220` (nullptr stub) |
| NiNode | `0x1401933d0` (override) | `0x1401933c0` (override) |
| BSTriShape | `0x140038230` (nullptr stub) | `0x140038220` (nullptr stub) |

The override functions `0x1401933d0` and `0x1401933c0` appear in NiNode and all NiNode-derived
vtables (BSFadeNode, BSMultiBoundNode, BSNiNode, etc.) — 20+ classes. They are NOT present in
any geometry-derived vtable. This confirms they return `this` for NiNode subclasses.

**Verdict: CONFIRMED** — Slots 0x03 and 0x04, same in flat and VR.

---

### Item 3: NiNode Children Array

**CommonLibF4VR claims:** `GetRuntimeData()` uses `REL::RelocateMember<RUNTIME_DATA>(this, 0x120, 0x160)`.
RUNTIME_DATA contains NiTObjectArray<NiPointer<NiAVObject>> at offset 0x00 within the struct.

**Ghidra verification (from NiNode functions FUN_141c17af0, FUN_141c17b80, FUN_141c191b0):**

All three functions iterate children using:
```c
count = *(ushort*)(niNode + 0x172);  // _freeIdx within NiTArray
data  = *(ptr**)(niNode + 0x168);    // _data within NiTArray
for (i = 0; i < count; i++) {
    child = data[i];  // *(ptr*)(data + i * 8)
}
```

NiTArray layout (from header, sizeof = 0x18):
- +0x00: vtable (8 bytes)
- +0x08: _data pointer (8 bytes)
- +0x10: _capacity (uint16)
- +0x12: _freeIdx (uint16) -- USED AS ITERATION COUNT
- +0x14: _size (uint16)
- +0x16: _growthSize (uint16)

Children array at VR offset 0x160:
- +0x160: NiTArray vtable (`0x142e57a50`)
- +0x168: _data (array of NiPointer<NiAVObject>)
- +0x170: _capacity
- +0x172: _freeIdx (used as child count for iteration)
- +0x174: _size
- +0x176: _growthSize

NiNode constructor (FUN_141c17c50) confirms:
- `puVar3[0x2c]` (offset 0x160) = vtable 0x142e57a50
- `puVar3[0x2d]` (offset 0x168) = 0 (empty data)
- `*(uint32*)(puVar3 + 0x174)` = 0x10000 (_size=0, _growthSize=1)

**IMPORTANT NOTE:** The game uses `_freeIdx` (offset 0x12 within NiTArray, absolute 0x172)
as the iteration bound, NOT `_size` (offset 0x14, absolute 0x174). For a packed array where
children are always contiguous from index 0, _freeIdx == effective count.

**Verdict: CONFIRMED** — Children at VR offset 0x160 via `GetRuntimeData()`. Data pointer at +0x168, count at +0x172.

---

### Item 4: NiAVObject::flags Offset and Culled Bit

**CommonLibF4VR claims:** `NiTFlags<uint64_t, NiAVObject> flags` at offset 0x108.
`GetAppCulled() = flags.flags & 1` (bit 0 = culled/hidden).

**Ghidra verification:**

**SetAppCulled (FUN_141c23380, VR vtable slot 0x30):**
```c
void SetAppCulled(NiAVObject* this, bool cull) {
    if ((*(byte*)(this + 0x108) & 1) != cull) {
        if (!cull)
            *(uint64*)(this + 0x108) &= ~1;       // Clear bit 0
        else
            *(uint64*)(this + 0x108) |= 1;        // Set bit 0
        *(uint64*)(this + 0x108) |= 0x20000000;   // Set dirty flag (bit 29)
        // Walk parent chain setting dirty
        parent = *(ptr*)(this + 0x28);
        while (parent && !(*(uint64*)(parent + 0x108) >> 29 & 1)) {
            *(uint64*)(parent + 0x108) |= 0x20000000;
            parent = *(ptr*)(parent + 0x28);
        }
    }
}
```

**UpdateWorldData (FUN_141c23740) also uses:**
```c
if ((*(ulonglong*)(param_1 + 0x108) >> 0x2D & 1) != 0)  // bit 45 = inherit transform
```

**NiAVObject constructor (FUN_141c23ea0):**
```c
param_1[0x21] = 0x2000000000000e;  // offset 0x108, initial flags value
```
Initial flags = 0x2000000000000e:
- Bit 0 = 0 (NOT culled by default)
- Bit 1 = 1
- Bit 2 = 1
- Bit 3 = 1
- Bit 45 (0x2D) = 1 (inherit transform flag set)

**Flags field is at offset 0x108 in BOTH flat and VR** — no RelocateMember offset adjustment.
The NiAVObject constructor and SetAppCulled both access it directly at 0x108 in the VR binary.

**Known flag bits (from Ghidra):**
- Bit 0: App culled (hidden)
- Bit 29 (0x1D): Dirty/needs-update flag
- Bit 45 (0x2D): Inherit parent transform (when set, copies parent world transform directly)

**Verdict: CONFIRMED** — Flags at 0x108, bit 0 = culled. Same offset in flat and VR.

---

### VR Vtable Slot Shift Discovery

The VR NiAVObject vtable has 3 EXTRA virtual slots compared to flat, inserted around slot 0x2A-0x2C.
This causes all slots >= 0x2D (flat) to shift by +3 in VR:

| Function | Flat Slot | VR Slot | Notes |
|----------|-----------|---------|-------|
| IsNode() const | 0x03 | 0x03 | No shift |
| IsNode() | 0x04 | 0x04 | No shift |
| IsTriShape() | 0x0A | 0x0A | No shift |
| UpdateControllers | 0x28 | 0x28 | No shift |
| (VR extra) | — | 0x2A | UpdateWorldData variant (FUN_141c23010) |
| (VR extra) | — | 0x2B | UpdateControllers variant (FUN_141c232f0) |
| (VR extra) | — | 0x2C | Unknown |
| SetAppCulled | 0x2D | 0x30 | +3 shift |
| UpdateWorldData | 0x34 | 0x37 | +3 shift (confirmed by header) |
| AttachChild | 0x3A | 0x3D | +3 shift (confirmed by header) |

**Critical for ROCK:** IsTriShape (0x0A), IsNode (0x03/0x04), and all NiObject type-check
virtuals (slots 0x02-0x27) are NOT shifted in VR and can be called directly.

---

### Additional Offset Verification

| Field | CommonLibF4VR Offset | Ghidra VR Binary | Status |
|-------|---------------------|------------------|--------|
| NiAVObject::parent | 0x028 | 0x28 (SetAppCulled walks parent at +0x28) | CONFIRMED |
| NiAVObject::local (NiTransform) | 0x030 | 0x30 (UpdateWorldData uses +0x30) | CONFIRMED |
| NiAVObject::world (NiTransform) | 0x070 | 0x70 (UpdateWorldData writes +0x70) | CONFIRMED |
| NiAVObject::collisionObject | 0x100 | 0x100 (destructor accesses param_1[0x20]) | CONFIRMED |
| NiAVObject::flags | 0x108 | 0x108 (multiple functions confirm) | CONFIRMED |
| NiNode::children (VR) | 0x160 | 0x160 (constructor + iteration) | CONFIRMED |
