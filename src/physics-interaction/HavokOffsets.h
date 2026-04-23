#pragma once

// HavokOffsets.h — Named constants for reverse-engineered Havok memory offsets
// used across multiple ROCK source files.
//
// SCOPE: Only multi-file or structural offsets live here. Single-use offsets with
// inline Ghidra comments stay in their source files — no value in indirection.
//
// All addresses verified via Ghidra against FO4VR 1.2.72 binary.
//
// Naming conventions:
//   k{Struct}_{FieldName}  — struct-relative byte offset
//   kFunc_{FunctionName}   — REL::Offset address for engine functions

#include <cstdint>

namespace frik::rock::offsets
{
    // =========================================================================
    // bhkNPCollisionObject layout
    // =========================================================================

    // NiCollisionObject::sceneObject lives at +0x10 in the CommonLibF4VR layout and
    // is the stable typed owner-node access path already used elsewhere in ROCK.
    // Keep diagnostics aligned with the typed field instead of a raw owner-pointer
    // assumption on a partially audited base layout.
    constexpr std::uintptr_t kCollisionObject_OwnerNode = 0x10;

    // bhkNPCollisionObject+0x20 holds the bhkPhysicsSystem* pointer.
    // Used to traverse collision objects and enumerate body IDs.
    // Verified in Hand.cpp (collectHeldBodyIds) and WeaponCollision.cpp
    // (findWeaponShapeRecursive) — identical access pattern.
    constexpr std::uintptr_t kCollisionObject_PhysSystemPtr = 0x20;

    // =========================================================================
    // hknpWorld internals
    // =========================================================================

    // hknpWorld+0x150 holds the hknpModifierManager* pointer (0x618 bytes).
    // Ghidra-verified: hknpWorld ctor (0x141540d90) stores result of
    // FUN_141725450 (modifierManager init) at param_1[0x2A] = world+0x150.
    // The modifier manager holds 10×0x88 modifier slots + callback pointers
    // + the collision filter reference at +0x5E8.
    // NOTE: This is NOT the event dispatcher (which lives at world+0x648).
    constexpr std::uintptr_t kHknpWorld_ModifierManager = 0x150;

    // hknpModifierManager+0x5E8 holds the bhkCollisionFilter* pointer.
    // Ghidra-verified: FUN_141725450 initializes *(param+0x5E8) = 0 (NULL),
    // then Bethesda's world setup populates it with the active filter.
    // Full chain: *(*(world+0x150) + 0x5E8) → bhkCollisionFilter*.
    constexpr std::uintptr_t kModifierMgr_FilterPtr = 0x5E8;

    // filterPtr+0x1A0 is the collision matrix base (uint64 array indexed by layer).
    // Used to read/write per-layer collision masks.
    constexpr std::uintptr_t kFilter_CollisionMatrix = 0x1A0;

    // hknpWorld+0xE0 holds the motion array pointer.
    // Used for constraint solver access to motion data (GrabConstraint).
    constexpr std::uintptr_t kHknpWorld_MotionArrayPtr = 0xE0;

    // =========================================================================
    // hknpBody layout (within body array)
    // =========================================================================

    // hknpBody+0x44 holds the collision filter info (uint32).
    // Used to save/restore filter info during grab and weapon collision setup.
    // Verified in HandGrab.cpp (grab/release filter save) and WeaponCollision.cpp.
    constexpr std::uintptr_t kBody_CollisionFilterInfo = 0x44;

    // hknpBody+0x88 holds the bhkNPCollisionObject* back-pointer.
    // Binary-verified in bhkNPCollisionObject::vfunction48 on 2026-04-22.
    constexpr std::uintptr_t kBody_CollisionObjectBackPointer = 0x88;

    // =========================================================================
    // hknpMotion layout (within motion array)
    // =========================================================================

    // hknpMotion+0x38 holds the motionPropertiesId (uint16).
    // Identifies the motion type (static=0, dynamic=1, keyframed=2, etc.).
    // Used to detect/verify dynamic vs. keyframed state.
    constexpr std::uintptr_t kMotion_PropertiesId = 0x38;

    // =========================================================================
    // SetLocalTransforms atom sub-offsets
    // (relative to constraint data base pointer)
    //
    // These offsets address fields within the SetLocalTransforms atom that starts
    // at constraintData+0x20. They appear as raw pointer arithmetic in both
    // GrabConstraint.cpp (initial setup) and HandGrab.cpp (per-frame updates, logging).
    //
    // Layout: TransformA (rotation 3x16 + position 16) then TransformB (same).
    // =========================================================================

    // TransformA columns and position
    constexpr int kTransformA_Col0 = 0x30;   // rotation column 0 (4 floats)
    constexpr int kTransformA_Col1 = 0x40;   // rotation column 1
    constexpr int kTransformA_Col2 = 0x50;   // rotation column 2
    constexpr int kTransformA_Pos  = 0x60;   // translation (pivotA)

    // TransformB columns and position
    constexpr int kTransformB_Col0 = 0x70;   // rotation column 0
    constexpr int kTransformB_Col1 = 0x80;   // rotation column 1
    constexpr int kTransformB_Col2 = 0x90;   // rotation column 2
    constexpr int kTransformB_Pos  = 0xA0;   // translation (pivotB)

    // =========================================================================
    // Engine function addresses (REL::Offset values)
    // Used via: static REL::Relocation<sig_t> func{ REL::Offset(kFunc_xxx) };
    // =========================================================================

    // hknpBSWorld::setBodyCollisionFilterInfo(world*, bodyId, filterInfo, mode)
    // Used in HandGrab (grab/release filter) and WeaponCollision (layer setup).
    constexpr std::uintptr_t kFunc_SetBodyCollisionFilterInfo = 0x1DF5B80;

    // hknpBSWorld::setBodyVelocity(world*, bodyId, linVel*, angVel*)
    // RAW Havok call — NOT deferred-safe. Only use from main thread outside physics step.
    // Prefer kFunc_SetBodyVelocityDeferred for safety.
    constexpr std::uintptr_t kFunc_SetBodyVelocity = 0x1539F30;

    // Bethesda deferred-safe SetBodyTransform(world*, bodyId, hkTransformf*, activeBehavior)
    // Checks TLS+0x1528: if inside physics step → queues deferred command (type 0x60100,
    // 0x50 bytes). If outside step → calls hknpWorld::setBodyTransform directly.
    // Does NOT require bhkNPCollisionObject — takes world + bodyId directly.
    // Ghidra-verified: 0x141df55f0 (blind audit 2026-03-31).
    constexpr std::uintptr_t kFunc_SetBodyTransformDeferred = 0x1DF55F0;

    // Bethesda deferred-safe SetBodyVelocity(world*, bodyId, linVel*, angVel*)
    // Checks TLS+0x1528: if inside physics step → queues deferred command (type 0x90100,
    // 0x30 bytes). If outside step → calls hknpWorld::setBodyVelocity directly.
    // Also handles near-zero check and body activation after velocity set.
    // Does NOT require bhkNPCollisionObject — takes world + bodyId directly.
    // NOTE: The separate SetLinearVelocity/SetAngularVelocity wrappers are NOT deferred-safe.
    // Always use this combined version for deferred safety.
    // Ghidra-verified: 0x141df56f0 (blind audit 2026-03-31).
    constexpr std::uintptr_t kFunc_SetBodyVelocityDeferred = 0x1DF56F0;

    // hknpBSWorld::setBodyKeyframed(world*, bodyId)
    // Transitions a body to keyframed motion. Used in Hand and WeaponCollision.
    constexpr std::uintptr_t kFunc_SetBodyKeyframed = 0x1DF5CB0;

    // hkpPhysicsUtils::computeHardKeyFrame(...)
    // Computes linear/angular velocities to drive a body to a target transform.
    // Used in Hand and WeaponCollision keyframe stepping.
    constexpr std::uintptr_t kFunc_ComputeHardKeyFrame = 0x153a6a0;

    // hknpWorld::rebuildMotionMassProperties(world*, motionId, rebuildMode)
    // Recomputes combined mass, COM, inertia for a motion and all linked bodies.
    // Walks circular body chain via body+0x64. Updates body+0x80 (COM offset),
    // body+0x7C (AABB radius). Eigendecomposition for inertia tensor.
    // rebuildMode: 0=immediate, 1=deferred.
    // Ghidra-verified: 0x141546570 (blind audit 2026-03-31).
    constexpr std::uintptr_t kFunc_RebuildMotionMassProperties = 0x1546570;

    // bhkNPCollisionObject::SetMotionType(this*, motionType)
    // Full motion type transition via Bethesda wrapper. Handles:
    //   - Resolving bodyId from physics system
    //   - Acquiring world lock at world+0x690
    //   - Creating/assigning motion slots for DYNAMIC transition
    //   - Walking linked body chain via body+0x64/0x68
    //   - Rebuilding mass properties and collision caches
    //   - Setting correct body+0x40 flags (clears KEYFRAMED 0x4 for DYNAMIC)
    // motionType: 0=STATIC, 1=DYNAMIC, 2=KEYFRAMED.
    // Ghidra-verified: 0x141e07300 (blind audit 2026-03-31).
    constexpr std::uintptr_t kFunc_CollisionObject_SetMotionType = 0x1E07300;

    // =========================================================================
    // BethesdaPhysicsBody creation pipeline (Ghidra blind audit 2026-03-31)
    // Following CreatePhantomBody (0x140f0a340) golden reference.
    // =========================================================================

    // --- Object construction ---

    // bhkNPCollisionObject::ctor(this*, bodyIndex, bhkPhysicsSystem*)
    // Creates 0x30-byte collision object wrapper. Sets vtable, physics system ref, body index.
    constexpr std::uintptr_t kFunc_CollisionObject_Ctor = 0x1E07710;

    // bhkNPCollisionObject::CreateInstance(this*, bhkWorld*)
    // Instantiates body in world from physics system data. Sets body+0x88 back-pointer.
    constexpr std::uintptr_t kFunc_CollisionObject_CreateInstance = 0x1E07AC0;

    // bhkPhysicsSystem::ctor(this*, hknpPhysicsSystemData*)
    // Creates 0x28-byte physics system wrapper. Refs the system data.
    constexpr std::uintptr_t kFunc_PhysicsSystem_Ctor = 0x1E0C2B0;

    // bhkPhysicsSystem::GetBodyId(this*, hknpBodyId* out, int index)
    // Reads body ID from physics system instance body array.
    constexpr std::uintptr_t kFunc_PhysicsSystem_GetBodyId = 0x1E0C460;

    // hknpPhysicsSystemData::ctor(this*)
    // Initializes 0x78-byte system data with empty hkArrays.
    constexpr std::uintptr_t kFunc_PhysicsSystemData_Ctor = 0x5EAB0;

    // hknpBodyCinfo::ctor(this*)
    // Initializes 0x60-byte body creation info with defaults (identity transform, invalid IDs).
    constexpr std::uintptr_t kFunc_BodyCinfo_Ctor = 0x1561DD0;

    // hknpMaterial::ctor(this*)
    // Initializes 0x50-byte material with physics defaults (friction=0.5, restitution=0.5,
    // maxContactImpulse=FLT_MAX, etc.). WAVE 5 CORRECTION: Previously mislabeled as
    // "hknpMotionCinfo::init" — blind verification proved this is the material constructor.
    // The actual motion cinfo init function needs to be found separately.
    constexpr std::uintptr_t kFunc_MaterialCtor = 0x1536CB0;

    // NiCollisionObject::LinkObject(this*, NiAVObject*)
    // Sets bidirectional link: collObj+0x08=owner, niAVObject+0x100=collObj.
    constexpr std::uintptr_t kFunc_CollisionObject_LinkObject = 0x2996CB0;

    // --- Body operations (require bhkNPCollisionObject as this*) ---

    // bhkNPCollisionObject::DriveToKeyFrame(this*, NiTransform*, float dt)
    // Full keyframed body driver: computeHardKeyFrame + velocity limit check +
    // auto-teleport fallback + velocity application. Replaces manual 3-step process.
    // Returns true (0x901) on success, false if no world or dt <= 0.
    constexpr std::uintptr_t kFunc_CollisionObject_DriveToKeyFrame = 0x1E086E0;

    // bhkNPCollisionObject::SetTransform(this*, hkTransformf*)
    // Deferred-safe (delegates to inner function that checks TLS+0x1528).
    constexpr std::uintptr_t kFunc_CollisionObject_SetTransform = 0x1E08A70;

    // bhkNPCollisionObject::SetVelocity(this*, linVel*, angVel*)
    // Deferred-safe combined linear+angular velocity set.
    constexpr std::uintptr_t kFunc_CollisionObject_SetVelocity = 0x1E082A0;

    // bhkNPCollisionObject::SetLinearVelocity(this*, vel*) — NOT deferred-safe
    constexpr std::uintptr_t kFunc_CollisionObject_SetLinearVelocity = 0x1E08050;

    // bhkNPCollisionObject::SetAngularVelocity(this*, vel*) — NOT deferred-safe
    constexpr std::uintptr_t kFunc_CollisionObject_SetAngularVelocity = 0x1E08170;

    // bhkNPCollisionObject::ApplyLinearImpulse(this*, impulse*)
    constexpr std::uintptr_t kFunc_CollisionObject_ApplyLinearImpulse = 0x1E08520;

    // bhkNPCollisionObject::ApplyPointImpulseAt(this*, impulse*, point*)
    constexpr std::uintptr_t kFunc_CollisionObject_ApplyPointImpulse = 0x1E08640;

    // bhkNPCollisionObject::SetMass(this*, float mass)
    constexpr std::uintptr_t kFunc_CollisionObject_SetMass = 0x1E08C00;

    // bhkNPCollisionObject::GetCollisionFilterInfo(this*) → uint32
    constexpr std::uintptr_t kFunc_CollisionObject_GetFilterInfo = 0x1E08D60;

    // bhkNPCollisionObject::GetCenterOfMassInWorld(this*, hkVector4f* out)
    constexpr std::uintptr_t kFunc_CollisionObject_GetCOMWorld = 0x1E08EF0;

    // bhkNPCollisionObject::GetShape(this*) → hknpShape*
    constexpr std::uintptr_t kFunc_CollisionObject_GetShape = 0x1E07F30;

    // IsBodyConstrained(bhkNPCollisionObject*) → bool
    // Iterates physics system constraint list, returns true if body has any constraint.
    constexpr std::uintptr_t kFunc_IsBodyConstrained = 0x1E09170;

    // --- World-level operations ---

    // bhkWorld::AddPhysicsSystem(bhkWorld*, hknpPhysicsSystem* instance, bool deactivated)
    constexpr std::uintptr_t kFunc_World_AddPhysicsSystem = 0x1DFAC30;

    // bhkWorld::RemovePhysicsSystem(bhkWorld*, hknpPhysicsSystem* instance)
    constexpr std::uintptr_t kFunc_World_RemovePhysicsSystem = 0x1DFAD00;

    // bhkWorld::SetMotion(NiAVObject* root, motionType, activate, force, recurse)
    // Tree-level: walks NiNode tree, sets motion type on ALL collision objects.
    // motionType: 0=STATIC, 1=DYNAMIC, 2=KEYFRAMED.
    constexpr std::uintptr_t kFunc_World_SetMotion = 0x1DF95B0;

    // bhkWorld::EnableCollision(NiAVObject* root, enable, activate, force)
    // Tree-level: walks NiNode tree, enables/disables collision on ALL objects.
    constexpr std::uintptr_t kFunc_World_EnableCollision = 0x1DF9940;

    // bhkWorld::PickObject(bhkWorld*, bhkPickData&)
    // Raycast with own locking. Handles body→NiAVObject resolution.
    constexpr std::uintptr_t kFunc_World_PickObject = 0x1DF8D60;

    // bhkWorld::AddPhysicsStepListener(bhkWorld*, listener&)
    constexpr std::uintptr_t kFunc_World_AddStepListener = 0x1DFA7B0;

    // --- Allocators ---

    // Bethesda physics memory pool (global singleton, for NiObject-derived types)
    constexpr std::uintptr_t kData_BethesdaAllocatorPool = 0x392E400;

    // Bethesda allocator function: allocate(pool*, size, alignment) → void*
    constexpr std::uintptr_t kFunc_BethesdaAlloc = 0x1B91950;

    // Havok TLS allocator key (TlsGetValue index for thread-local Havok heap)
    constexpr std::uintptr_t kData_HavokTlsAllocKey = 0x5B63B20;

    // hkArray::_reserveMore — doubles capacity and reallocates buffer. Returns void.
    // Does NOT increment size — caller must manually increment after reserving.
    // Signature: void(allocatorGlobal*, hkArray*, int stride)
    // Ghidra-verified decompilation: FUN_14155d820 (2026-03-31).
    constexpr std::uintptr_t kFunc_HkArray_ReserveMore = 0x155D820;

    // hkArray allocator vtable global — address of global that HOLDS the allocator vtable ptr.
    // CRITICAL: Pass the RELOCATED ADDRESS of this global (via .address()), NOT a stack ptr.
    // Original crash: passing &REL::Relocation gave a stack address → null vtable → EXCEPTION_ACCESS_VIOLATION.
    // Correct: reinterpret_cast<void*>(REL::Relocation<uintptr_t>{REL::Offset(kData_...)}.address())
    constexpr std::uintptr_t kData_HkArrayAllocatorGlobal = 0x3866310;

    // hknpWorld::enableBodyFlags(world*, bodyId, flags, mode)
    // Enables body flags (contact reporting 0x80, keep-awake 0x8000000, etc.)
    constexpr std::uintptr_t kFunc_EnableBodyFlags = 0x153C090;

    // hknpWorld::disableBodyFlags(world*, bodyId, flags, mode)
    constexpr std::uintptr_t kFunc_DisableBodyFlags = 0x153C150;

    // hknpWorld::activateBody(world*, bodyId)
    // Wakes a sleeping body for physics simulation.
    constexpr std::uintptr_t kFunc_ActivateBody = 0x1546EF0;

    // =========================================================================
    // NiNode creation (Ghidra blind audit 2026-03-31, skeleton wave 4)
    // From BuildSceneNodeHierarchy (0x140ef21a0) golden reference.
    // =========================================================================

    // NiNode::ctor(this*) — default constructor, no parent.
    // Object size: 0x180 bytes. Sets vtable 0x142e57a68.
    // Calls NiAVObject::ctor (0x141c23ea0) as base init.
    constexpr std::uintptr_t kFunc_NiNode_Ctor = 0x1C17DD0;

    // NiNode::ctor(this*, parent*) — constructor with parent attachment.
    constexpr std::uintptr_t kFunc_NiNode_CtorWithParent = 0x1C17D30;

    // NiNode::SetName(node*, BSFixedString*) — sets the node's debug name.
    constexpr std::uintptr_t kFunc_NiNode_SetName = 0x1C16C30;

    // BSFixedString::Create(outStr*, const char*) — creates an interned string.
    constexpr std::uintptr_t kFunc_BSFixedString_Create = 0x1BC1650;

    // NiNode primary vtable.
    constexpr std::uintptr_t kData_NiNode_Vtable = 0x2E57A68;

    // NiNode object size (from BuildSceneNodeHierarchy allocations).
    constexpr std::size_t kNiNodeSize = 0x180;

    // NiNode allocation alignment (from BuildSceneNodeHierarchy).
    constexpr int kNiNodeAlignment = 0x10;

    // =========================================================================
    // hknpPhysicsSystemData array layout (WAVE 5 blind verification 2026-03-31)
    // Corrected from earlier agent analysis. Struct size: 0x78 bytes.
    // Each entry: hkArray = {void* data(+0), int32 size(+8), int32 capFlags(+C)}
    // =========================================================================
    constexpr int kSysData_Materials       = 0x10;  // hkArray<hknpMaterial>, stride 0x50
    constexpr int kSysData_Array1          = 0x20;  // hkArray (stride 0x60, motion properties?)
    constexpr int kSysData_Array2          = 0x30;  // hkArray (stride 0x70, body transforms?)
    constexpr int kSysData_BodyCinfos      = 0x40;  // hkArray<hknpBodyCinfo>, stride 0x60
    constexpr int kSysData_ConstraintInfos = 0x50;  // hkArray<hknpConstraintCinfo>, stride 0x18
    constexpr int kSysData_Shapes          = 0x60;  // hkArray<hknpShape*>, stride 0x08

    // =========================================================================
    // Collision query functions (WAVE 6 blind verification 2026-03-31)
    // =========================================================================

    // hknpWorld::castRay(world*, hknpRayCastQuery*, collector*)
    // World-level raycast with broadphase + narrowphase. Timer: "TtWorldCastRay".
    constexpr std::uintptr_t kFunc_World_CastRay = 0x15A6B10;

    // hknpWorld::getClosestPoints(world*, query*, transform*, collector*)
    // Closest points between query shape and all world bodies. 23+ callers.
    constexpr std::uintptr_t kFunc_World_GetClosestPoints = 0x15A6DF0;

    // hknpWorld::queryAabb(world*, hknpAabbQuery*, collector*) — full with narrowphase
    // Broadphase + per-body narrowphase overlap. Timer: "TtWorldQueryAabb".
    // This is what ROCK ObjectDetection uses.
    constexpr std::uintptr_t kFunc_World_QueryAabb = 0x15A64B0;

    // hknpWorld::queryAabb(world*, query*, hkArray<bodyId>*) — broadphase only
    // Fast path: returns candidate body IDs without narrowphase. 1 caller.
    constexpr std::uintptr_t kFunc_World_QueryAabbBroadphaseOnly = 0x15A6330;

    // =========================================================================
    // Additional function addresses (raw hex cleanup — C4 review item)
    // =========================================================================

    // VR native grab drop — releases BSMouseSpringAction springs on objects.
    // Called when ROCK grabs to kill lingering native springs.
    // Ghidra: FUN_140F1AB90 — VR Grab Drop
    constexpr std::uintptr_t kFunc_NativeVRGrabDrop = 0xF1AB90;

    // hknpWorld::setBodyMotionProperties(world*, bodyId, motionPropertiesId)
    // Raw motion properties preset change. Does NOT clear KEYFRAMED flag.
    // Prefer kFunc_CollisionObject_SetMotionType for full motion type transitions.
    constexpr std::uintptr_t kFunc_SetBodyMotionProperties = 0x153B2F0;

    // hknpConvexPolytopeShape vtable (convex base, not capsule-specific).
    // Used to overwrite capsule vtable when creating box shapes.
    // Ghidra: capsule vtable=0x142c9a208, convex=0x142c9a108.
    constexpr std::uintptr_t kData_ConvexPolytopeVtable = 0x2C9A108;

    // hknpMaterialLibrary::addMaterial(library*, outId*, material*)
    // Registers a custom material in the world's material library.
    constexpr std::uintptr_t kFunc_MaterialLibrary_AddMaterial = 0x1537840;

    // getConstraintInfoUtil — static utility that walks an atom chain and
    // computes ConstraintInfo (schema sizes, solver results, temps).
    // Found by decompiling 4 constraint types' getConstraintInfo stubs —
    // all tail-call to this same address. HIGGS calls equivalent at SkyrimVR 0xACC490.
    // Ghidra: FUN_141A4AD20.
    constexpr std::uintptr_t kFunc_GetConstraintInfoUtil = 0x1A4AD20;

    // HandleBumpedCharacter — CC bump handler. Hooked to suppress bumps near hands.
    // Ghidra: FUN_141E24980.
    constexpr std::uintptr_t kFunc_HandleBumpedCharacter = 0x1E24980;

    // VR Grab Initiate — native VR grab system entry point. Patched to NOP.
    // Ghidra: FUN_140F19250.
    constexpr std::uintptr_t kFunc_VRGrabInitiate = 0xF19250;

    // bhkCharProxyController::processConstraintsCallback — CC constraint callback.
    // Hooked to zero surface velocity for held objects (prevents CC push).
    // Ghidra: FUN_141E4B7E0.
    constexpr std::uintptr_t kFunc_ProcessConstraintsCallback = 0x1E4B7E0;

    // Main loop hook site — CALL instruction in the game loop where ROCK hooks.
    // Used for validateCriticalOffsets and the main per-frame hook.
    constexpr std::uintptr_t kHookSite_MainLoop = 0xD8405E;

    // bhkCollisionFilter global singleton — persistent filter created in bhkWorld::InitSystem.
    // Used as fallback when world modifier manager filter is null.
    constexpr std::uintptr_t kData_CollisionFilterSingleton = 0x59429B8;

    // Event subscription function — subscribe_ext for contact event callbacks.
    // Signature: void(signal*, userData*, callbackInfo*)
    constexpr std::uintptr_t kFunc_SubscribeContactEvent = 0x3B9E50;
}
