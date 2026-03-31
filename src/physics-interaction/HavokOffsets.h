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
    // Used in Hand (keyframe drive), HandGrab (release zero-vel), WeaponCollision.
    constexpr std::uintptr_t kFunc_SetBodyVelocity = 0x1539F30;

    // hknpBSWorld::setBodyKeyframed(world*, bodyId)
    // Transitions a body to keyframed motion. Used in Hand and WeaponCollision.
    constexpr std::uintptr_t kFunc_SetBodyKeyframed = 0x1DF5CB0;

    // hkpPhysicsUtils::computeHardKeyFrame(...)
    // Computes linear/angular velocities to drive a body to a target transform.
    // Used in Hand and WeaponCollision keyframe stepping.
    constexpr std::uintptr_t kFunc_ComputeHardKeyFrame = 0x153a6a0;
}
