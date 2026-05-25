#pragma once

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/object/GrabTargetKind.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpConstraintCinfo.h"
#include "RE/Havok/hknpWorld.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace rock
{

    struct HkPositionMotor
    {
        void* vtable;
        std::int16_t referenceCount;
        std::uint16_t memSizeAndFlags;
        std::uint32_t pad0C;
        std::uint8_t type;
        std::uint8_t pad11[7];
        float minForce;
        float maxForce;
        float tau;
        float damping;
        float proportionalRecoveryVelocity;
        float constantRecoveryVelocity;
    };
    static_assert(sizeof(HkPositionMotor) == 0x30);

    constexpr std::size_t HK_POSITION_MOTOR_SIZE = 0x30;

    inline constexpr std::uintptr_t MOTOR_VTABLE_POSITION = 0x142e95fe8;
    inline constexpr std::uint16_t HK_REFERENCED_OBJECT_INITIAL_REFCOUNT = 1;
    inline constexpr std::uint16_t HK_REFERENCED_OBJECT_DEFAULT_MEM_FLAGS = 0xffff;
    inline constexpr std::uint32_t HK_REFERENCED_OBJECT_HEADER_DWORD = 0xffff0001u;

    /*
     * ROCK's grab constraint is a custom FO4VR hknp/hkp atom contract, not a
     * stock hkpRagdollConstraintData instance. Ghidra disassembly of FO4VR's
     * stock ragdoll constructor and getConstraintInfo virtual shows the atom
     * block starts at this+0x20 (`ADD RCX, 0x20`), even though the decompiler's
     * synthetic field labels misleadingly call it offset_0x18. Keeping the
     * object/atom boundary identical to FO4VR reduces the number of inherited
     * vtable paths that can observe an impossible layout.
     */
    inline constexpr std::size_t GRAB_CONSTRAINT_SIZE = 0x168;

    inline constexpr int ATOMS_START = 0x20;
    inline constexpr int ATOM_TRANSFORMS = ATOMS_START;
    inline constexpr int ATOM_STABILIZE = ATOM_TRANSFORMS + 0x90;
    inline constexpr int ATOM_RAGDOLL_MOT = ATOM_STABILIZE + 0x10;
    inline constexpr int ATOM_LIN_MOTOR_0 = ATOM_RAGDOLL_MOT + 0x60;
    inline constexpr int ATOM_LIN_MOTOR_1 = ATOM_LIN_MOTOR_0 + 0x18;
    inline constexpr int ATOM_LIN_MOTOR_2 = ATOM_LIN_MOTOR_1 + 0x18;
    inline constexpr int ATOMS_SIZE = 0x148;

    inline constexpr int GRAB_TRANSFORM_A_COL0 = ATOM_TRANSFORMS + 0x10;
    inline constexpr int GRAB_TRANSFORM_A_COL1 = ATOM_TRANSFORMS + 0x20;
    inline constexpr int GRAB_TRANSFORM_A_COL2 = ATOM_TRANSFORMS + 0x30;
    inline constexpr int GRAB_TRANSFORM_A_POS = ATOM_TRANSFORMS + 0x40;
    inline constexpr int GRAB_TRANSFORM_B_COL0 = ATOM_TRANSFORMS + 0x50;
    inline constexpr int GRAB_TRANSFORM_B_COL1 = ATOM_TRANSFORMS + 0x60;
    inline constexpr int GRAB_TRANSFORM_B_COL2 = ATOM_TRANSFORMS + 0x70;
    inline constexpr int GRAB_TRANSFORM_B_POS = ATOM_TRANSFORMS + 0x80;

    inline constexpr std::uint16_t ATOM_TYPE_SET_LOCAL_TRANSFORMS = 2;
    inline constexpr std::uint16_t ATOM_TYPE_SETUP_STABILIZATION = 23;
    inline constexpr std::uint16_t ATOM_TYPE_RAGDOLL_MOTOR = 19;
    inline constexpr std::uint16_t ATOM_TYPE_LIN_MOTOR = 11;

    inline constexpr int RAGDOLL_MOTOR_TARGET_BRCA = 0x10;
    inline constexpr int RAGDOLL_MOTOR_MOTORS = 0x40;

    inline constexpr int RUNTIME_SOLVER_RESULTS = 12;
    inline constexpr int HKP_SOLVER_RESULT_SIZE = 0x08;
    inline constexpr int RT_SOLVER_RESULTS_BYTES = RUNTIME_SOLVER_RESULTS * HKP_SOLVER_RESULT_SIZE;

    /*
     * FO4VR hknp allocates hkp external runtime as solver results first, then
     * atom private state. Stock point-to-plane, prismatic, and linear-clearance
     * constraints all report runtime size as numSolverResults * 8 plus their
     * motor state. Keep all motor offsets past the 12 result slots.
     */
    inline constexpr int RT_RAGDOLL_INIT_OFFSET = RT_SOLVER_RESULTS_BYTES;
    inline constexpr int RT_RAGDOLL_PREV_ANG_OFFSET = RT_RAGDOLL_INIT_OFFSET + 0x04;
    inline constexpr int RT_LINEAR_INIT_BASE = RT_RAGDOLL_PREV_ANG_OFFSET + 0x0C;
    inline constexpr int RT_LINEAR_PREV_POS_BASE = RT_LINEAR_INIT_BASE + 0x04;
    inline constexpr int RT_RUNTIME_USED_SIZE = RT_LINEAR_PREV_POS_BASE + 0x0C;
    inline constexpr int RUNTIME_REPORTED_SIZE = RT_RUNTIME_USED_SIZE * 2;

    inline constexpr int VTABLE_SLOT_DESTRUCTOR = 0;
    inline constexpr int VTABLE_SLOT_GET_TYPE = 4;
    inline constexpr int VTABLE_SLOT_GET_CONSTRAINT_INFO = 5;
    inline constexpr int VTABLE_SLOT_IS_VALID = 6;
    inline constexpr int VTABLE_SLOT_SET_SOLVING_METHOD = 9;
    inline constexpr int VTABLE_SLOT_GET_INERTIA_STABILIZATION = 12;
    inline constexpr int VTABLE_SLOT_SET_DISPLAY_FLAGS = 13;
    inline constexpr int VTABLE_SLOT_GET_DISPLAY_FLAGS = 14;
    inline constexpr int VTABLE_SLOT_SET_MOTOR_MODE = 15;
    inline constexpr int VTABLE_SLOT_SET_MAX_FRICTION = 16;
    inline constexpr int VTABLE_SLOT_GET_MAX_FRICTION = 17;
    inline constexpr int VTABLE_SLOT_GET_RUNTIME_INFO = 18;
    inline constexpr int VTABLE_SLOT_ADD_INSTANCE = 20;

    inline constexpr std::uintptr_t RAGDOLL_VTABLE = 0x142e18298;

    enum class GrabAngularAuthority : std::uint8_t
    {
        HknpRagdollMotorAtom = 1,
    };

    inline const char* grabAngularAuthorityName(GrabAngularAuthority authority) noexcept
    {
        switch (authority) {
        case GrabAngularAuthority::HknpRagdollMotorAtom:
        default:
            return "hknpRagdollMotorAtom";
        }
    }

    struct ActiveConstraint
    {
        std::uint32_t constraintId = 0x7FFF'FFFF;
        void* constraintData = nullptr;
        HkPositionMotor* angularMotor = nullptr;
        HkPositionMotor* linearMotor = nullptr;
        GrabAngularAuthority angularAuthority = GrabAngularAuthority::HknpRagdollMotorAtom;
        float currentTau = 0.0f;
        float currentMaxForce = 0.0f;
        float targetMaxForce = 0.0f;

        bool isValid() const { return constraintId != 0x7FFF'FFFF && constraintData != nullptr; }
        bool usesRagdollAngularMotorAtom() const { return angularAuthority == GrabAngularAuthority::HknpRagdollMotorAtom; }

        void clear()
        {
            constraintId = 0x7FFF'FFFF;
            constraintData = nullptr;
            angularMotor = nullptr;
            linearMotor = nullptr;
            angularAuthority = GrabAngularAuthority::HknpRagdollMotorAtom;
            currentTau = 0.0f;
            currentMaxForce = 0.0f;
            targetMaxForce = 0.0f;
        }
    };

    struct GrabConstraintMotorTuning
    {
        float linearTau = 0.03f;
        float linearDamping = 0.8f;
        float linearProportionalRecovery = 2.0f;
        float linearConstantRecovery = 1.0f;
        float linearMaxForce = 2000.0f;

        float angularTau = 0.03f;
        float angularDamping = 0.8f;
        float angularProportionalRecovery = 2.0f;
        float angularConstantRecovery = 1.0f;
        float angularMaxForce = 2000.0f;
        GrabAngularAuthority angularAuthority = GrabAngularAuthority::HknpRagdollMotorAtom;
    };

    inline constexpr int MOTION_PACKED_INERTIA_OFFSET = 0x20;

    inline float unpackBfloat16(std::int16_t packed)
    {
        std::uint32_t expanded = static_cast<std::uint32_t>(static_cast<std::uint16_t>(packed)) << 16;
        float result;
        std::memcpy(&result, &expanded, sizeof(float));
        return result;
    }

    inline std::int16_t repackBfloat16(float value)
    {
        std::int32_t raw;
        std::memcpy(&raw, &value, sizeof(std::int32_t));
        std::int32_t shifted = raw >> 16;

        if (shifted > 32767)
            shifted = 32767;
        if (shifted < -32768)
            shifted = -32768;
        return static_cast<std::int16_t>(shifted);
    }

    struct SavedObjectState
    {
        struct SavedMotionInertiaState
        {
            RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
            std::uint32_t motionIndex = 0;
            std::int16_t savedPackedInertia[3] = { 0, 0, 0 };
            bool inertiaModified = false;
        };

        RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
        RE::TESObjectREFR* refr = nullptr;
        grab_target::Kind targetKind = grab_target::Kind::LooseObject;
        std::uint32_t originalFilterInfo = 0;
        std::uint16_t originalMotionPropsId = 0;
        float inverseMass = 0.0f;
        std::int16_t savedPackedInertia[3] = { 0, 0, 0 };
        bool inertiaModified = false;
        std::vector<SavedMotionInertiaState> motionInertiaStates;

        bool isValid() const { return bodyId.value != 0x7FFF'FFFF && refr != nullptr; }

        void clear()
        {
            bodyId.value = 0x7FFF'FFFF;
            refr = nullptr;
            targetKind = grab_target::Kind::LooseObject;
            originalFilterInfo = 0;
            originalMotionPropsId = 0;
            inverseMass = 0.0f;
            savedPackedInertia[0] = savedPackedInertia[1] = savedPackedInertia[2] = 0;
            inertiaModified = false;
            motionInertiaStates.clear();
        }
    };

    void normalizeGrabbedInertia(RE::hknpWorld* world, RE::hknpBodyId bodyId, SavedObjectState& savedState);

    void normalizeGrabbedInertiaForBodies(RE::hknpWorld* world, RE::hknpBodyId primaryBodyId, const std::vector<std::uint32_t>& heldBodyIds, SavedObjectState& savedState);

    void restoreGrabbedInertia(RE::hknpWorld* world, SavedObjectState& savedState);

    HkPositionMotor* createPositionMotor(float tau, float damping, float proportionalRecoveryVelocity, float constantRecoveryVelocity, float minForce, float maxForce);

    ActiveConstraint createGrabConstraint(RE::hknpWorld* world, RE::hknpBodyId handBodyId, RE::hknpBodyId objectBodyId,
        const RE::NiTransform& handBodyWorld, const RE::NiTransform& constraintFrameABodySpace, const RE::NiPoint3& palmWorldGame, const float* pivotBBodyLocalHk,
        const RE::NiTransform& desiredBodyTransformHandSpace, float tau, float damping, float maxForce, float proportionalRecovery, float constantRecovery);

    ActiveConstraint createGrabConstraint(RE::hknpWorld* world, RE::hknpBodyId handBodyId, RE::hknpBodyId objectBodyId,
        const RE::NiTransform& handBodyWorld, const RE::NiTransform& constraintFrameABodySpace, const RE::NiPoint3& palmWorldGame, const float* pivotBBodyLocalHk,
        const RE::NiTransform& desiredBodyTransformHandSpace, const GrabConstraintMotorTuning& tuning);

    void destroyGrabConstraint(RE::hknpWorld* world, ActiveConstraint& constraint);
    void serviceRetiredGrabConstraintPayloads(std::uint32_t completedPhysicsSteps = 1);

    void cleanupGrabConstraintVtable();
}
