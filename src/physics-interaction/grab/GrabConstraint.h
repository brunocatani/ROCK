#pragma once

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpConstraintCinfo.h"
#include "RE/Havok/hknpWorld.h"

#include <vector>

namespace frik::rock
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

    inline constexpr std::size_t GRAB_CONSTRAINT_SIZE = 0x168;

    inline constexpr int ATOMS_START = 0x20;
    inline constexpr int ATOM_TRANSFORMS = 0x20;
    inline constexpr int ATOM_STABILIZE = 0xB0;
    inline constexpr int ATOM_RAGDOLL_MOT = 0xC0;
    inline constexpr int ATOM_LIN_MOTOR_0 = 0x120;
    inline constexpr int ATOM_LIN_MOTOR_1 = 0x138;
    inline constexpr int ATOM_LIN_MOTOR_2 = 0x150;
    inline constexpr int ATOMS_SIZE = 0x148;

    inline constexpr std::uint16_t ATOM_TYPE_SET_LOCAL_TRANSFORMS = 2;
    inline constexpr std::uint16_t ATOM_TYPE_SETUP_STABILIZATION = 23;
    inline constexpr std::uint16_t ATOM_TYPE_RAGDOLL_MOTOR = 19;
    inline constexpr std::uint16_t ATOM_TYPE_LIN_MOTOR = 11;

    inline constexpr int RUNTIME_SOLVER_RESULTS = 12;
    inline constexpr int RUNTIME_REPORTED_SIZE = 0x100;

    inline constexpr int RT_RAGDOLL_INIT_OFFSET = 0x60;
    inline constexpr int RT_RAGDOLL_PREV_ANG_OFFSET = 0x64;

    inline constexpr int VTABLE_SLOT_GET_TYPE = 4;
    inline constexpr int VTABLE_SLOT_GET_CONSTRAINT_INFO = 5;
    inline constexpr int VTABLE_SLOT_IS_VALID = 6;
    inline constexpr int VTABLE_SLOT_GET_RUNTIME_INFO = 18;
    inline constexpr int VTABLE_SLOT_ADD_INSTANCE = 20;

    inline constexpr std::uintptr_t RAGDOLL_VTABLE = 0x142e18298;

    struct ActiveConstraint
    {
        std::uint32_t constraintId = 0x7FFF'FFFF;
        void* constraintData = nullptr;
        HkPositionMotor* angularMotor = nullptr;
        HkPositionMotor* linearMotor = nullptr;
        float currentTau = 0.0f;
        float currentMaxForce = 0.0f;
        float targetMaxForce = 0.0f;

        bool isValid() const { return constraintId != 0x7FFF'FFFF && constraintData != nullptr; }

        void clear()
        {
            constraintId = 0x7FFF'FFFF;
            constraintData = nullptr;
            angularMotor = nullptr;
            linearMotor = nullptr;
            currentTau = 0.0f;
            currentMaxForce = 0.0f;
            targetMaxForce = 0.0f;
        }
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
        const RE::NiTransform& handBodyWorld, const RE::NiPoint3& palmWorldGame, const float* pivotBBodyLocalHk,
        const RE::NiTransform& desiredBodyTransformHandSpace, float tau, float damping, float maxForce, float proportionalRecovery, float constantRecovery);

    void destroyGrabConstraint(RE::hknpWorld* world, ActiveConstraint& constraint);

    void cleanupGrabConstraintVtable();
}
