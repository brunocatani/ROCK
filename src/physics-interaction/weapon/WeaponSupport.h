#pragma once

/*
 * Weapon support-grip behavior is grouped here so support authority, pose policy, and the two-handed solver evolve as one coherent subsystem.
 */


// ---- WeaponSupportAuthorityPolicy.h ----

/*
 * Equipped sidearm support grip is split from full two-handed weapon authority
 * because pistols need the visual stability of a left-hand mesh grab without
 * turning the left controller into a weapon steering input. HIGGS-style full
 * two-hand solving remains the correct model for long guns, while sidearms use
 * wrist from the stored weapon-local grab frame. Runtime classification uses
 * the weapon animation grip keywords because Fallout 4 only exposes "gun" at
 * the weapon-type enum level, while actual vanilla and modded pistol records
 * carry `AnimsGripPistol`. Long-gun grip keywords are checked before the
 * sidearm signal so rifle-like weapons keep the solver that owns weapon
 * transform authority. `BGSKeywordForm::HasKeyword(..., TBO_InstanceData*)`
 * preserves weapon-mod instance changes. Name fallback tokens only cover
 * vanilla sidearms whose records do not expose a stronger semantic signal.
 */

#include "physics-interaction/TransformMath.h"

#include <array>
#include <cstdint>
#include <string_view>

namespace frik::rock::weapon_support_authority_policy
{
    enum class WeaponSupportAuthorityMode
    {
        FullTwoHandedSolver = 0,
        VisualOnlySupport = 1,
    };

    enum class WeaponSupportWeaponClass
    {
        Unknown = 0,
        Sidearm = 1,
        LongGun = 2,
    };

    struct EquippedWeaponIdentity
    {
        std::uint32_t formID{ 0 };
        std::string_view displayName{};
        std::string_view nodeName{};
        bool hasPistolGripKeyword{ false };
        bool hasInstancePistolGripKeyword{ false };
        bool hasLongGunGripKeyword{ false };
        bool hasInstanceLongGunGripKeyword{ false };
    };

    inline constexpr bool hasPistolGripSemantic(const EquippedWeaponIdentity& identity)
    {
        return identity.hasPistolGripKeyword || identity.hasInstancePistolGripKeyword;
    }

    inline constexpr bool hasLongGunGripSemantic(const EquippedWeaponIdentity& identity)
    {
        return identity.hasLongGunGripKeyword || identity.hasInstanceLongGunGripKeyword;
    }

    inline constexpr WeaponSupportAuthorityMode resolveSupportAuthorityMode(
        bool visualOnlySidearmSupportEnabled,
        WeaponSupportWeaponClass weaponClass)
    {
        return visualOnlySidearmSupportEnabled && weaponClass == WeaponSupportWeaponClass::Sidearm ?
                   WeaponSupportAuthorityMode::VisualOnlySupport :
                   WeaponSupportAuthorityMode::FullTwoHandedSolver;
    }

    inline constexpr bool supportGripOwnsWeaponTransform(WeaponSupportAuthorityMode mode)
    {
        return mode == WeaponSupportAuthorityMode::FullTwoHandedSolver;
    }

    inline constexpr bool supportGripAppliesPrimaryHandAuthority(WeaponSupportAuthorityMode mode)
    {
        return mode == WeaponSupportAuthorityMode::FullTwoHandedSolver;
    }

    inline constexpr bool supportGripAppliesSupportHandAuthority(WeaponSupportAuthorityMode)
    {
        return true;
    }

    template <class Transform>
    inline Transform buildVisualOnlySupportHandWorld(const Transform& weaponWorld, const Transform& supportHandWeaponLocal)
    {
        return transform_math::composeTransforms(weaponWorld, supportHandWeaponLocal);
    }

    inline char lowerAscii(char value)
    {
        return value >= 'A' && value <= 'Z' ? static_cast<char>(value - 'A' + 'a') : value;
    }

    inline std::string_view trimAscii(std::string_view value)
    {
        while (!value.empty() && (value.front() == ' ' || value.front() == '\t' || value.front() == '\r' || value.front() == '\n')) {
            value.remove_prefix(1);
        }
        while (!value.empty() && (value.back() == ' ' || value.back() == '\t' || value.back() == '\r' || value.back() == '\n')) {
            value.remove_suffix(1);
        }
        return value;
    }

    inline bool containsIgnoreCase(std::string_view haystack, std::string_view needle)
    {
        needle = trimAscii(needle);
        if (needle.empty() || needle.size() > haystack.size()) {
            return false;
        }

        for (std::size_t start = 0; start + needle.size() <= haystack.size(); ++start) {
            bool matched = true;
            for (std::size_t index = 0; index < needle.size(); ++index) {
                if (lowerAscii(haystack[start + index]) != lowerAscii(needle[index])) {
                    matched = false;
                    break;
                }
            }
            if (matched) {
                return true;
            }
        }
        return false;
    }

    inline bool identityNameMatchesAny(const EquippedWeaponIdentity& identity, const std::string_view* tokens, std::size_t tokenCount)
    {
        for (std::size_t index = 0; index < tokenCount; ++index) {
            if (containsIgnoreCase(identity.displayName, tokens[index]) || containsIgnoreCase(identity.nodeName, tokens[index])) {
                return true;
            }
        }
        return false;
    }

    inline WeaponSupportWeaponClass classifyEquippedWeaponForSupportGrip(const EquippedWeaponIdentity& identity)
    {
        if (hasLongGunGripSemantic(identity)) {
            return WeaponSupportWeaponClass::LongGun;
        }
        if (hasPistolGripSemantic(identity)) {
            return WeaponSupportWeaponClass::Sidearm;
        }

        static constexpr std::array<std::string_view, 8> kSidearmNameFallbackTokens{
            "pistol",
            "revolver",
            "deliverer",
            "alien blaster",
            "gamma gun",
            "flare gun",
            "the gainer",
            "western revolver",
        };
        if (identityNameMatchesAny(identity, kSidearmNameFallbackTokens.data(), kSidearmNameFallbackTokens.size())) {
            return WeaponSupportWeaponClass::Sidearm;
        }

        static constexpr std::array<std::string_view, 11> kLongGunTokens{
            "rifle",
            "shotgun",
            "musket",
            "launcher",
            "minigun",
            "fat man",
            "flamer",
            "gatling",
            "harpoon",
            "submachine",
            "machine gun",
        };
        if (identityNameMatchesAny(identity, kLongGunTokens.data(), kLongGunTokens.size())) {
            return WeaponSupportWeaponClass::LongGun;
        }

        return WeaponSupportWeaponClass::Unknown;
    }
}

// ---- WeaponSupportGripPolicy.h ----

#include "physics-interaction/weapon/WeaponTypes.h"

namespace frik::rock::weapon_support_grip_policy
{
    /*
     * HIGGS gates two-handing by whether the held item is two-handable and by
     * the hand's live contact with the equipped weapon, not by a narrow list of
     * approved mesh part names. PAPER owns semantic reload/action routing
     * through the provider API; ROCK's support grip remains a generic equipped-weapon contact so
     * stocks, magazines, accessories, unknown NIF chunks, and melee blades can
     * all become the locked support point the player actually touched.
     */

    inline WeaponGripPoseId fallbackPoseForPart(WeaponPartKind partKind)
    {
        switch (partKind) {
        case WeaponPartKind::Foregrip:
            return WeaponGripPoseId::VerticalForegrip;
        case WeaponPartKind::Pump:
            return WeaponGripPoseId::PumpGrip;
        case WeaponPartKind::Handguard:
            return WeaponGripPoseId::HandguardClamp;
        case WeaponPartKind::Barrel:
            return WeaponGripPoseId::BarrelWrap;
        case WeaponPartKind::Magazine:
        case WeaponPartKind::Magwell:
            return WeaponGripPoseId::MagwellHold;
        case WeaponPartKind::Stock:
        case WeaponPartKind::Receiver:
        case WeaponPartKind::Grip:
        case WeaponPartKind::Bolt:
        case WeaponPartKind::Slide:
        case WeaponPartKind::ChargingHandle:
        case WeaponPartKind::BreakAction:
        case WeaponPartKind::Cylinder:
        case WeaponPartKind::Chamber:
        case WeaponPartKind::LaserCell:
        case WeaponPartKind::Lever:
        case WeaponPartKind::Sight:
        case WeaponPartKind::Accessory:
            return WeaponGripPoseId::ReceiverSupport;
        case WeaponPartKind::Shell:
        case WeaponPartKind::Round:
        case WeaponPartKind::CosmeticAmmo:
        case WeaponPartKind::Other:
        default:
            return WeaponGripPoseId::BarrelWrap;
        }
    }

    inline WeaponGripPoseId resolveSupportGripPose(const WeaponInteractionContact& contact)
    {
        return contact.fallbackGripPose != WeaponGripPoseId::None ? contact.fallbackGripPose : fallbackPoseForPart(contact.partKind);
    }

    inline bool canUseContactForSupportGrip(const WeaponInteractionContact& contact, const WeaponInteractionRuntimeState& runtimeState)
    {
        return contact.valid && runtimeState.supportGripAllowed;
    }
}

// ---- WeaponSupportThumbPosePolicy.h ----

/*
 * Two-handed equipped-weapon support grip uses the HIGGS mesh probe to decide
 * whether the thumb should follow the alternate thumb curve. FRIK's existing
 * scalar pose API can bend the thumb, but it cannot switch the local rotation
 * basis the way HIGGS does. Keep the publication decision isolated here so the
 * runtime only sends local thumb transforms when the mesh solve selected that
 * alternate path and the provider can actually accept those transforms.
 */

namespace frik::rock::weapon_support_thumb_pose_policy
{
    [[nodiscard]] constexpr bool shouldPublishAlternateThumbLocalOverride(
        const bool solvedFingerPose,
        const bool usedAlternateThumbCurve,
        const bool hasLocalTransformApi)
    {
        return solvedFingerPose && usedAlternateThumbCurve && hasLocalTransformApi;
    }

    template <class Vector>
    [[nodiscard]] constexpr Vector predictThumbNodeWorldForGripFrame(
        const Vector& liveThumbNodeWorld,
        const Vector& currentSupportGripPivotWorld,
        const Vector& targetGripPointWorld)
    {
        const Vector supportHandOffset{
            targetGripPointWorld.x - currentSupportGripPivotWorld.x,
            targetGripPointWorld.y - currentSupportGripPivotWorld.y,
            targetGripPointWorld.z - currentSupportGripPivotWorld.z,
        };
        return Vector{
            liveThumbNodeWorld.x + supportHandOffset.x,
            liveThumbNodeWorld.y + supportHandOffset.y,
            liveThumbNodeWorld.z + supportHandOffset.z,
        };
    }

    template <class Vector>
    [[nodiscard]] constexpr Vector vectorToGripFromPredictedThumbNode(
        const Vector& liveThumbNodeWorld,
        const Vector& currentSupportGripPivotWorld,
        const Vector& targetGripPointWorld)
    {
        const Vector predictedNodeWorld =
            predictThumbNodeWorldForGripFrame(liveThumbNodeWorld, currentSupportGripPivotWorld, targetGripPointWorld);
        return Vector{
            targetGripPointWorld.x - predictedNodeWorld.x,
            targetGripPointWorld.y - predictedNodeWorld.y,
            targetGripPointWorld.z - predictedNodeWorld.z,
        };
    }
}

// ---- WeaponTwoHandedGripMath.h ----


namespace frik::rock::weapon_two_handed_grip_math
{
    /*
     * Equipped weapon two-hand support has two independent ownership rules:
     * the support hand must be attached to the mesh point it actually touched,
     * and it must not also own a normal dynamic-object grab. Keeping these
     * rules as pure math/policy avoids mixing HIGGS-style weapon authority with
     * ROCK's separate dynamic object grab path.
     */

    template <class Transform, class Vector>
    inline Transform alignHandFrameToGripPoint(const Transform& handWorldTransform, const Vector& currentGripPivotWorld, const Vector& targetGripPointWorld)
    {
        Transform result = handWorldTransform;
        const Vector correction = weaponSolverSub(targetGripPointWorld, currentGripPivotWorld);
        result.translate = weaponSolverAdd(result.translate, correction);
        return result;
    }

    inline bool canStartSupportGrip(bool touchingSupportPart, bool gripPressed, bool supportHandHoldingObject)
    {
        return touchingSupportPart && gripPressed && !supportHandHoldingObject;
    }

    inline bool shouldContinueSupportGrip(bool gripPressed, bool supportHandHoldingObject)
    {
        return gripPressed && !supportHandHoldingObject;
    }

    inline bool canProcessNormalGrabInput(bool isLeft, bool equippedWeaponSupportGripActive, bool rightHandWeaponEquipped)
    {
        if (isLeft) {
            return !equippedWeaponSupportGripActive;
        }

        return !rightHandWeaponEquipped;
    }
}

// ---- WeaponTwoHandedSolver.h ----

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <cmath>

namespace frik::rock
{
    /*
     * FRIK's suppressed offhand grip works because it solves the weapon
     * transform from two points: the fixed primary grip and the offhand support
     * point. ROCK uses generated mesh collision, so this solver returns one
     * coherent weapon-root transform that can be applied to both the visible
     * weapon node and every generated collision body.
     */

    template <class Transform, class Vector>
    struct WeaponTwoHandedSolverInput
    {
        Transform weaponWorldTransform{};
        Vector primaryGripLocal{};
        Vector supportGripLocal{};
        Vector primaryTargetWorld{};
        Vector supportTargetWorld{};
        Vector supportNormalLocal{};
        Vector supportNormalTargetWorld{};
        float minimumSeparation{ 0.001f };
        float supportNormalTwistFactor{ 0.0f };
        bool useSupportNormalTwist{ false };
    };

    template <class Transform>
    struct WeaponTwoHandedSolverResult
    {
        Transform weaponWorldTransform{};
        decltype(Transform{}.rotate) rotationDelta{};
        bool solved{ false };
        float primaryError{ 0.0f };
        float supportError{ 0.0f };
    };

    template <class Vector>
    inline Vector weaponSolverSub(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    template <class Vector>
    inline Vector weaponSolverAdd(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
    }

    template <class Vector>
    inline Vector weaponSolverScale(const Vector& vector, float scale)
    {
        return Vector{ vector.x * scale, vector.y * scale, vector.z * scale };
    }

    template <class Vector>
    inline float weaponSolverDot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector weaponSolverCross(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
    }

    template <class Vector>
    inline float weaponSolverLength(const Vector& vector)
    {
        return std::sqrt(weaponSolverDot(vector, vector));
    }

    template <class Vector>
    inline Vector weaponSolverNormalize(const Vector& vector)
    {
        const float length = weaponSolverLength(vector);
        if (length <= 0.000001f) {
            return Vector{};
        }
        return weaponSolverScale(vector, 1.0f / length);
    }

    template <class Matrix, class Vector>
    inline Matrix weaponSolverStoredRotationFromConventionalRows(const Vector rows[3])
    {
        Matrix result{};
        for (int row = 0; row < 3; ++row) {
            result.entry[row][0] = rows[0].x;
            result.entry[row][1] = rows[1].x;
            result.entry[row][2] = rows[2].x;
        }
        result.entry[0][0] = rows[0].x;
        result.entry[0][1] = rows[1].x;
        result.entry[0][2] = rows[2].x;
        result.entry[1][0] = rows[0].y;
        result.entry[1][1] = rows[1].y;
        result.entry[1][2] = rows[2].y;
        result.entry[2][0] = rows[0].z;
        result.entry[2][1] = rows[1].z;
        result.entry[2][2] = rows[2].z;
        return result;
    }

    template <class Vector>
    inline Vector weaponSolverOrthogonalAxis(const Vector& value)
    {
        Vector axis = std::abs(value.x) < 0.7f ? Vector{ 1.0f, 0.0f, 0.0f } : Vector{ 0.0f, 1.0f, 0.0f };
        return weaponSolverNormalize(weaponSolverCross(value, axis));
    }

    template <class Matrix, class Vector>
    inline Matrix weaponSolverRotationBetweenStored(const Vector& fromRaw, const Vector& toRaw)
    {
        const Vector from = weaponSolverNormalize(fromRaw);
        const Vector to = weaponSolverNormalize(toRaw);
        float cosTheta = (std::max)(-1.0f, (std::min)(1.0f, weaponSolverDot(from, to)));

        if (cosTheta > 0.9999f) {
            return transform_math::makeIdentityRotation<Matrix>();
        }

        Vector axis{};
        float sinTheta = 0.0f;
        if (cosTheta < -0.9999f) {
            axis = weaponSolverOrthogonalAxis(from);
            sinTheta = 0.0f;
            cosTheta = -1.0f;
        } else {
            axis = weaponSolverNormalize(weaponSolverCross(from, to));
            sinTheta = std::sqrt((std::max)(0.0f, 1.0f - cosTheta * cosTheta));
        }

        const float x = axis.x;
        const float y = axis.y;
        const float z = axis.z;
        const float oneMinusCos = 1.0f - cosTheta;

        Vector conventionalRows[3]{
            Vector{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
            Vector{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
            Vector{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
        };

        return weaponSolverStoredRotationFromConventionalRows<Matrix, Vector>(conventionalRows);
    }

    template <class Matrix, class Vector>
    inline Matrix weaponSolverAxisAngleStored(const Vector& axisRaw, float angle)
    {
        const Vector axis = weaponSolverNormalize(axisRaw);
        const float x = axis.x;
        const float y = axis.y;
        const float z = axis.z;
        const float cosTheta = std::cos(angle);
        const float sinTheta = std::sin(angle);
        const float oneMinusCos = 1.0f - cosTheta;

        Vector conventionalRows[3]{
            Vector{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
            Vector{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
            Vector{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
        };

        return weaponSolverStoredRotationFromConventionalRows<Matrix, Vector>(conventionalRows);
    }

    template <class Matrix, class Vector>
    inline Matrix weaponSolverApplyWorldRotationToStoredBasis(const Matrix& worldRotationStored, const Matrix& baseRotation)
    {
        Matrix result{};
        const Vector basis[3]{
            Vector{ baseRotation.entry[0][0], baseRotation.entry[0][1], baseRotation.entry[0][2] },
            Vector{ baseRotation.entry[1][0], baseRotation.entry[1][1], baseRotation.entry[1][2] },
            Vector{ baseRotation.entry[2][0], baseRotation.entry[2][1], baseRotation.entry[2][2] },
        };

        for (int axis = 0; axis < 3; ++axis) {
            const Vector rotated = transform_math::rotateLocalVectorToWorld(worldRotationStored, basis[axis]);
            result.entry[axis][0] = rotated.x;
            result.entry[axis][1] = rotated.y;
            result.entry[axis][2] = rotated.z;
        }
        return result;
    }

    template <class Matrix, class Vector>
    inline Vector weaponSolverApplyStoredWorldRotationToVector(const Matrix& worldRotationStored, const Vector& vector)
    {
        return transform_math::rotateLocalVectorToWorld(worldRotationStored, vector);
    }

    template <class Vector>
    inline Vector weaponSolverProjectOntoPlane(const Vector& vector, const Vector& planeNormal)
    {
        const float normalDot = weaponSolverDot(vector, planeNormal);
        return weaponSolverSub(vector, weaponSolverScale(planeNormal, normalDot));
    }

    template <class Vector>
    inline Vector makeLockedSupportGripTarget(
        const Vector& primaryTargetWorld,
        const Vector& supportControllerWorld,
        const Vector& fallbackSupportTargetWorld,
        float lockedGripDistance,
        float minimumSeparation)
    {
        /*
         * HIGGS keeps two-handed weapon grabs as captured hand-to-weapon
         * relationships and uses controller motion to rotate the held weapon,
         * not to slide the visual contact point along the model. ROCK keeps
         * that rule explicit by preserving the captured primary-support
         * distance while still using the support controller direction for aim.
         */
        if (lockedGripDistance <= minimumSeparation) {
            return fallbackSupportTargetWorld;
        }

        Vector targetAxis = weaponSolverSub(supportControllerWorld, primaryTargetWorld);
        if (weaponSolverLength(targetAxis) <= minimumSeparation) {
            targetAxis = weaponSolverSub(fallbackSupportTargetWorld, primaryTargetWorld);
        }
        if (weaponSolverLength(targetAxis) <= minimumSeparation) {
            return fallbackSupportTargetWorld;
        }

        const Vector direction = weaponSolverNormalize(targetAxis);
        return weaponSolverAdd(primaryTargetWorld, weaponSolverScale(direction, lockedGripDistance));
    }

    template <class Transform, class Vector>
    inline WeaponTwoHandedSolverResult<Transform> solveTwoHandedWeaponTransform(const WeaponTwoHandedSolverInput<Transform, Vector>& input)
    {
        WeaponTwoHandedSolverResult<Transform> result{};
        result.weaponWorldTransform = input.weaponWorldTransform;
        result.rotationDelta = transform_math::makeIdentityRotation<decltype(input.weaponWorldTransform.rotate)>();

        const Vector localAxis = weaponSolverSub(input.supportGripLocal, input.primaryGripLocal);
        const Vector currentAxisWorld = transform_math::localVectorToWorld(input.weaponWorldTransform, localAxis);
        const Vector desiredAxisWorld = weaponSolverSub(input.supportTargetWorld, input.primaryTargetWorld);

        if (weaponSolverLength(currentAxisWorld) <= input.minimumSeparation || weaponSolverLength(desiredAxisWorld) <= input.minimumSeparation) {
            return result;
        }

        const auto rotationDelta = weaponSolverRotationBetweenStored<decltype(input.weaponWorldTransform.rotate), Vector>(currentAxisWorld, desiredAxisWorld);
        result.rotationDelta = rotationDelta;
        result.weaponWorldTransform.rotate =
            weaponSolverApplyWorldRotationToStoredBasis<decltype(input.weaponWorldTransform.rotate), Vector>(rotationDelta, input.weaponWorldTransform.rotate);

        const Vector primaryAfterRotation = transform_math::localPointToWorld(result.weaponWorldTransform, input.primaryGripLocal);
        const Vector primaryCorrection = weaponSolverSub(input.primaryTargetWorld, primaryAfterRotation);
        result.weaponWorldTransform.translate = weaponSolverAdd(result.weaponWorldTransform.translate, primaryCorrection);

        const Vector primaryWorld = transform_math::localPointToWorld(result.weaponWorldTransform, input.primaryGripLocal);
        const Vector supportWorld = transform_math::localPointToWorld(result.weaponWorldTransform, input.supportGripLocal);
        result.primaryError = weaponSolverLength(weaponSolverSub(primaryWorld, input.primaryTargetWorld));
        result.supportError = weaponSolverLength(weaponSolverSub(supportWorld, input.supportTargetWorld));

        if (input.useSupportNormalTwist && input.supportNormalTwistFactor > 0.0f) {
            const Vector twistAxis = weaponSolverNormalize(weaponSolverSub(input.supportTargetWorld, input.primaryTargetWorld));
            const Vector currentNormalWorld = transform_math::localVectorToWorld(result.weaponWorldTransform, input.supportNormalLocal);
            const Vector desiredNormalWorld = input.supportNormalTargetWorld;
            const Vector currentProjected = weaponSolverNormalize(weaponSolverProjectOntoPlane(currentNormalWorld, twistAxis));
            const Vector desiredProjected = weaponSolverNormalize(weaponSolverProjectOntoPlane(desiredNormalWorld, twistAxis));

            if (weaponSolverLength(currentProjected) > input.minimumSeparation && weaponSolverLength(desiredProjected) > input.minimumSeparation) {
                const float dotValue = (std::max)(-1.0f, (std::min)(1.0f, weaponSolverDot(currentProjected, desiredProjected)));
                const Vector crossValue = weaponSolverCross(currentProjected, desiredProjected);
                const float signedAngle = std::atan2(weaponSolverDot(twistAxis, crossValue), dotValue) * input.supportNormalTwistFactor;
                const auto twistRotation = weaponSolverAxisAngleStored<decltype(input.weaponWorldTransform.rotate), Vector>(twistAxis, signedAngle);
                result.rotationDelta =
                    weaponSolverApplyWorldRotationToStoredBasis<decltype(input.weaponWorldTransform.rotate), Vector>(twistRotation, result.rotationDelta);

                const Vector supportPivot = input.supportTargetWorld;
                const Vector pivotToWeapon = weaponSolverSub(result.weaponWorldTransform.translate, supportPivot);
                const Vector rotatedPivotToWeapon = weaponSolverApplyStoredWorldRotationToVector<decltype(input.weaponWorldTransform.rotate), Vector>(twistRotation, pivotToWeapon);
                result.weaponWorldTransform.translate = weaponSolverAdd(supportPivot, rotatedPivotToWeapon);
                result.weaponWorldTransform.rotate =
                    weaponSolverApplyWorldRotationToStoredBasis<decltype(input.weaponWorldTransform.rotate), Vector>(twistRotation, result.weaponWorldTransform.rotate);

                const Vector primaryAfterTwist = transform_math::localPointToWorld(result.weaponWorldTransform, input.primaryGripLocal);
                const Vector primaryTwistCorrection = weaponSolverSub(input.primaryTargetWorld, primaryAfterTwist);
                result.weaponWorldTransform.translate = weaponSolverAdd(result.weaponWorldTransform.translate, primaryTwistCorrection);

                const Vector primaryFinal = transform_math::localPointToWorld(result.weaponWorldTransform, input.primaryGripLocal);
                const Vector supportFinal = transform_math::localPointToWorld(result.weaponWorldTransform, input.supportGripLocal);
                result.primaryError = weaponSolverLength(weaponSolverSub(primaryFinal, input.primaryTargetWorld));
                result.supportError = weaponSolverLength(weaponSolverSub(supportFinal, input.supportTargetWorld));
            }
        }

        result.solved = true;
        return result;
    }

    template <class Transform, class Vector>
    inline WeaponTwoHandedSolverResult<Transform> solveTwoHandedWeaponTransformFrikPivot(const WeaponTwoHandedSolverInput<Transform, Vector>& input)
    {
        /*
         * This named entry point is intentionally kept beside the generic
         * solver. FRIK's suppressed two-handed grip already proved the right
         * firearm behavior: aim from the right-hand primary grip toward the
         * offhand support point, then retranslate the weapon so the primary
         * grip pivot does not drift. ROCK adds HIGGS-style semantic mesh
         * anchors and full weapon/collision authority around that same math
         * instead of inventing a second aiming convention.
         */
        return solveTwoHandedWeaponTransform(input);
    }
}
