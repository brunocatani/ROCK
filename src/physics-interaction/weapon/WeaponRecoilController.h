#pragma once

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::weapon_recoil_policy
{
    enum class Profile : std::uint8_t
    {
        OneHand,
        FullTwoHand,
        CloseSupport,
        PowerArmor,
    };

    struct ProfileGains
    {
        float translation;
        float rotation;
    };

    // Independent profiles: tuning one hold must never retune another.
    inline constexpr ProfileGains kOneHand{ 1.0f, 1.0f };
    inline constexpr ProfileGains kFullTwoHand{ 1.0f, 1.0f };
    inline constexpr ProfileGains kCloseSupport{ 0.45f, 0.30f };
    inline constexpr ProfileGains kPowerArmor{ 0.45f, 0.30f };

    [[nodiscard]] inline constexpr Profile selectProfile(
        const bool inPowerArmor, const bool closeSupport, const bool fullTwoHanded) noexcept
    {
        return inPowerArmor ? Profile::PowerArmor :
               closeSupport ? Profile::CloseSupport :
               fullTwoHanded ? Profile::FullTwoHand : Profile::OneHand;
    }

    [[nodiscard]] inline constexpr ProfileGains gainsFor(const Profile profile) noexcept
    {
        switch (profile) {
        case Profile::PowerArmor: return kPowerArmor;
        case Profile::CloseSupport: return kCloseSupport;
        case Profile::FullTwoHand: return kFullTwoHand;
        default: return kOneHand;
        }
    }

    [[nodiscard]] inline constexpr const char* name(const Profile profile) noexcept
    {
        switch (profile) {
        case Profile::PowerArmor: return "power-armor";
        case Profile::CloseSupport: return "close-support";
        case Profile::FullTwoHand: return "full-two-hand";
        default: return "one-hand";
        }
    }

    // Matches the API role bits; physical left/right are resolved per sample.
    enum class HandMask : std::uint32_t { None = 0, Primary = 1, Offhand = 2 };

    [[nodiscard]] inline constexpr HandMask deliveryHand(
        const bool ownedCarry, const bool firingHandIsLeft,
        const bool nativePrimaryIsLeft) noexcept
    {
        // Owned carry/solver publishes already-recoiled weapon AND hand targets.
        // FRIK must not add a second kick to those targets next skeleton frame.
        return ownedCarry ? HandMask::None :
               firingHandIsLeft == nativePrimaryIsLeft ? HandMask::Primary : HandMask::Offhand;
    }

    [[nodiscard]] inline constexpr bool needsOneHandPresentation(
        const bool kickActive, const bool needsNeutralFrame) noexcept
    {
        return kickActive || needsNeutralFrame;
    }

    struct SampleIdentity
    {
        std::uintptr_t weaponNode{ 0 };  // Identity only; never dereferenced.
        std::uint64_t weaponGeneration{ 0 };
        std::uint64_t equippedOwnership{ 0 };
        Profile profile{ Profile::OneHand };
        bool firingHandIsLeft{ false };
        bool nativePrimaryIsLeft{ false };
        bool fullTwoHanded{ false };
        bool oneHanded{ false };

        [[nodiscard]] bool operator==(const SampleIdentity&) const = default;
    };

    // One callback ticket per ROCK update. State/role changes cannot replay a
    // kick captured for another owner, profile, native hand mapping, or solve.
    struct SampleTicket
    {
        SampleIdentity identity{};
        std::uint64_t sequence{ 0 };
        std::uint64_t observed{ 0 };
        bool valid{ false };
        bool ready{ false };

        void invalidate() noexcept { valid = false; ready = false; }
        void beginUpdate(const bool enabled) noexcept
        {
            if (!enabled) {
                invalidate();
            }
            ready = valid && sequence != observed;
            observed = sequence;
        }
        [[nodiscard]] bool consume(const SampleIdentity& current) noexcept
        {
            const bool accepted = ready && valid && identity == current;
            ready = false;
            return accepted;
        }
    };
}

namespace rock::weapon_recoil_authority_math
{
    // Numerical identity tolerance, not a recoil strength/dead-zone setting.
    template <class Transform>
    [[nodiscard]] inline bool hasKick(const Transform& local) noexcept
    {
        constexpr float epsilon = 0.00001f;
        if (std::abs(local.translate.x) > epsilon || std::abs(local.translate.y) > epsilon ||
            std::abs(local.translate.z) > epsilon) {
            return true;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (std::abs(local.rotate.entry[row][column] - (row == column ? 1.0f : 0.0f)) > epsilon) {
                    return true;
                }
            }
        }
        return false;
    }

    template <class Transform>
    inline void applyOneHandKick(const Transform& delta, const Transform& weaponBase,
        const Transform& handBase, Transform& weaponTarget, Transform& handTarget)
    {
        weaponTarget = transform_math::composeTransforms(delta, weaponBase);
        handTarget = transform_math::composeTransforms(delta, handBase);
    }

    template <class Transform>
    [[nodiscard]] inline bool tryBuildControlledKick(
        const Transform& nativeKickLocal,
        const weapon_recoil_policy::ProfileGains gains,
        Transform& outControlledKickLocal)
    {
        const auto isRigidKick = [](const Transform& local) {
            if (!std::isfinite(local.translate.x) ||
                !std::isfinite(local.translate.y) ||
                !std::isfinite(local.translate.z) ||
                !std::isfinite(local.scale) ||
                std::abs(local.scale - 1.0f) > 0.01f) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(local.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            for (int row = 0; row < 3; ++row) {
                for (int other = row; other < 3; ++other) {
                    float dot = 0.0f;
                    for (int column = 0; column < 3; ++column) {
                        dot += local.rotate.entry[row][column] * local.rotate.entry[other][column];
                    }
                    if (std::abs(dot - (row == other ? 1.0f : 0.0f)) > 0.01f) {
                        return false;
                    }
                }
            }
            const auto& r = local.rotate.entry;
            const float determinant = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
            return std::abs(determinant - 1.0f) <= 0.01f;
        };

        outControlledKickLocal =
            transform_math::makeIdentityTransform<Transform>();
        if (!isRigidKick(nativeKickLocal) ||
            !std::isfinite(gains.translation) || gains.translation < 0.0f ||
            !std::isfinite(gains.rotation) || gains.rotation < 0.0f) {
            return false;
        }

        float nativeQuaternion[4]{};
        transform_math::niRowsToHavokQuaternion(
            nativeKickLocal.rotate,
            nativeQuaternion);
        float quaternionLengthSquared = 0.0f;
        for (const float component : nativeQuaternion) {
            if (!std::isfinite(component)) {
                return false;
            }
            quaternionLengthSquared += component * component;
        }
        if (!std::isfinite(quaternionLengthSquared) ||
            quaternionLengthSquared <= 0.000001f) {
            return false;
        }
        const float inverseQuaternionLength =
            1.0f / std::sqrt(quaternionLengthSquared);
        for (float& component : nativeQuaternion) {
            component *= inverseQuaternionLength;
        }
        if (nativeQuaternion[3] < 0.0f) {
            for (float& component : nativeQuaternion) {
                component = -component;
            }
        }

        const float vectorLength = std::sqrt(
            nativeQuaternion[0] * nativeQuaternion[0] +
            nativeQuaternion[1] * nativeQuaternion[1] +
            nativeQuaternion[2] * nativeQuaternion[2]);
        float controlledQuaternion[4]{ 0.0f, 0.0f, 0.0f, 1.0f };
        if (std::isfinite(vectorLength) && vectorLength > 0.000001f) {
            const float halfAngle = std::atan2(
                vectorLength,
                std::clamp(nativeQuaternion[3], -1.0f, 1.0f));
            const float controlledHalfAngle =
                halfAngle * gains.rotation;
            const float axisScale =
                std::sin(controlledHalfAngle) / vectorLength;
            controlledQuaternion[0] = nativeQuaternion[0] * axisScale;
            controlledQuaternion[1] = nativeQuaternion[1] * axisScale;
            controlledQuaternion[2] = nativeQuaternion[2] * axisScale;
            controlledQuaternion[3] = std::cos(controlledHalfAngle);
        }

        outControlledKickLocal.rotate =
            transform_math::havokQuaternionToNiRows<
                decltype(outControlledKickLocal.rotate)>(
                controlledQuaternion);
        outControlledKickLocal.translate = {
            nativeKickLocal.translate.x *
                gains.translation,
            nativeKickLocal.translate.y *
                gains.translation,
            nativeKickLocal.translate.z *
                gains.translation,
        };
        outControlledKickLocal.scale = 1.0f;
        return isRigidKick(outControlledKickLocal);
    }

    template <class Transform>
    [[nodiscard]] inline Transform mirrorLocalAcrossSagittal(
        const Transform& local)
    {
        Transform mirror = transform_math::makeIdentityTransform<Transform>();
        mirror.rotate.entry[0][0] = -1.0f;
        return transform_math::composeTransforms(
            mirror,
            transform_math::composeTransforms(local, mirror));
    }

    template <class Transform>
    [[nodiscard]] inline Transform resolveWorldDelta(
        const Transform& nativeKickLocal,
        const Transform& nativeKickParentWorld,
        const Transform& nativePrimaryWandWorld,
        const Transform& nativeOffhandWandWorld,
        const bool targetIsNativeOffhand)
    {
        // Conjugating identity at distant world coordinates can introduce
        // cancellation noise. A neutral sample must remain exactly neutral.
        if (!hasKick(nativeKickLocal)) {
            return transform_math::makeIdentityTransform<Transform>();
        }
        Transform kickParentWorld = nativeKickParentWorld;
        Transform kickLocal = nativeKickLocal;
        if (targetIsNativeOffhand) {
            const Transform kickParentInPrimaryWand =
                transform_math::composeTransforms(
                    transform_math::invertTransform(nativePrimaryWandWorld),
                    nativeKickParentWorld);
            kickParentWorld = transform_math::composeTransforms(
                nativeOffhandWandWorld,
                mirrorLocalAcrossSagittal(kickParentInPrimaryWand));
            kickLocal = mirrorLocalAcrossSagittal(nativeKickLocal);
        }

        return transform_math::composeTransforms(
            kickParentWorld,
            transform_math::composeTransforms(
                kickLocal,
                transform_math::invertTransform(kickParentWorld)));
    }
}

