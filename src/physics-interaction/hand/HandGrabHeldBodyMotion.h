// Velocity and mass of held bodies: writing a held body's velocity, reading the
// mass of a held body set, and classifying what a held body touched.
#pragma once

#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock::hand_grab_internal
{
    inline void setHeldVelocity(RE::hknpWorld* world, RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds, const RE::NiPoint3& linearVelocity,
        const RE::NiPoint3& angularVelocity, bool overrideAngularVelocity, float angularVelocityKeep = 1.0f,
        bool includeConnectedLinearVelocity = true, bool includeConnectedAngularVelocity = true)
    {
        if (!world) {
            return;
        }

        constexpr std::size_t kMaxVelocityMotionSlots = 96;
        std::array<std::uint32_t, kMaxVelocityMotionSlots> updatedMotionSlots{};
        std::size_t updatedMotionSlotCount = 0;

        auto alreadyUpdated = [&updatedMotionSlots, &updatedMotionSlotCount](std::uint32_t motionIndex) {
            for (std::size_t i = 0; i < updatedMotionSlotCount; ++i) {
                if (updatedMotionSlots[i] == motionIndex) {
                    return true;
                }
            }
            return false;
        };

        auto setBody = [&](std::uint32_t bodyId, bool primaryBody) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }

            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
            if (!body) {
                return;
            }

            const std::uint32_t motionIndex = body->motionIndex;
            if (!body_frame::hasUsableMotionIndex(motionIndex) || alreadyUpdated(motionIndex)) {
                return;
            }

            if (updatedMotionSlotCount >= updatedMotionSlots.size()) {
                return;
            }

            auto* motion = havok_runtime::getMotion(world, motionIndex);
            if (!motion) {
                return;
            }

            updatedMotionSlots[updatedMotionSlotCount++] = motionIndex;
            const bool applyLinearForBody = primaryBody || includeConnectedLinearVelocity;
            const float angularKeep =
                std::clamp(std::isfinite(angularVelocityKeep) ? angularVelocityKeep : 1.0f, 0.0f, 1.0f);
            const bool overrideAngularForBody =
                overrideAngularVelocity && (primaryBody || includeConnectedAngularVelocity);
            const RE::hkVector4f linearHavok = applyLinearForBody
                ? RE::hkVector4f{ linearVelocity.x, linearVelocity.y, linearVelocity.z, 0.0f }
                : RE::hkVector4f{ motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z, 0.0f };
            const RE::hkVector4f angularHavok = overrideAngularForBody
                ? RE::hkVector4f{ angularVelocity.x, angularVelocity.y, angularVelocity.z, 0.0f }
                : RE::hkVector4f{ motion->angularVelocity.x * angularKeep, motion->angularVelocity.y * angularKeep,
                      motion->angularVelocity.z * angularKeep, 0.0f };
            havok_runtime::setBodyVelocityDeferred(world, bodyId, linearHavok, angularHavok);
        };

        setBody(primaryBodyId.value, true);
        if (includeConnectedLinearVelocity || includeConnectedAngularVelocity) {
            for (const auto bodyId : heldBodyIds) {
                setBody(bodyId, false);
            }
        }
    }

    inline float readBodyMass(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        if (!world || bodyId.value == INVALID_BODY_ID) {
            return 0.0f;
        }

        auto* motion = havok_runtime::getBodyMotion(world, bodyId);
        if (!motion) {
            return 0.0f;
        }

        const auto packedInvMass = static_cast<std::int16_t>(motion->packedInverseInertia[3]);
        if (packedInvMass == 0) {
            return 0.0f;
        }

        return grab_mass_policy::massFromInverseMass(unpackBfloat16(packedInvMass));
    }

    struct HeldBodyMassSummary
    {
        float primaryMass = 0.0f;
        float aggregateMass = 0.0f;
        std::uint32_t sampledBodies = 0;
        std::uint32_t uniqueMotions = 0;

        [[nodiscard]] float motorMass() const noexcept
        {
            if (std::isfinite(aggregateMass) && aggregateMass > 0.0f) {
                return aggregateMass;
            }
            return (std::isfinite(primaryMass) && primaryMass > 0.0f) ? primaryMass : 0.0f;
        }
    };

    inline HeldBodyMassSummary readHeldBodyMassSummary(RE::hknpWorld* world, RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds, bool includeConnectedBodies = true)
    {
        /*
             * Dynamic grab owns one held object even when FO4VR exposes that
             * object as several hknp bodies. Lifecycle, inertia normalization,
             * release velocity, and nearby damping already operate on the whole
             * accepted body set. The motor mass budget must use the same object
             * scope, with unique-motion dedupe, so multipart loose weapons are
             * not budgeted from whichever child body happened to be selected.
             */
        HeldBodyMassSummary summary{};
        summary.primaryMass = readBodyMass(world, primaryBodyId);
        if (!world) {
            summary.aggregateMass = summary.primaryMass;
            return summary;
        }

        constexpr std::size_t kMaxMassMotionSlots = 96;
        std::array<std::uint32_t, kMaxMassMotionSlots> sampledMotionSlots{};
        std::size_t sampledMotionSlotCount = 0;

        auto motionAlreadySampled = [&sampledMotionSlots, &sampledMotionSlotCount](std::uint32_t motionIndex) {
            for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                if (sampledMotionSlots[i] == motionIndex) {
                    return true;
                }
            }
            return false;
        };

        auto sampleBody = [&](std::uint32_t rawBodyId) {
            if (rawBodyId == INVALID_BODY_ID) {
                return;
            }

            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ rawBodyId });
            if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex) ||
                motionAlreadySampled(body->motionIndex)) {
                return;
            }
            if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                return;
            }

            const float mass = readBodyMass(world, RE::hknpBodyId{ rawBodyId });
            if (!std::isfinite(mass) || mass <= 0.0f) {
                return;
            }

            sampledMotionSlots[sampledMotionSlotCount++] = body->motionIndex;
            summary.aggregateMass += mass;
            ++summary.sampledBodies;
            summary.uniqueMotions = static_cast<std::uint32_t>(sampledMotionSlotCount);
        };

        sampleBody(primaryBodyId.value);
        if (includeConnectedBodies) {
            for (const auto bodyId : heldBodyIds) {
                sampleBody(bodyId);
            }
        }

        if (!(std::isfinite(summary.aggregateMass) && summary.aggregateMass > 0.0f)) {
            summary.aggregateMass = summary.primaryMass;
        }
        return summary;
    }

    inline held_object_contact_policy::HeldContactOtherMotion classifyHeldContactOtherMotion(RE::hknpWorld* world,
        std::uint32_t bodyId)
    {
        if (!world || bodyId == INVALID_BODY_ID) {
            return held_object_contact_policy::HeldContactOtherMotion::Unknown;
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
        if (!motion) {
            return held_object_contact_policy::HeldContactOtherMotion::Unknown;
        }

        const float mass = readBodyMass(world, RE::hknpBodyId{ bodyId });
        return mass > 0.0f ? held_object_contact_policy::HeldContactOtherMotion::Dynamic
                           : held_object_contact_policy::HeldContactOtherMotion::FixedOrStatic;
    }
}
