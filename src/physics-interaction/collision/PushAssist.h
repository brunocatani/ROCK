#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace RE
{
    class NiPoint3;
    class hknpWorld;
}

namespace rock::push_assist
{
    enum class PushAssistSkipReason : std::uint8_t
    {
        None,
        Disabled,
        Cooldown,
        BelowMinSpeed,
        InvalidImpulse,
    };

    template <class Vec3>
    struct PushAssistInput
    {
        bool enabled = true;
        Vec3 sourceVelocity{};
        float minSpeed = 0.0f;
        float maxImpulse = 0.0f;
        float layerMultiplier = 1.0f;
        float cooldownRemainingSeconds = 0.0f;
    };

    template <class Vec3>
    struct PushAssistResult
    {
        bool apply = false;
        Vec3 impulse{};
        float impulseMagnitude = 0.0f;
        PushAssistSkipReason skipReason = PushAssistSkipReason::None;
    };

    template <class Vec3>
    inline float length(const Vec3& value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    }

    template <class Vec3>
    inline PushAssistResult<Vec3> computePushImpulse(const PushAssistInput<Vec3>& input)
    {
        PushAssistResult<Vec3> result{};
        if (!input.enabled) {
            result.skipReason = PushAssistSkipReason::Disabled;
            return result;
        }
        if (input.cooldownRemainingSeconds > 0.0f) {
            result.skipReason = PushAssistSkipReason::Cooldown;
            return result;
        }

        const float speed = length(input.sourceVelocity);
        if (!std::isfinite(speed) || speed < input.minSpeed) {
            result.skipReason = PushAssistSkipReason::BelowMinSpeed;
            return result;
        }

        const float unclampedMagnitude = speed * (std::max)(0.0f, input.layerMultiplier);
        const float maxImpulse = (std::max)(0.0f, input.maxImpulse);
        const float impulseMagnitude = maxImpulse > 0.0f ? (std::min)(unclampedMagnitude, maxImpulse) : unclampedMagnitude;
        if (!std::isfinite(impulseMagnitude) || impulseMagnitude <= 0.0f) {
            result.skipReason = PushAssistSkipReason::InvalidImpulse;
            return result;
        }

        const float invSpeed = 1.0f / speed;
        result.impulse.x = input.sourceVelocity.x * invSpeed * impulseMagnitude;
        result.impulse.y = input.sourceVelocity.y * invSpeed * impulseMagnitude;
        result.impulse.z = input.sourceVelocity.z * invSpeed * impulseMagnitude;
        result.impulseMagnitude = impulseMagnitude;
        result.apply = true;
        return result;
    }

    template <class Vec3>
    inline Vec3 computeVelocityDeltaFromImpulse(const Vec3& impulse, float inverseMass)
    {
        Vec3 result{};
        if (!std::isfinite(inverseMass) || inverseMass <= 0.0f) {
            return result;
        }
        result.x = impulse.x * inverseMass;
        result.y = impulse.y * inverseMass;
        result.z = impulse.z * inverseMass;
        return result;
    }

    bool applyLinearImpulse(void* collisionObject, const RE::NiPoint3& impulseHavok);

    bool applyLinearVelocityDeltaDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiPoint3& velocityDeltaHavok);
}
