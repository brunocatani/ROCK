#pragma once

#include "physics-interaction/grab/GrabCore.h"

namespace rock::grab_acquisition
{
    inline float rotationDegrees(const RE::NiTransform& from, const RE::NiTransform& to)
    {
        float a[4], b[4];
        transform_math::niRowsToHavokQuaternion(from.rotate, a);
        transform_math::niRowsToHavokQuaternion(to.rotate, b);
        float dot = 0.0f;
        for (int i = 0; i < 4; ++i) dot += a[i] * b[i];
        return 2.0f * std::acos(std::clamp(std::abs(dot), 0.0f, 1.0f)) * 57.295779513f;
    }

    // A body origin may be far from its grip. Interpolate the grip's position
    // and the body's rotation with one fraction, then recover the body origin.
    inline RE::NiTransform interpolateGrip(const RE::NiTransform& from,
        const RE::NiTransform& to, const RE::NiPoint3& bodyGrip, float fraction)
    {
        if (fraction <= 0.0f) return from;
        if (fraction >= 1.0f) return to;
        float a[4], b[4], q[4];
        transform_math::niRowsToHavokQuaternion(from.rotate, a);
        transform_math::niRowsToHavokQuaternion(to.rotate, b);
        float dot = 0.0f;
        for (int i = 0; i < 4; ++i) dot += a[i] * b[i];
        if (dot < 0.0f) {
            for (float& value : b) value = -value;
            dot = -dot;
        }
        float fromWeight = 1.0f - fraction;
        float toWeight = fraction;
        if (dot < 0.9995f) {
            const float angle = std::acos(std::clamp(dot, 0.0f, 1.0f));
            fromWeight = std::sin(fromWeight * angle) / std::sin(angle);
            toWeight = std::sin(toWeight * angle) / std::sin(angle);
        }
        for (int i = 0; i < 4; ++i) q[i] = fromWeight * a[i] + toWeight * b[i];
        RE::NiTransform result = to;
        result.rotate = transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(q);
        result.scale = from.scale + (to.scale - from.scale) * fraction;
        const auto firstGrip = transform_math::localPointToWorld(from, bodyGrip);
        const auto lastGrip = transform_math::localPointToWorld(to, bodyGrip);
        result.translate = {};
        const auto rotatedGrip = transform_math::localPointToWorld(result, bodyGrip);
        result.translate = firstGrip + (lastGrip - firstGrip) * fraction - rotatedGrip;
        return result;
    }

    // Game-thread state for one offset arrival. Physics receives only the
    // sampled proxy target; the frozen body relation and joint pivots never vary.
    struct Transition
    {
        RE::NiTransform initialBodyInPalm{};
        float durationSeconds = 0.0f;
        float elapsedSeconds = 0.0f;
        bool pending = false;

        bool targetComplete() const { return elapsedSeconds >= durationSeconds; }

        bool begin(const RE::NiTransform& palmProxy, const RE::NiTransform& bodyWorld,
            const RE::NiTransform& finalBodyInPalm, const RE::NiPoint3& bodyGrip)
        {
            *this = {};
            if (!grab_authority_frame_math::isFiniteTransform(palmProxy) || palmProxy.scale <= 0.0f ||
                !grab_authority_frame_math::isFiniteTransform(bodyWorld) || bodyWorld.scale <= 0.0f ||
                !grab_authority_frame_math::isFiniteTransform(finalBodyInPalm) || finalBodyInPalm.scale <= 0.0f ||
                !grab_authority_frame_math::isFiniteVector(bodyGrip)) return false;
            initialBodyInPalm = grab_frame_math::objectInGeneratedProxyLocalSpace(palmProxy, bodyWorld);
            const auto firstGrip = transform_math::localPointToWorld(initialBodyInPalm, bodyGrip);
            const auto finalGrip = transform_math::localPointToWorld(finalBodyInPalm, bodyGrip);
            const float distance = std::sqrt(vector_math::lengthSquared(finalGrip - firstGrip)) * palmProxy.scale;
            const float angle = rotationDegrees(initialBodyInPalm, finalBodyInPalm);
            if (!std::isfinite(distance) || !std::isfinite(angle)) return false;
            pending = distance > 0.01f || angle > 0.1f;
            // Average acquisition rates; smoothstep starts/stops without a
            // target-velocity discontinuity. Rotation alone also gets an interval.
            if (pending) durationSeconds = (std::max)({ 0.10f, distance / 120.0f, angle / 360.0f });
            return true;
        }

        RE::NiTransform sample(const RE::NiTransform& palmProxy,
            const RE::NiTransform& finalBodyInPalm, const RE::NiPoint3& bodyGrip, float deltaSeconds)
        {
            if (!pending || targetComplete()) return palmProxy;
            if (std::isfinite(deltaSeconds) && deltaSeconds > 0.0f)
                elapsedSeconds = (std::min)(durationSeconds, elapsedSeconds + deltaSeconds);
            if (targetComplete()) return palmProxy;
            const float t = elapsedSeconds / durationSeconds;
            const auto bodyInPalm = interpolateGrip(initialBodyInPalm, finalBodyInPalm, bodyGrip, t * t * (3.0f - 2.0f * t));
            const auto desiredBody = grab_frame_math::objectFromGeneratedProxyLocalSpace(palmProxy, bodyInPalm);
            auto proxy = transform_math::composeTransforms(desiredBody, transform_math::invertTransform(finalBodyInPalm));
            // Scene relations store rows; the generated physical proxy stores columns.
            proxy.rotate = transform_math::transposeRotation(proxy.rotate);
            return proxy;
        }

        bool finishIfSettled(float gripError, float angleError, float touchDistance)
        {
            if (!pending || !targetComplete() || !std::isfinite(gripError) || !std::isfinite(angleError) ||
                !std::isfinite(touchDistance) || gripError > (std::max)(0.1f, touchDistance) || angleError > 5.0f)
                return false;
            pending = false;
            return true;
        }
    };
}
