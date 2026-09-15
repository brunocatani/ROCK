#pragma once

#include "physics-interaction/grab/GrabCore.h"

namespace rock::grab_offset_acquisition
{
    // Main-thread capture/update state. Only the resulting proxy transform is
    // queued to physics. The final body relation and both local joint pivots
    // remain fixed; the object stays dynamic throughout acquisition.
    template <class Transform>
    struct Transition
    {
        Transform startBodyInProxy{};
        float elapsedSeconds{ 0.0f };
        float durationSeconds{ 0.0f };
        bool active{ false };
    };

    template <class Matrix>
    inline float rotationAngleRadians(const Matrix& from, const Matrix& to)
    {
        float a[4]{}, b[4]{};
        transform_math::niRowsToHavokQuaternion(from, a);
        transform_math::niRowsToHavokQuaternion(to, b);
        double dot = 0.0, normA = 0.0, normB = 0.0;
        for (int i = 0; i < 4; ++i) {
            dot += static_cast<double>(a[i]) * b[i];
            normA += static_cast<double>(a[i]) * a[i];
            normB += static_cast<double>(b[i]) * b[i];
        }
        return static_cast<float>(2.0 * std::acos(std::clamp(std::abs(dot) / std::sqrt(normA * normB), 0.0, 1.0)));
    }

    template <class Transform, class Vector>
    inline Transform interpolateAtGrip(const Transform& start, const Transform& target,
        const Vector& gripBodyLocal, float fraction)
    {
        const float t = std::clamp(fraction, 0.0f, 1.0f);
        if (t <= 0.0f) return start;
        if (t >= 1.0f) return target;

        float a[4]{}, b[4]{}, blended[4]{};
        transform_math::niRowsToHavokQuaternion(start.rotate, a);
        transform_math::niRowsToHavokQuaternion(target.rotate, b);
        float dot = 0.0f;
        for (int i = 0; i < 4; ++i) dot += a[i] * b[i];
        if (dot < 0.0f) {
            for (float& component : b) component = -component;
            dot = -dot;
        }
        dot = std::clamp(dot, 0.0f, 1.0f);
        float fromWeight = 1.0f - t;
        float toWeight = t;
        if (dot < 0.9995f) {
            const float angle = std::acos(dot);
            const float inverseSin = 1.0f / std::sin(angle);
            fromWeight = std::sin((1.0f - t) * angle) * inverseSin;
            toWeight = std::sin(t * angle) * inverseSin;
        }
        for (int i = 0; i < 4; ++i) blended[i] = a[i] * fromWeight + b[i] * toWeight;
        Transform result = target;
        result.rotate = transform_math::havokQuaternionToNiRows<decltype(result.rotate)>(blended);
        result.scale = start.scale + (target.scale - start.scale) * t;
        const auto firstGrip = transform_math::localPointToWorld(start, gripBodyLocal);
        const auto lastGrip = transform_math::localPointToWorld(target, gripBodyLocal);
        const auto rotatedGrip = transform_math::localPointToWorld(result, gripBodyLocal);
        result.translate.x += firstGrip.x + (lastGrip.x - firstGrip.x) * t - rotatedGrip.x;
        result.translate.y += firstGrip.y + (lastGrip.y - firstGrip.y) * t - rotatedGrip.y;
        result.translate.z += firstGrip.z + (lastGrip.z - firstGrip.z) * t - rotatedGrip.z;
        return result;
    }

    template <class Transform, class Vector>
    inline Transition<Transform> begin(const Transform& rawProxyWorld,
        const Transform& bodyWorld, const Transform& targetBodyInProxy, const Vector& gripBodyLocal)
    {
        Transition<Transform> state{};
        state.startBodyInProxy = grab_frame_math::objectInGeneratedProxyLocalSpace(rawProxyWorld, bodyWorld);
        const auto firstGrip = transform_math::localPointToWorld(state.startBodyInProxy, gripBodyLocal);
        const auto lastGrip = transform_math::localPointToWorld(targetBodyInProxy, gripBodyLocal);
        const float distance = std::sqrt(vector_math::lengthSquared(lastGrip - firstGrip)) * std::abs(rawProxyWorld.scale);
        const float angle = rotationAngleRadians(state.startBodyInProxy.rotate, targetBodyInProxy.rotate);
        if (!std::isfinite(distance) || !std::isfinite(angle)) return state;
        if (distance < 0.001f && angle < 0.0001f) return state;
        // A smoothstep takes 1.5 times its average speed at the midpoint.
        // Size the interval from both errors, so pure rotation also receives
        // a continuous target and a tiny translation never requests a swing.
        state.durationSeconds = (std::max)({ 0.10f, distance / 120.0f, angle / 6.283185307f });
        state.active = true;
        return state;
    }

    template <class Transform, class Vector>
    inline Transform advance(Transition<Transform>& state, const Transform& rawProxyWorld,
        const Transform& targetBodyInProxy, const Vector& gripBodyLocal, float deltaSeconds)
    {
        if (!state.active) return rawProxyWorld;
        if (std::isfinite(deltaSeconds) && deltaSeconds > 0.0f) {
            state.elapsedSeconds = (std::min)(state.durationSeconds, state.elapsedSeconds + deltaSeconds);
        }
        if (state.elapsedSeconds >= state.durationSeconds) {
            state.active = false;
            return rawProxyWorld;
        }
        const float t = state.elapsedSeconds / state.durationSeconds;
        const float eased = t * t * (3.0f - 2.0f * t);
        const auto bodyInProxy = interpolateAtGrip(state.startBodyInProxy, targetBodyInProxy, gripBodyLocal, eased);
        const auto bodyWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(rawProxyWorld, bodyInProxy);
        return grab_frame_math::generatedProxyFromObjectWorld(bodyWorld, targetBodyInProxy);
    }
}
