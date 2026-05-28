#pragma once

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::grab_authority_experiment
{
    enum class Mode : std::uint8_t
    {
        CurrentHybridBaseline = 0,
        RawRawAuthorityFrame = 10,
        ProxyProxyAuthorityFrame = 11,
        HybridHardSwitchOnMismatch = 20,
        HybridSmoothBlendOnMismatch = 21,
        HybridStartupHardSwitchOnMismatch = 22,
        HybridStartupSmoothBlendOnMismatch = 23,
        HybridBlendUntilTouchHeld = 24,
        BodyLocalPivotBTruth = 30,
        BodyLocalPivotBTruthWithHybridBlend = 31,
        AuthorityLocalPivotAFreeze = 40,
        AuthorityLocalPivotAFreezeWithHybridBlend = 41,
        UnifiedAuthorityLocalAAndBodyLocalB = 50,
        UnifiedAuthorityLocalAAndBodyLocalBWithBlend = 51,
    };

    inline int modeId(Mode mode) noexcept
    {
        return static_cast<int>(mode);
    }

    inline bool isKnownModeId(int mode) noexcept
    {
        switch (mode) {
        case 0:
        case 10:
        case 11:
        case 20:
        case 21:
        case 22:
        case 23:
        case 24:
        case 30:
        case 31:
        case 40:
        case 41:
        case 50:
        case 51:
            return true;
        default:
            return false;
        }
    }

    inline Mode modeFromId(int mode) noexcept
    {
        return isKnownModeId(mode) ? static_cast<Mode>(mode) : Mode::CurrentHybridBaseline;
    }

    inline const char* modeName(Mode mode) noexcept
    {
        switch (mode) {
        case Mode::CurrentHybridBaseline:
            return "CurrentHybridBaseline";
        case Mode::RawRawAuthorityFrame:
            return "RawRawAuthorityFrame";
        case Mode::ProxyProxyAuthorityFrame:
            return "ProxyProxyAuthorityFrame";
        case Mode::HybridHardSwitchOnMismatch:
            return "HybridHardSwitchOnMismatch";
        case Mode::HybridSmoothBlendOnMismatch:
            return "HybridSmoothBlendOnMismatch";
        case Mode::HybridStartupHardSwitchOnMismatch:
            return "HybridStartupHardSwitchOnMismatch";
        case Mode::HybridStartupSmoothBlendOnMismatch:
            return "HybridStartupSmoothBlendOnMismatch";
        case Mode::HybridBlendUntilTouchHeld:
            return "HybridBlendUntilTouchHeld";
        case Mode::BodyLocalPivotBTruth:
            return "BodyLocalPivotBTruth";
        case Mode::BodyLocalPivotBTruthWithHybridBlend:
            return "BodyLocalPivotBTruthWithHybridBlend";
        case Mode::AuthorityLocalPivotAFreeze:
            return "AuthorityLocalPivotAFreeze";
        case Mode::AuthorityLocalPivotAFreezeWithHybridBlend:
            return "AuthorityLocalPivotAFreezeWithHybridBlend";
        case Mode::UnifiedAuthorityLocalAAndBodyLocalB:
            return "UnifiedAuthorityLocalAAndBodyLocalB";
        case Mode::UnifiedAuthorityLocalAAndBodyLocalBWithBlend:
            return "UnifiedAuthorityLocalAAndBodyLocalBWithBlend";
        default:
            return "CurrentHybridBaseline";
        }
    }

    inline bool usesBodyLocalPivotBTruth(Mode mode) noexcept
    {
        switch (mode) {
        case Mode::BodyLocalPivotBTruth:
        case Mode::BodyLocalPivotBTruthWithHybridBlend:
        case Mode::UnifiedAuthorityLocalAAndBodyLocalB:
        case Mode::UnifiedAuthorityLocalAAndBodyLocalBWithBlend:
            return true;
        default:
            return false;
        }
    }

    inline bool usesAuthorityLocalPivotAFreeze(Mode mode) noexcept
    {
        switch (mode) {
        case Mode::AuthorityLocalPivotAFreeze:
        case Mode::AuthorityLocalPivotAFreezeWithHybridBlend:
        case Mode::UnifiedAuthorityLocalAAndBodyLocalB:
        case Mode::UnifiedAuthorityLocalAAndBodyLocalBWithBlend:
            return true;
        default:
            return false;
        }
    }

    inline bool usesHardMismatchSwitch(Mode mode) noexcept
    {
        return mode == Mode::HybridHardSwitchOnMismatch ||
               mode == Mode::HybridStartupHardSwitchOnMismatch;
    }

    inline bool usesSmoothMismatchBlend(Mode mode) noexcept
    {
        switch (mode) {
        case Mode::HybridSmoothBlendOnMismatch:
        case Mode::HybridStartupSmoothBlendOnMismatch:
        case Mode::HybridBlendUntilTouchHeld:
        case Mode::BodyLocalPivotBTruthWithHybridBlend:
        case Mode::AuthorityLocalPivotAFreezeWithHybridBlend:
        case Mode::UnifiedAuthorityLocalAAndBodyLocalBWithBlend:
            return true;
        default:
            return false;
        }
    }

    inline bool isStartupLimitedMode(Mode mode) noexcept
    {
        return mode == Mode::HybridStartupHardSwitchOnMismatch ||
               mode == Mode::HybridStartupSmoothBlendOnMismatch;
    }

    inline bool isUntilTouchHeldMode(Mode mode) noexcept
    {
        return mode == Mode::HybridBlendUntilTouchHeld;
    }

    struct Policy
    {
        Mode mode = Mode::CurrentHybridBaseline;
        float mismatchBlendStartDegrees = 15.0f;
        float mismatchBlendFullDegrees = 45.0f;
        float mismatchBlendMaxWeight = 1.0f;
        float startupBlendMaxSeconds = 0.35f;
    };

    inline Policy sanitizePolicy(Policy policy) noexcept
    {
        policy.mismatchBlendStartDegrees =
            std::clamp(std::isfinite(policy.mismatchBlendStartDegrees) ? policy.mismatchBlendStartDegrees : 15.0f, 0.0f, 180.0f);
        policy.mismatchBlendFullDegrees =
            std::clamp(std::isfinite(policy.mismatchBlendFullDegrees) ? policy.mismatchBlendFullDegrees : 45.0f, policy.mismatchBlendStartDegrees, 180.0f);
        policy.mismatchBlendMaxWeight =
            std::clamp(std::isfinite(policy.mismatchBlendMaxWeight) ? policy.mismatchBlendMaxWeight : 1.0f, 0.0f, 1.0f);
        policy.startupBlendMaxSeconds =
            std::clamp(std::isfinite(policy.startupBlendMaxSeconds) ? policy.startupBlendMaxSeconds : 0.35f, 0.0f, 5.0f);
        return policy;
    }

    inline bool modeAllowsMismatchBlendNow(Mode mode, float grabAgeSeconds, bool touchHeld, float startupBlendMaxSeconds) noexcept
    {
        if (isStartupLimitedMode(mode)) {
            const float age = std::isfinite(grabAgeSeconds) ? (std::max)(0.0f, grabAgeSeconds) : 0.0f;
            return age <= (std::max)(0.0f, startupBlendMaxSeconds);
        }
        if (isUntilTouchHeldMode(mode)) {
            return !touchHeld;
        }
        return true;
    }

    template <class Matrix>
    inline float rotationDeltaDegrees(const Matrix& a, const Matrix& b)
    {
        const Matrix delta = transform_math::multiplyStoredRotations(transform_math::transposeRotation(a), b);
        const float cosTheta = std::clamp((delta.entry[0][0] + delta.entry[1][1] + delta.entry[2][2] - 1.0f) * 0.5f, -1.0f, 1.0f);
        return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
    }

    struct Quaternion
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;
    };

    inline Quaternion normalize(Quaternion q) noexcept
    {
        const float length = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (!std::isfinite(length) || length <= 0.000001f) {
            return {};
        }

        const float invLength = 1.0f / length;
        q.x *= invLength;
        q.y *= invLength;
        q.z *= invLength;
        q.w *= invLength;
        return q;
    }

    inline float dot(const Quaternion& lhs, const Quaternion& rhs) noexcept
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
    }

    template <class Matrix>
    inline Quaternion matrixToQuaternion(const Matrix& matrix)
    {
        float values[4]{};
        transform_math::niRowsToHavokQuaternion(matrix, values);
        return normalize(Quaternion{ values[0], values[1], values[2], values[3] });
    }

    template <class Matrix>
    inline Matrix quaternionToMatrix(const Quaternion& q)
    {
        const Quaternion normalized = normalize(q);
        const float values[4]{ normalized.x, normalized.y, normalized.z, normalized.w };
        return transform_math::havokQuaternionToNiRows<Matrix>(values);
    }

    inline Quaternion slerp(Quaternion from, Quaternion to, float alpha) noexcept
    {
        alpha = std::clamp(std::isfinite(alpha) ? alpha : 0.0f, 0.0f, 1.0f);
        from = normalize(from);
        to = normalize(to);

        float cosine = dot(from, to);
        if (cosine < 0.0f) {
            to.x = -to.x;
            to.y = -to.y;
            to.z = -to.z;
            to.w = -to.w;
            cosine = -cosine;
        }

        if (cosine > 0.9995f) {
            return normalize(Quaternion{
                from.x + (to.x - from.x) * alpha,
                from.y + (to.y - from.y) * alpha,
                from.z + (to.z - from.z) * alpha,
                from.w + (to.w - from.w) * alpha,
            });
        }

        cosine = std::clamp(cosine, -1.0f, 1.0f);
        const float theta = std::acos(cosine);
        const float sinTheta = std::sin(theta);
        if (std::abs(sinTheta) <= 0.000001f) {
            return from;
        }

        const float fromScale = std::sin((1.0f - alpha) * theta) / sinTheta;
        const float toScale = std::sin(alpha * theta) / sinTheta;
        return normalize(Quaternion{
            from.x * fromScale + to.x * toScale,
            from.y * fromScale + to.y * toScale,
            from.z * fromScale + to.z * toScale,
            from.w * fromScale + to.w * toScale,
        });
    }

    inline float smoothMismatchBlendWeight(float mismatchDegrees, const Policy& policy) noexcept
    {
        const Policy sanitized = sanitizePolicy(policy);
        const float angle = std::isfinite(mismatchDegrees) ? mismatchDegrees : 0.0f;
        if (sanitized.mismatchBlendFullDegrees <= sanitized.mismatchBlendStartDegrees + 0.0001f) {
            return angle >= sanitized.mismatchBlendStartDegrees ? sanitized.mismatchBlendMaxWeight : 0.0f;
        }

        const float t = std::clamp(
            (angle - sanitized.mismatchBlendStartDegrees) /
                (sanitized.mismatchBlendFullDegrees - sanitized.mismatchBlendStartDegrees),
            0.0f,
            1.0f);
        return t * sanitized.mismatchBlendMaxWeight;
    }

    inline float hardMismatchBlendWeight(float mismatchDegrees, const Policy& policy) noexcept
    {
        const Policy sanitized = sanitizePolicy(policy);
        const float angle = std::isfinite(mismatchDegrees) ? mismatchDegrees : 0.0f;
        return angle >= sanitized.mismatchBlendStartDegrees ? sanitized.mismatchBlendMaxWeight : 0.0f;
    }

    template <class Transform>
    struct AuthorityFrameEvaluation
    {
        Transform frame{};
        float rawProxyMismatchDegrees = 0.0f;
        float blendWeight = 0.0f;
        const char* rotationSource = "raw";
    };

    template <class Transform>
    inline Transform makeCurrentHybridFrame(const Transform& rawHandWorld, const Transform& proxyWorld)
    {
        Transform result = proxyWorld;
        result.rotate = rawHandWorld.rotate;
        result.scale = rawHandWorld.scale;
        return result;
    }

    template <class Transform>
    inline AuthorityFrameEvaluation<Transform> makeAuthorityFrame(
        const Transform& rawHandWorld,
        const Transform& proxyWorld,
        Policy policy,
        float grabAgeSeconds,
        bool touchHeld)
    {
        policy = sanitizePolicy(policy);

        AuthorityFrameEvaluation<Transform> result{};
        result.frame = makeCurrentHybridFrame(rawHandWorld, proxyWorld);
        result.rawProxyMismatchDegrees = rotationDeltaDegrees(rawHandWorld.rotate, proxyWorld.rotate);

        switch (policy.mode) {
        case Mode::RawRawAuthorityFrame:
            result.frame = rawHandWorld;
            result.rotationSource = "rawRaw";
            return result;
        case Mode::ProxyProxyAuthorityFrame:
            result.frame = proxyWorld;
            result.rotationSource = "proxyProxy";
            return result;
        default:
            break;
        }

        const bool hardSwitch = usesHardMismatchSwitch(policy.mode);
        const bool smoothBlend = usesSmoothMismatchBlend(policy.mode);
        if ((hardSwitch || smoothBlend) &&
            modeAllowsMismatchBlendNow(policy.mode, grabAgeSeconds, touchHeld, policy.startupBlendMaxSeconds)) {
            result.blendWeight = hardSwitch ?
                hardMismatchBlendWeight(result.rawProxyMismatchDegrees, policy) :
                smoothMismatchBlendWeight(result.rawProxyMismatchDegrees, policy);
            if (result.blendWeight > 0.0001f) {
                using Matrix = decltype(result.frame.rotate);
                result.frame.rotate = quaternionToMatrix<Matrix>(
                    slerp(matrixToQuaternion(rawHandWorld.rotate), matrixToQuaternion(proxyWorld.rotate), result.blendWeight));
                result.rotationSource = hardSwitch ? "proxyHardSwitch" : "rawProxySmoothBlend";
            }
        }

        return result;
    }
}
