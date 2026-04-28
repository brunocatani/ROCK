#pragma once

#include "TransformMath.h"
#include "WeaponPartClassifier.h"
#include "WeaponSemanticTypes.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>

namespace frik::rock::weapon_primary_grip_math
{
    /*
     * HIGGS stores hand-to-weapon relationships at grab start and advances all
     * visible/collision state from that relationship. ROCK's weapon authority
     * follows the same pattern, but first chooses the primary grip from FO4VR
     * weapon mesh semantics so FRIK can solve the visible weapon from a real
     * grip frame instead of relying only on authored INI offsets.
     */

    enum class PrimaryGripSource : std::uint8_t
    {
        FallbackCurrent = 0,
        Mesh = 1,
        IniOverride = 2,
    };

    template <class Vector>
    struct PrimaryGripCandidate
    {
        std::string_view name{};
        WeaponPartKind partKind{ WeaponPartKind::Other };
        Vector localMin{};
        Vector localMax{};
        Vector localCenter{};
        std::size_t triangleCount{ 0 };
        Vector localSurfaceNormal{};
        bool hasSurfaceNormal{ false };
    };

    template <class Transform>
    struct PrimaryGripSelection
    {
        Transform gripWeaponLocal{};
        PrimaryGripSource source{ PrimaryGripSource::FallbackCurrent };
        float confidence{ 0.0f };
        std::size_t candidateIndex{ std::numeric_limits<std::size_t>::max() };
        bool selected{ false };
    };

    inline bool isSupportGripName(std::string_view name)
    {
        return weaponPartNameContains(name, "foregrip") ||
               weaponPartNameContains(name, "fore grip") ||
               weaponPartNameContains(name, "verticalgrip") ||
               weaponPartNameContains(name, "vertical grip") ||
               weaponPartNameContains(name, "angledgrip") ||
               weaponPartNameContains(name, "angled grip") ||
               weaponPartNameContains(name, "handguard") ||
               weaponPartNameContains(name, "hand guard") ||
               weaponPartNameContains(name, "forearm") ||
               weaponPartNameContains(name, "fore arm") ||
               weaponPartNameContains(name, "pump") ||
               weaponPartNameContains(name, "magwell") ||
               weaponPartNameContains(name, "mag well") ||
               weaponPartNameContains(name, "barrel") ||
               weaponPartNameContains(name, "stock") ||
               weaponPartNameContains(name, "support");
    }

    inline bool isPrimaryGripName(std::string_view name)
    {
        if (isSupportGripName(name)) {
            return false;
        }

        return weaponPartNameContains(name, "p-grip") ||
               weaponPartNameContains(name, "p_grip") ||
               weaponPartNameContains(name, "primarygrip") ||
               weaponPartNameContains(name, "primary grip") ||
               weaponPartNameContains(name, "pistolgrip") ||
               weaponPartNameContains(name, "pistol grip") ||
               weaponPartNameContains(name, "grip") ||
               weaponPartNameContains(name, "handle");
    }

    inline float scorePrimaryGripCandidateName(std::string_view name)
    {
        if (isSupportGripName(name)) {
            return 0.0f;
        }
        if (weaponPartNameContains(name, "p-grip") || weaponPartNameContains(name, "p_grip") ||
            weaponPartNameContains(name, "primarygrip") || weaponPartNameContains(name, "primary grip")) {
            return 0.98f;
        }
        if (weaponPartNameContains(name, "pistolgrip") || weaponPartNameContains(name, "pistol grip")) {
            return 0.96f;
        }
        if (weaponPartNameContains(name, "grip") || weaponPartNameContains(name, "handle")) {
            return 0.94f;
        }
        return 0.0f;
    }

    inline float scorePrimaryGripCandidateKind(WeaponPartKind partKind)
    {
        switch (partKind) {
        case WeaponPartKind::Grip:
            return 0.92f;
        case WeaponPartKind::Foregrip:
        case WeaponPartKind::Handguard:
        case WeaponPartKind::Pump:
        case WeaponPartKind::Magwell:
        case WeaponPartKind::Barrel:
            return 0.0f;
        case WeaponPartKind::Receiver:
            return 0.55f;
        case WeaponPartKind::Stock:
            return 0.35f;
        default:
            return 0.10f;
        }
    }

    template <class Vector>
    inline float scorePrimaryGripCandidate(const PrimaryGripCandidate<Vector>& candidate)
    {
        if (candidate.triangleCount == 0) {
            return 0.0f;
        }

        const float nameScore = scorePrimaryGripCandidateName(candidate.name);
        const float kindScore = scorePrimaryGripCandidateKind(candidate.partKind);
        const float baseScore = (std::max)(nameScore, kindScore);
        const float geometryBonus = candidate.triangleCount >= 12 ? 0.03f : 0.0f;
        return (std::min)(1.0f, baseScore + geometryBonus);
    }

    template <class Vector>
    inline Vector makeVector(float x, float y, float z)
    {
        return Vector{ x, y, z };
    }

    template <class Vector>
    inline Vector subtractVector(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    template <class Vector>
    inline float dotVector(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector crossVector(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x);
    }

    template <class Vector>
    inline Vector normalizeOrFallback(const Vector& vector, const Vector& fallback)
    {
        const float lenSq = dotVector(vector, vector);
        if (lenSq <= 0.000001f) {
            return fallback;
        }
        const float invLen = 1.0f / std::sqrt(lenSq);
        return makeVector<Vector>(vector.x * invLen, vector.y * invLen, vector.z * invLen);
    }

    template <class Vector>
    inline Vector selectBoundsAxis(const Vector& extent, bool longest)
    {
        const float x = std::abs(extent.x);
        const float y = std::abs(extent.y);
        const float z = std::abs(extent.z);

        if (longest) {
            if (x >= y && x >= z) {
                return makeVector<Vector>(1.0f, 0.0f, 0.0f);
            }
            if (y >= x && y >= z) {
                return makeVector<Vector>(0.0f, 1.0f, 0.0f);
            }
            return makeVector<Vector>(0.0f, 0.0f, 1.0f);
        }

        if (x <= y && x <= z) {
            return makeVector<Vector>(1.0f, 0.0f, 0.0f);
        }
        if (y <= x && y <= z) {
            return makeVector<Vector>(0.0f, 1.0f, 0.0f);
        }
        return makeVector<Vector>(0.0f, 0.0f, 1.0f);
    }

    template <class Matrix, class Vector>
    inline void setRotationRowsFromAxes(Matrix& matrix, const Vector& xAxis, const Vector& yAxis, const Vector& zAxis)
    {
        matrix.entry[0][0] = xAxis.x;
        matrix.entry[0][1] = xAxis.y;
        matrix.entry[0][2] = xAxis.z;
        matrix.entry[1][0] = yAxis.x;
        matrix.entry[1][1] = yAxis.y;
        matrix.entry[1][2] = yAxis.z;
        matrix.entry[2][0] = zAxis.x;
        matrix.entry[2][1] = zAxis.y;
        matrix.entry[2][2] = zAxis.z;
    }

    template <class Transform, class Vector>
    inline Transform makeGripFrameFromCandidate(const PrimaryGripCandidate<Vector>& candidate)
    {
        Transform frame = transform_math::makeIdentityTransform<Transform>();
        frame.translate = candidate.localCenter;

        const Vector extent = subtractVector(candidate.localMax, candidate.localMin);
        const Vector fallbackLongAxis = selectBoundsAxis(extent, true);
        const Vector fallbackNormalAxis = selectBoundsAxis(extent, false);
        Vector xAxis = normalizeOrFallback(fallbackLongAxis, makeVector<Vector>(1.0f, 0.0f, 0.0f));
        Vector zAxis = candidate.hasSurfaceNormal
            ? normalizeOrFallback(candidate.localSurfaceNormal, fallbackNormalAxis)
            : fallbackNormalAxis;

        if (std::abs(dotVector(xAxis, zAxis)) > 0.94f) {
            zAxis = std::abs(xAxis.z) < 0.94f ? makeVector<Vector>(0.0f, 0.0f, 1.0f) : makeVector<Vector>(0.0f, 1.0f, 0.0f);
        }

        Vector yAxis = normalizeOrFallback(crossVector(zAxis, xAxis), makeVector<Vector>(0.0f, 1.0f, 0.0f));
        xAxis = normalizeOrFallback(crossVector(yAxis, zAxis), xAxis);
        zAxis = normalizeOrFallback(zAxis, makeVector<Vector>(0.0f, 0.0f, 1.0f));
        setRotationRowsFromAxes(frame.rotate, xAxis, yAxis, zAxis);
        return frame;
    }

    template <class Transform, class Vector>
    inline PrimaryGripSelection<Transform> selectPrimaryGripFrame(
        const std::vector<PrimaryGripCandidate<Vector>>& candidates,
        const Transform& fallbackFrame,
        bool iniOverrideActive)
    {
        constexpr float MeshConfidenceThreshold = 0.75f;

        PrimaryGripSelection<Transform> result{};
        result.gripWeaponLocal = fallbackFrame;
        result.source = iniOverrideActive ? PrimaryGripSource::IniOverride : PrimaryGripSource::FallbackCurrent;
        result.confidence = iniOverrideActive ? 1.0f : 0.0f;
        result.selected = iniOverrideActive;

        if (iniOverrideActive) {
            return result;
        }

        float bestScore = 0.0f;
        std::size_t bestIndex = std::numeric_limits<std::size_t>::max();
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            const float score = scorePrimaryGripCandidate(candidates[index]);
            if (score > bestScore) {
                bestScore = score;
                bestIndex = index;
            }
        }

        if (bestIndex != std::numeric_limits<std::size_t>::max() && bestScore >= MeshConfidenceThreshold) {
            result.gripWeaponLocal = makeGripFrameFromCandidate<Transform>(candidates[bestIndex]);
            result.source = PrimaryGripSource::Mesh;
            result.confidence = bestScore;
            result.candidateIndex = bestIndex;
            result.selected = true;
        }

        return result;
    }

    template <class Transform>
    inline Transform solveWeaponWorldFromPrimaryGrip(const Transform& rightHandWorldTarget, const Transform& gripWeaponLocal)
    {
        return transform_math::composeTransforms(rightHandWorldTarget, transform_math::invertTransform(gripWeaponLocal));
    }
}
