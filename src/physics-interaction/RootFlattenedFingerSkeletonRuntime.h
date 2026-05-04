#pragma once

/*
 * ROCK's runtime hand geometry is sourced from the live FO4/FRIK skeleton, not
 * authored INI landmark tables. This helper resolves the rendered finger chains
 * and palm-facing normal into a compact world-space snapshot for collider,
 * pose, and debug systems while leaving final pose publication to the FRIK API.
 */

#include "RE/NetImmerse/NiPoint.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace frik::rock::root_flattened_finger_skeleton_runtime
{
    struct FingerChain
    {
        std::array<RE::NiPoint3, 3> points{};
        bool valid = false;
    };

    struct Snapshot
    {
        std::array<FingerChain, 5> fingers{};
        RE::NiPoint3 palmNormalWorld{ 0.0f, 0.0f, -1.0f };
        bool palmNormalValid = false;
        bool valid = false;
    };

    struct FingerLandmark
    {
        RE::NiPoint3 base{};
        RE::NiPoint3 openDirection{ 1.0f, 0.0f, 0.0f };
        float length = 0.0f;
        bool valid = false;
    };

    struct LandmarkSet
    {
        std::array<FingerLandmark, 5> fingers{};
        RE::NiPoint3 palmNormalWorld{ 0.0f, 0.0f, -1.0f };
        bool valid = false;
    };

    inline const char* fingerBoneName(bool isLeft, std::size_t fingerIndex, std::size_t segmentIndex)
    {
        static constexpr std::array<const char*, 15> kRightNames{
            "RArm_Finger11",
            "RArm_Finger12",
            "RArm_Finger13",
            "RArm_Finger21",
            "RArm_Finger22",
            "RArm_Finger23",
            "RArm_Finger31",
            "RArm_Finger32",
            "RArm_Finger33",
            "RArm_Finger41",
            "RArm_Finger42",
            "RArm_Finger43",
            "RArm_Finger51",
            "RArm_Finger52",
            "RArm_Finger53"
        };
        static constexpr std::array<const char*, 15> kLeftNames{
            "LArm_Finger11",
            "LArm_Finger12",
            "LArm_Finger13",
            "LArm_Finger21",
            "LArm_Finger22",
            "LArm_Finger23",
            "LArm_Finger31",
            "LArm_Finger32",
            "LArm_Finger33",
            "LArm_Finger41",
            "LArm_Finger42",
            "LArm_Finger43",
            "LArm_Finger51",
            "LArm_Finger52",
            "LArm_Finger53"
        };

        if (fingerIndex >= 5 || segmentIndex >= 3) {
            return nullptr;
        }

        const std::size_t index = fingerIndex * 3 + segmentIndex;
        return isLeft ? kLeftNames[index] : kRightNames[index];
    }

    inline float distance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        const float dx = lhs.x - rhs.x;
        const float dy = lhs.y - rhs.y;
        const float dz = lhs.z - rhs.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    inline RE::NiPoint3 normalizedOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (!std::isfinite(lengthSquared) || lengthSquared <= 0.000001f) {
            const float fallbackLengthSquared = fallback.x * fallback.x + fallback.y * fallback.y + fallback.z * fallback.z;
            if (!std::isfinite(fallbackLengthSquared) || fallbackLengthSquared <= 0.000001f) {
                return RE::NiPoint3(1.0f, 0.0f, 0.0f);
            }
            const float fallbackInv = 1.0f / std::sqrt(fallbackLengthSquared);
            return RE::NiPoint3(fallback.x * fallbackInv, fallback.y * fallbackInv, fallback.z * fallbackInv);
        }

        const float inv = 1.0f / std::sqrt(lengthSquared);
        return RE::NiPoint3(value.x * inv, value.y * inv, value.z * inv);
    }

    inline FingerLandmark buildFingerLandmark(const FingerChain& chain)
    {
        FingerLandmark landmark{};
        if (!chain.valid) {
            return landmark;
        }

        const float firstLength = distance(chain.points[0], chain.points[1]);
        const float secondLength = distance(chain.points[1], chain.points[2]);
        const float fullLength = firstLength + secondLength;
        if (!std::isfinite(fullLength) || fullLength <= 0.000001f) {
            return landmark;
        }

        landmark.base = chain.points[0];
        landmark.openDirection = normalizedOrFallback(chain.points[2] - chain.points[0], RE::NiPoint3(1.0f, 0.0f, 0.0f));
        landmark.length = fullLength;
        landmark.valid = true;
        return landmark;
    }

    inline LandmarkSet buildLandmarkSet(const Snapshot& snapshot)
    {
        LandmarkSet set{};
        bool allValid = snapshot.valid && snapshot.palmNormalValid;
        set.palmNormalWorld = normalizedOrFallback(snapshot.palmNormalWorld, RE::NiPoint3(0.0f, 0.0f, -1.0f));
        for (std::size_t finger = 0; finger < set.fingers.size(); ++finger) {
            set.fingers[finger] = buildFingerLandmark(snapshot.fingers[finger]);
            allValid = allValid && set.fingers[finger].valid;
        }
        set.valid = allValid;
        return set;
    }

    bool resolveLiveFingerSkeletonSnapshot(bool isLeft, Snapshot& outSnapshot, std::string* outMissingBoneName = nullptr);
}
