#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>
#include <vector>

#include "physics-interaction/debug/DebugMath.h"

namespace rock::skeleton_bone_debug_math
{
    // ROCK needs a skeleton map that can stand on its own instead of depending
    // on the FRIK API for every bone. This layer makes live FO4VR root-flattened
    // bone state visible. Body collider descriptors live in
    // body/BodyColliderDescriptors.h.

    enum class DebugSkeletonBoneMode : int
    {
        Off = 0,
        CoreBodyAndFingers = 1,
        HandsAndForearmsOnly = 2,
        AllFlattenedBones = 3
    };

    enum class SkeletonBoneSnapshotSource : int
    {
        None = 0,
        GameRootFlattenedBoneTree = 1,
        FirstPersonDiagnosticOnly = 2
    };

    enum class DebugSkeletonBoneSource : int
    {
        GameRootFlattenedBoneTree = 1,
        FirstPersonDiagnosticOnly = 2
    };

    template <class Vector>
    struct AxisEndpoints
    {
        Vector xEnd{};
        Vector yEnd{};
        Vector zEnd{};
    };

    inline constexpr std::array<std::string_view, 30> kRequiredFingerBoneNames{
        "LArm_Finger11", "LArm_Finger12", "LArm_Finger13",
        "LArm_Finger21", "LArm_Finger22", "LArm_Finger23",
        "LArm_Finger31", "LArm_Finger32", "LArm_Finger33",
        "LArm_Finger41", "LArm_Finger42", "LArm_Finger43",
        "LArm_Finger51", "LArm_Finger52", "LArm_Finger53",
        "RArm_Finger11", "RArm_Finger12", "RArm_Finger13",
        "RArm_Finger21", "RArm_Finger22", "RArm_Finger23",
        "RArm_Finger31", "RArm_Finger32", "RArm_Finger33",
        "RArm_Finger41", "RArm_Finger42", "RArm_Finger43",
        "RArm_Finger51", "RArm_Finger52", "RArm_Finger53"
    };

    inline constexpr std::array<std::string_view, 31> kRequiredCoreBoneNames{
        "COM", "Pelvis", "SPINE1", "SPINE2", "Chest", "Neck", "Head",
        "LArm_Collarbone", "LArm_UpperArm", "LArm_ForeArm1", "LArm_ForeArm2", "LArm_ForeArm3", "LArm_Hand",
        "RArm_Collarbone", "RArm_UpperArm", "RArm_ForeArm1", "RArm_ForeArm2", "RArm_ForeArm3", "RArm_Hand",
        "LLeg_Thigh", "LLeg_Calf", "LLeg_Foot", "LLeg_Toe1",
        "RLeg_Thigh", "RLeg_Calf", "RLeg_Foot", "RLeg_Toe1",
        "LArm_UpperTwist1", "LArm_UpperTwist2", "RArm_UpperTwist1", "RArm_UpperTwist2"
    };

    inline constexpr std::array<std::string_view, 16> kHandsAndForearmsBoneNames{
        "LArm_ForeArm1", "LArm_ForeArm2", "LArm_ForeArm3", "LArm_Hand",
        "RArm_ForeArm1", "RArm_ForeArm2", "RArm_ForeArm3", "RArm_Hand",
        "LArm_UpperArm", "RArm_UpperArm", "LArm_Collarbone", "RArm_Collarbone",
        "LArm_UpperTwist1", "LArm_UpperTwist2", "RArm_UpperTwist1", "RArm_UpperTwist2"
    };

    template <std::size_t Count>
    inline constexpr bool allBoneNamesNonEmpty(const std::array<std::string_view, Count>& names)
    {
        for (const auto name : names) {
            if (name.empty()) {
                return false;
            }
        }
        return true;
    }

    static_assert(allBoneNamesNonEmpty(kRequiredFingerBoneNames), "Required finger bone table contains an empty name.");
    static_assert(allBoneNamesNonEmpty(kRequiredCoreBoneNames), "Required core bone table contains an empty name.");
    static_assert(allBoneNamesNonEmpty(kHandsAndForearmsBoneNames), "Hands/forearms bone table contains an empty name.");

    inline constexpr const std::array<std::string_view, 30>& requiredFingerBoneNames() { return kRequiredFingerBoneNames; }
    inline constexpr const std::array<std::string_view, 31>& requiredCoreBoneNames() { return kRequiredCoreBoneNames; }
    inline constexpr const std::array<std::string_view, 16>& handsAndForearmsBoneNames() { return kHandsAndForearmsBoneNames; }
    inline constexpr std::size_t skeletonOverlayBudget() { return 768; }

    inline int sanitizeMaxSkeletonBonesDrawn(int value)
    {
        if (value < 0) {
            return 0;
        }
        return static_cast<int>((std::min)(static_cast<std::size_t>(value), skeletonOverlayBudget()));
    }

    inline int sanitizeMaxSkeletonAxesDrawn(int value)
    {
        if (value < 0) {
            return 0;
        }
        return static_cast<int>((std::min)(static_cast<std::size_t>(value), skeletonOverlayBudget()));
    }

    inline bool equalsIgnoreCase(std::string_view lhs, std::string_view rhs)
    {
        if (lhs.size() != rhs.size()) {
            return false;
        }

        for (std::size_t i = 0; i < lhs.size(); ++i) {
            char a = lhs[i];
            char b = rhs[i];
            if (a >= 'A' && a <= 'Z') {
                a = static_cast<char>(a - 'A' + 'a');
            }
            if (b >= 'A' && b <= 'Z') {
                b = static_cast<char>(b - 'A' + 'a');
            }
            if (a != b) {
                return false;
            }
        }
        return true;
    }

    inline bool containsName(std::string_view name, std::string_view token)
    {
        if (token.empty() || token.size() > name.size()) {
            return false;
        }

        for (std::size_t offset = 0; offset + token.size() <= name.size(); ++offset) {
            if (equalsIgnoreCase(name.substr(offset, token.size()), token)) {
                return true;
            }
        }
        return false;
    }

    inline std::string_view trimFilterToken(std::string_view value)
    {
        while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
            value.remove_prefix(1);
        }
        while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {
            value.remove_suffix(1);
        }
        return value;
    }

    inline bool commaSeparatedFilterContains(std::string_view filter, std::string_view boneName)
    {
        filter = trimFilterToken(filter);
        if (filter.empty()) {
            return false;
        }

        while (!filter.empty()) {
            const std::size_t comma = filter.find(',');
            const std::string_view token = trimFilterToken(filter.substr(0, comma));
            if (token == boneName) {
                return true;
            }
            if (comma == std::string_view::npos) {
                break;
            }
            filter.remove_prefix(comma + 1);
        }
        return false;
    }

    inline bool shouldDrawSkeletonAxis(std::string_view axisFilter, std::string_view boneName, std::size_t axesAlreadyDrawn, std::size_t axisCap)
    {
        if (axisCap == 0 || axesAlreadyDrawn >= axisCap) {
            return false;
        }

        axisFilter = trimFilterToken(axisFilter);
        if (axisFilter.empty()) {
            return true;
        }

        return commaSeparatedFilterContains(axisFilter, boneName);
    }

    template <std::size_t Count>
    inline bool containsExact(const std::array<std::string_view, Count>& names, std::string_view name)
    {
        return std::find(names.begin(), names.end(), name) != names.end();
    }

    inline bool isExcludedDefaultBone(std::string_view name)
    {
        return containsName(name, "_skin") ||
               containsName(name, "_armor") ||
               containsName(name, "weapon") ||
               containsName(name, "camera") ||
               containsName(name, "projectile") ||
               containsName(name, "animobject") ||
               containsName(name, "pipboy") ||
               containsName(name, "helper") ||
               containsName(name, "ui") ||
               containsName(name, "hud") ||
               containsName(name, "scope") ||
               containsName(name, "light") ||
               containsName(name, "pauldron");
    }

    inline DebugSkeletonBoneMode sanitizeDebugSkeletonBoneMode(int mode)
    {
        switch (mode) {
        case 0:
            return DebugSkeletonBoneMode::Off;
        case 1:
            return DebugSkeletonBoneMode::CoreBodyAndFingers;
        case 2:
            return DebugSkeletonBoneMode::HandsAndForearmsOnly;
        case 3:
            return DebugSkeletonBoneMode::AllFlattenedBones;
        default:
            return DebugSkeletonBoneMode::CoreBodyAndFingers;
        }
    }

    inline DebugSkeletonBoneSource sanitizeDebugSkeletonBoneSource(int source)
    {
        switch (source) {
        case 1:
            return DebugSkeletonBoneSource::GameRootFlattenedBoneTree;
        case 2:
            return DebugSkeletonBoneSource::FirstPersonDiagnosticOnly;
        default:
            return DebugSkeletonBoneSource::GameRootFlattenedBoneTree;
        }
    }

    inline const char* modeName(DebugSkeletonBoneMode mode)
    {
        switch (mode) {
        case DebugSkeletonBoneMode::Off:
            return "Off";
        case DebugSkeletonBoneMode::CoreBodyAndFingers:
            return "CoreBodyAndFingers";
        case DebugSkeletonBoneMode::HandsAndForearmsOnly:
            return "HandsAndForearmsOnly";
        case DebugSkeletonBoneMode::AllFlattenedBones:
            return "AllFlattenedBones";
        }
        return "Unknown";
    }

    inline const char* snapshotSourceName(SkeletonBoneSnapshotSource source)
    {
        switch (source) {
        case SkeletonBoneSnapshotSource::None:
            return "None";
        case SkeletonBoneSnapshotSource::GameRootFlattenedBoneTree:
            return "GameRootFlattenedBoneTree";
        case SkeletonBoneSnapshotSource::FirstPersonDiagnosticOnly:
            return "FirstPersonDiagnosticOnly";
        }
        return "Unknown";
    }

    inline const char* sourceName(DebugSkeletonBoneSource source)
    {
        switch (source) {
        case DebugSkeletonBoneSource::GameRootFlattenedBoneTree:
            return "GameRootFlattenedBoneTree";
        case DebugSkeletonBoneSource::FirstPersonDiagnosticOnly:
            return "FirstPersonDiagnosticOnly";
        }
        return "Unknown";
    }

    inline bool shouldIncludeBone(DebugSkeletonBoneMode mode, std::string_view name)
    {
        if (name.empty()) {
            return false;
        }

        if (mode == DebugSkeletonBoneMode::Off) {
            return false;
        }

        if (mode == DebugSkeletonBoneMode::AllFlattenedBones) {
            return true;
        }

        if (isExcludedDefaultBone(name)) {
            return false;
        }

        if (containsExact(kRequiredFingerBoneNames, name)) {
            return true;
        }

        if (mode == DebugSkeletonBoneMode::HandsAndForearmsOnly) {
            return containsExact(kHandsAndForearmsBoneNames, name);
        }

        return containsExact(kRequiredCoreBoneNames, name);
    }

    inline int resolveDrawableParentIndex(std::size_t boneIndex, const std::vector<int>& parentIndices, const std::vector<bool>& included)
    {
        if (boneIndex >= parentIndices.size() || boneIndex >= included.size()) {
            return -1;
        }

        int parent = parentIndices[boneIndex];
        int guard = 0;
        while (parent >= 0 && static_cast<std::size_t>(parent) < parentIndices.size() && guard++ < 512) {
            if (static_cast<std::size_t>(parent) < included.size() && included[static_cast<std::size_t>(parent)]) {
                return parent;
            }
            parent = parentIndices[static_cast<std::size_t>(parent)];
        }
        return -1;
    }

    template <class Vector>
    inline Vector makeVector(float x, float y, float z)
    {
        Vector out{};
        out.x = x;
        out.y = y;
        out.z = z;
        return out;
    }

    template <class Vector>
    inline Vector normalized(Vector value)
    {
        const float lenSq = value.x * value.x + value.y * value.y + value.z * value.z;
        if (lenSq <= 0.000001f) {
            return makeVector<Vector>(0.0f, 0.0f, 0.0f);
        }

        const float invLen = 1.0f / std::sqrt(lenSq);
        return makeVector<Vector>(value.x * invLen, value.y * invLen, value.z * invLen);
    }

    template <class Transform>
    inline AxisEndpoints<decltype(Transform{}.translate)> computeAxisEndpoints(const Transform& transform, float axisLength)
    {
        using Vector = decltype(transform.translate);
        const Vector xAxis = normalized(debug_axis_math::rotateNiLocalToWorld(transform.rotate, makeVector<Vector>(1.0f, 0.0f, 0.0f)));
        const Vector yAxis = normalized(debug_axis_math::rotateNiLocalToWorld(transform.rotate, makeVector<Vector>(0.0f, 1.0f, 0.0f)));
        const Vector zAxis = normalized(debug_axis_math::rotateNiLocalToWorld(transform.rotate, makeVector<Vector>(0.0f, 0.0f, 1.0f)));

        return AxisEndpoints<Vector>{
            .xEnd = makeVector<Vector>(transform.translate.x + xAxis.x * axisLength, transform.translate.y + xAxis.y * axisLength, transform.translate.z + xAxis.z * axisLength),
            .yEnd = makeVector<Vector>(transform.translate.x + yAxis.x * axisLength, transform.translate.y + yAxis.y * axisLength, transform.translate.z + yAxis.z * axisLength),
            .zEnd = makeVector<Vector>(transform.translate.x + zAxis.x * axisLength, transform.translate.y + zAxis.y * axisLength, transform.translate.z + zAxis.z * axisLength),
        };
    }

}
