#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>
#include <vector>

#include "DebugAxisMath.h"

namespace frik::rock::skeleton_bone_debug_math
{
    // ROCK needs a skeleton map that can stand on its own instead of depending
    // on the FRIK API for every bone. HIGGS used debug transforms to make live
    // physics state visible; this layer applies the same reference-driven idea
    // to the FO4VR root flattened bone tree that FRIK mutates for body and hand
    // IK, while keeping future collider roles scaffolded separately from the
    // visualizer that proves the skeleton first.

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

    enum class BoneColliderProfileVariant : int
    {
        Standard = 0,
        PowerArmor = 1
    };

    enum class BoneColliderRole : int
    {
        UpperArmSegment,
        ForearmSegment,
        HandSegment,
        FingerSegment,
        TorsoSegment,
        LegSegment,
        FootSegment
    };

    enum class BoneColliderEndpointMode : int
    {
        ChildBone,
        ExtrapolatedTip,
        DerivedLandmark
    };

    struct BoneColliderDescriptor
    {
        BoneColliderRole role = BoneColliderRole::FingerSegment;
        std::string_view startBone{};
        std::string_view endBone{};
        float radiusGameUnits = 1.0f;
        float convexRadiusGameUnits = 0.25f;
        bool enabled = true;
        BoneColliderEndpointMode endpointMode = BoneColliderEndpointMode::ChildBone;
    };

    struct FingerBoneChain
    {
        std::string_view base{};
        std::string_view middle{};
        std::string_view tip{};
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

    inline constexpr std::array<std::string_view, 32> kRequiredCoreBoneNames{
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

    inline constexpr std::array<FingerBoneChain, 10> kFingerBoneChains{
        FingerBoneChain{ "LArm_Finger11", "LArm_Finger12", "LArm_Finger13" },
        FingerBoneChain{ "LArm_Finger21", "LArm_Finger22", "LArm_Finger23" },
        FingerBoneChain{ "LArm_Finger31", "LArm_Finger32", "LArm_Finger33" },
        FingerBoneChain{ "LArm_Finger41", "LArm_Finger42", "LArm_Finger43" },
        FingerBoneChain{ "LArm_Finger51", "LArm_Finger52", "LArm_Finger53" },
        FingerBoneChain{ "RArm_Finger11", "RArm_Finger12", "RArm_Finger13" },
        FingerBoneChain{ "RArm_Finger21", "RArm_Finger22", "RArm_Finger23" },
        FingerBoneChain{ "RArm_Finger31", "RArm_Finger32", "RArm_Finger33" },
        FingerBoneChain{ "RArm_Finger41", "RArm_Finger42", "RArm_Finger43" },
        FingerBoneChain{ "RArm_Finger51", "RArm_Finger52", "RArm_Finger53" }
    };

    inline constexpr std::array<BoneColliderDescriptor, 23> kStandardBodyColliderDescriptors{
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "Pelvis", "SPINE1", 4.5f, 0.50f },
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "SPINE1", "SPINE2", 5.0f, 0.50f },
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "SPINE2", "Chest", 5.5f, 0.50f },
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "Chest", "Neck", 3.5f, 0.40f },
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "Neck", "Head", 3.0f, 0.35f },
        BoneColliderDescriptor{ BoneColliderRole::UpperArmSegment, "LArm_Collarbone", "LArm_UpperArm", 2.5f, 0.30f },
        BoneColliderDescriptor{ BoneColliderRole::UpperArmSegment, "LArm_UpperArm", "LArm_ForeArm1", 2.7f, 0.30f },
        BoneColliderDescriptor{ BoneColliderRole::ForearmSegment, "LArm_ForeArm1", "LArm_ForeArm2", 2.2f, 0.25f },
        BoneColliderDescriptor{ BoneColliderRole::ForearmSegment, "LArm_ForeArm2", "LArm_ForeArm3", 2.0f, 0.25f },
        BoneColliderDescriptor{ BoneColliderRole::HandSegment, "LArm_ForeArm3", "LArm_Hand", 2.1f, 0.25f },
        BoneColliderDescriptor{ BoneColliderRole::UpperArmSegment, "RArm_Collarbone", "RArm_UpperArm", 2.5f, 0.30f },
        BoneColliderDescriptor{ BoneColliderRole::UpperArmSegment, "RArm_UpperArm", "RArm_ForeArm1", 2.7f, 0.30f },
        BoneColliderDescriptor{ BoneColliderRole::ForearmSegment, "RArm_ForeArm1", "RArm_ForeArm2", 2.2f, 0.25f },
        BoneColliderDescriptor{ BoneColliderRole::ForearmSegment, "RArm_ForeArm2", "RArm_ForeArm3", 2.0f, 0.25f },
        BoneColliderDescriptor{ BoneColliderRole::HandSegment, "RArm_ForeArm3", "RArm_Hand", 2.1f, 0.25f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "Pelvis", "LLeg_Thigh", 3.4f, 0.35f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "LLeg_Thigh", "LLeg_Calf", 3.0f, 0.35f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "LLeg_Calf", "LLeg_Foot", 2.4f, 0.30f },
        BoneColliderDescriptor{ BoneColliderRole::FootSegment, "LLeg_Foot", "LLeg_Toe1", 2.2f, 0.25f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "Pelvis", "RLeg_Thigh", 3.4f, 0.35f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "RLeg_Thigh", "RLeg_Calf", 3.0f, 0.35f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "RLeg_Calf", "RLeg_Foot", 2.4f, 0.30f },
        BoneColliderDescriptor{ BoneColliderRole::FootSegment, "RLeg_Foot", "RLeg_Toe1", 2.2f, 0.25f }
    };

    inline constexpr std::array<BoneColliderDescriptor, 23> kPowerArmorBodyColliderDescriptors{
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "Pelvis", "SPINE1", 6.5f, 0.65f },
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "SPINE1", "SPINE2", 7.0f, 0.65f },
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "SPINE2", "Chest", 7.5f, 0.65f },
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "Chest", "Neck", 5.0f, 0.50f },
        BoneColliderDescriptor{ BoneColliderRole::TorsoSegment, "Neck", "Head", 4.0f, 0.45f },
        BoneColliderDescriptor{ BoneColliderRole::UpperArmSegment, "LArm_Collarbone", "LArm_UpperArm", 4.2f, 0.45f },
        BoneColliderDescriptor{ BoneColliderRole::UpperArmSegment, "LArm_UpperArm", "LArm_ForeArm1", 4.5f, 0.45f },
        BoneColliderDescriptor{ BoneColliderRole::ForearmSegment, "LArm_ForeArm1", "LArm_ForeArm2", 3.8f, 0.40f },
        BoneColliderDescriptor{ BoneColliderRole::ForearmSegment, "LArm_ForeArm2", "LArm_ForeArm3", 3.4f, 0.40f },
        BoneColliderDescriptor{ BoneColliderRole::HandSegment, "LArm_ForeArm3", "LArm_Hand", 3.5f, 0.35f },
        BoneColliderDescriptor{ BoneColliderRole::UpperArmSegment, "RArm_Collarbone", "RArm_UpperArm", 4.2f, 0.45f },
        BoneColliderDescriptor{ BoneColliderRole::UpperArmSegment, "RArm_UpperArm", "RArm_ForeArm1", 4.5f, 0.45f },
        BoneColliderDescriptor{ BoneColliderRole::ForearmSegment, "RArm_ForeArm1", "RArm_ForeArm2", 3.8f, 0.40f },
        BoneColliderDescriptor{ BoneColliderRole::ForearmSegment, "RArm_ForeArm2", "RArm_ForeArm3", 3.4f, 0.40f },
        BoneColliderDescriptor{ BoneColliderRole::HandSegment, "RArm_ForeArm3", "RArm_Hand", 3.5f, 0.35f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "Pelvis", "LLeg_Thigh", 5.2f, 0.50f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "LLeg_Thigh", "LLeg_Calf", 4.8f, 0.50f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "LLeg_Calf", "LLeg_Foot", 4.0f, 0.45f },
        BoneColliderDescriptor{ BoneColliderRole::FootSegment, "LLeg_Foot", "LLeg_Toe1", 3.5f, 0.35f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "Pelvis", "RLeg_Thigh", 5.2f, 0.50f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "RLeg_Thigh", "RLeg_Calf", 4.8f, 0.50f },
        BoneColliderDescriptor{ BoneColliderRole::LegSegment, "RLeg_Calf", "RLeg_Foot", 4.0f, 0.45f },
        BoneColliderDescriptor{ BoneColliderRole::FootSegment, "RLeg_Foot", "RLeg_Toe1", 3.5f, 0.35f }
    };

    inline constexpr const std::array<std::string_view, 30>& requiredFingerBoneNames() { return kRequiredFingerBoneNames; }
    inline constexpr const std::array<std::string_view, 32>& requiredCoreBoneNames() { return kRequiredCoreBoneNames; }
    inline constexpr std::size_t estimatedCoreBodyAndFingerBoneCount() { return kRequiredCoreBoneNames.size() + kRequiredFingerBoneNames.size(); }
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

    inline void appendFingerColliderDescriptors(std::vector<BoneColliderDescriptor>& descriptors, BoneColliderProfileVariant variant)
    {
        const bool powerArmor = variant == BoneColliderProfileVariant::PowerArmor;
        const float baseRadius = powerArmor ? 0.85f : 0.55f;
        const float middleRadius = powerArmor ? 0.75f : 0.45f;
        const float tipRadius = powerArmor ? 0.65f : 0.38f;
        const float convexRadius = powerArmor ? 0.15f : 0.10f;

        for (const auto& chain : kFingerBoneChains) {
            descriptors.push_back(BoneColliderDescriptor{
                BoneColliderRole::FingerSegment,
                chain.base,
                chain.middle,
                baseRadius,
                convexRadius,
                true,
                BoneColliderEndpointMode::ChildBone });
            descriptors.push_back(BoneColliderDescriptor{
                BoneColliderRole::FingerSegment,
                chain.middle,
                chain.tip,
                middleRadius,
                convexRadius,
                true,
                BoneColliderEndpointMode::ChildBone });
            descriptors.push_back(BoneColliderDescriptor{
                BoneColliderRole::FingerSegment,
                chain.tip,
                {},
                tipRadius,
                convexRadius,
                true,
                BoneColliderEndpointMode::ExtrapolatedTip });
        }
    }

    inline std::vector<BoneColliderDescriptor> colliderDescriptors(BoneColliderProfileVariant variant)
    {
        std::vector<BoneColliderDescriptor> descriptors;
        descriptors.reserve(kStandardBodyColliderDescriptors.size() + kFingerBoneChains.size() * 3);

        const auto& bodyDescriptors = variant == BoneColliderProfileVariant::PowerArmor ? kPowerArmorBodyColliderDescriptors : kStandardBodyColliderDescriptors;
        descriptors.insert(descriptors.end(), bodyDescriptors.begin(), bodyDescriptors.end());
        appendFingerColliderDescriptors(descriptors, variant);
        return descriptors;
    }

    inline bool descriptorTableBonesAreNamed(BoneColliderProfileVariant variant)
    {
        const auto descriptors = colliderDescriptors(variant);
        return std::all_of(descriptors.begin(), descriptors.end(), [](const BoneColliderDescriptor& descriptor) {
            const bool validEnd =
                descriptor.endpointMode == BoneColliderEndpointMode::ExtrapolatedTip ? descriptor.endBone.empty() : !descriptor.endBone.empty();
            const bool distinctNamedBones = descriptor.endBone.empty() || descriptor.startBone != descriptor.endBone;
            return descriptor.enabled &&
                   !descriptor.startBone.empty() &&
                   validEnd &&
                   distinctNamedBones &&
                   descriptor.radiusGameUnits > 0.0f &&
                   descriptor.convexRadiusGameUnits >= 0.0f;
        });
    }

    inline std::size_t fingerSegmentDescriptorCount(BoneColliderProfileVariant variant)
    {
        const auto descriptors = colliderDescriptors(variant);
        return static_cast<std::size_t>(std::count_if(descriptors.begin(), descriptors.end(), [](const BoneColliderDescriptor& descriptor) {
            return descriptor.role == BoneColliderRole::FingerSegment;
        }));
    }

    inline std::size_t tipExtrapolatedDescriptorCount(BoneColliderProfileVariant variant)
    {
        const auto descriptors = colliderDescriptors(variant);
        return static_cast<std::size_t>(std::count_if(descriptors.begin(), descriptors.end(), [](const BoneColliderDescriptor& descriptor) {
            return descriptor.role == BoneColliderRole::FingerSegment && descriptor.endpointMode == BoneColliderEndpointMode::ExtrapolatedTip;
        }));
    }

    inline float representativeForearmRadius(BoneColliderProfileVariant variant)
    {
        const auto descriptors = colliderDescriptors(variant);
        for (const auto& descriptor : descriptors) {
            if (descriptor.role == BoneColliderRole::ForearmSegment && descriptor.startBone == "RArm_ForeArm1") {
                return descriptor.radiusGameUnits;
            }
        }
        return 0.0f;
    }

    inline BoneColliderProfileVariant colliderVariantForPowerArmor(bool inPowerArmor)
    {
        return inPowerArmor ? BoneColliderProfileVariant::PowerArmor : BoneColliderProfileVariant::Standard;
    }
}
