#pragma once

/*
 * Multi-finger grab validation is intentionally separate from the runtime
 * Havok and mesh queries. HIGGS preserves a coherent object-in-hand transform
 * once a contact point is chosen; ROCK's earlier single semantic/palm fallback
 * could choose that point from incomplete evidence, which let one finger or a
 * palm-plane probe define the whole grab. This layer requires a grouped set of
 * distinct finger surface patches first, then captures the object-to-grip
 * transform so the held object keeps its current orientation instead of
 * snapping to a fingertip or palm axis.
 */

#include "HandColliderSemanticTypes.h"
#include "TransformMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace frik::rock::grab_multi_finger_contact_math
{
    inline constexpr std::uint32_t kInvalidBodyId = hand_collider_semantics::kInvalidBodyId;
    inline constexpr std::size_t kMaxFingerGroups = hand_collider_semantics::kHandFingerCount;

    struct GripContactSetOptions
    {
        bool enabled = true;
        std::uint32_t targetBodyId = kInvalidBodyId;
        int minimumFingerGroups = 3;
        std::uint32_t maxContactAgeFrames = 5;
        float minimumSpreadGameUnits = 1.0f;
    };

    template <class Vector>
    struct FingerContactPatch
    {
        bool valid = false;
        hand_collider_semantics::HandFinger finger = hand_collider_semantics::HandFinger::None;
        hand_collider_semantics::HandFingerSegment segment = hand_collider_semantics::HandFingerSegment::None;
        hand_collider_semantics::HandColliderRole role = hand_collider_semantics::HandColliderRole::PalmAnchor;
        std::uint32_t handBodyId = kInvalidBodyId;
        std::uint32_t objectBodyId = kInvalidBodyId;
        Vector handPointWorld{};
        Vector objectPointWorld{};
        Vector normalWorld{};
        float quality = 0.0f;
        std::uint32_t framesSinceContact = 0xFFFF'FFFFu;
    };

    template <class Vector>
    struct FingerContactGroup
    {
        bool valid = false;
        hand_collider_semantics::HandFinger finger = hand_collider_semantics::HandFinger::None;
        FingerContactPatch<Vector> patch{};
    };

    template <class Vector>
    struct GripContactSet
    {
        bool valid = false;
        std::uint32_t objectBodyId = kInvalidBodyId;
        std::array<FingerContactGroup<Vector>, kMaxFingerGroups> groups{};
        std::uint32_t groupCount = 0;
        Vector contactCenterWorld{};
        Vector handCenterWorld{};
        Vector averageNormalWorld{};
        float spreadGameUnits = 0.0f;
        const char* reason = "disabled";
    };

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
    inline Vector add(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    template <class Vector>
    inline Vector sub(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    template <class Vector>
    inline Vector mul(const Vector& value, float scale)
    {
        return makeVector<Vector>(value.x * scale, value.y * scale, value.z * scale);
    }

    template <class Vector>
    inline float dot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return dot(value, value);
    }

    template <class Vector>
    inline float length(const Vector& value)
    {
        return std::sqrt(lengthSquared(value));
    }

    template <class Vector>
    inline Vector normalizeOrZero(const Vector& value)
    {
        const float len = length(value);
        if (len <= 1.0e-6f || !std::isfinite(len)) {
            return {};
        }
        return mul(value, 1.0f / len);
    }

    inline int fingerIndex(hand_collider_semantics::HandFinger finger)
    {
        switch (finger) {
        case hand_collider_semantics::HandFinger::Thumb:
            return 0;
        case hand_collider_semantics::HandFinger::Index:
            return 1;
        case hand_collider_semantics::HandFinger::Middle:
            return 2;
        case hand_collider_semantics::HandFinger::Ring:
            return 3;
        case hand_collider_semantics::HandFinger::Pinky:
            return 4;
        default:
            return -1;
        }
    }

    inline int segmentPriority(hand_collider_semantics::HandFingerSegment segment,
        hand_collider_semantics::HandColliderRole role)
    {
        if (role == hand_collider_semantics::HandColliderRole::ThumbPad) {
            return 0;
        }
        switch (segment) {
        case hand_collider_semantics::HandFingerSegment::Tip:
            return 0;
        case hand_collider_semantics::HandFingerSegment::Middle:
            return 1;
        case hand_collider_semantics::HandFingerSegment::Base:
            return 2;
        default:
            return 10;
        }
    }

    template <class Vector>
    inline bool isBetterPatchForFinger(const FingerContactPatch<Vector>& candidate,
        const FingerContactPatch<Vector>& current)
    {
        if (!current.valid) {
            return true;
        }
        if (candidate.framesSinceContact != current.framesSinceContact) {
            return candidate.framesSinceContact < current.framesSinceContact;
        }
        const int candidatePriority = segmentPriority(candidate.segment, candidate.role);
        const int currentPriority = segmentPriority(current.segment, current.role);
        if (candidatePriority != currentPriority) {
            return candidatePriority < currentPriority;
        }
        return candidate.quality > current.quality;
    }

    template <class Vector>
    inline bool patchTargetsBody(const FingerContactPatch<Vector>& patch,
        std::uint32_t targetBodyId)
    {
        if (patch.objectBodyId == kInvalidBodyId) {
            return false;
        }
        return targetBodyId == kInvalidBodyId || patch.objectBodyId == targetBodyId;
    }

    template <class Vector>
    inline GripContactSet<Vector> buildGripContactSet(const std::vector<FingerContactPatch<Vector>>& patches,
        const GripContactSetOptions& options)
    {
        GripContactSet<Vector> result{};
        result.objectBodyId = options.targetBodyId;
        if (!options.enabled) {
            result.reason = "disabled";
            return result;
        }
        if (options.minimumFingerGroups < 1 || options.minimumFingerGroups > static_cast<int>(kMaxFingerGroups)) {
            result.reason = "invalidMinimumFingerGroups";
            return result;
        }

        bool sawUsableBody = false;
        std::uint32_t acceptedBodyId = options.targetBodyId;
        for (const auto& patch : patches) {
            if (!patch.valid || patch.handBodyId == kInvalidBodyId || patch.framesSinceContact > options.maxContactAgeFrames) {
                continue;
            }
            const int index = fingerIndex(patch.finger);
            if (index < 0) {
                continue;
            }
            if (options.targetBodyId != kInvalidBodyId && patch.objectBodyId != options.targetBodyId) {
                result.reason = "mixedBodies";
                return result;
            }
            if (!patchTargetsBody(patch, options.targetBodyId)) {
                continue;
            }
            if (acceptedBodyId == kInvalidBodyId) {
                acceptedBodyId = patch.objectBodyId;
            }
            if (patch.objectBodyId != acceptedBodyId) {
                result.reason = "mixedBodies";
                return result;
            }
            sawUsableBody = true;

            auto& group = result.groups[static_cast<std::size_t>(index)];
            if (isBetterPatchForFinger(patch, group.patch)) {
                group.valid = true;
                group.finger = patch.finger;
                group.patch = patch;
            }
        }

        if (!sawUsableBody) {
            result.reason = "noFingerContacts";
            return result;
        }

        Vector contactSum{};
        Vector handSum{};
        Vector normalSum{};
        for (const auto& group : result.groups) {
            if (!group.valid) {
                continue;
            }
            ++result.groupCount;
            contactSum = add(contactSum, group.patch.objectPointWorld);
            handSum = add(handSum, group.patch.handPointWorld);
            normalSum = add(normalSum, normalizeOrZero(group.patch.normalWorld));
        }

        if (result.groupCount < static_cast<std::uint32_t>(options.minimumFingerGroups)) {
            result.reason = "insufficientFingerGroups";
            return result;
        }

        const float invCount = 1.0f / static_cast<float>(result.groupCount);
        result.contactCenterWorld = mul(contactSum, invCount);
        result.handCenterWorld = mul(handSum, invCount);
        result.averageNormalWorld = normalizeOrZero(normalSum);
        result.objectBodyId = acceptedBodyId;

        float maxRadius = 0.0f;
        for (const auto& group : result.groups) {
            if (!group.valid) {
                continue;
            }
            maxRadius = (std::max)(maxRadius, length(sub(group.patch.objectPointWorld, result.contactCenterWorld)));
        }
        result.spreadGameUnits = maxRadius;
        if (result.spreadGameUnits < (std::max)(0.0f, options.minimumSpreadGameUnits)) {
            result.reason = "degenerateContactSpread";
            return result;
        }

        result.valid = true;
        result.reason = "validGripContactSet";
        return result;
    }

    template <class Transform>
    inline Transform captureObjectToGripFrame(const Transform& objectWorld,
        const Transform& gripWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(gripWorld), objectWorld);
    }

    template <class Transform>
    inline Transform recomposeObjectFromGripFrame(const Transform& gripWorld,
        const Transform& objectToGrip)
    {
        return transform_math::composeTransforms(gripWorld, objectToGrip);
    }
}
