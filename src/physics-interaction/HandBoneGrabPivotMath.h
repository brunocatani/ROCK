#pragma once

#include "HandColliderSemanticTypes.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace frik::rock::hand_bone_grab_pivot_math
{
    /*
     * A bone-derived hand can choose a grab pivot from what actually touched the
     * object. Palm-only contact uses the palm face; opposing thumb/finger
     * contact uses the midpoint; wider handle-like clusters use a weighted
     * centroid. The result becomes the precise palm-side pivot source for grab
     * frame creation while the object pivot remains frozen in object space.
     */

    template <class Vector>
    struct BoneContactPoint
    {
        bool valid = false;
        hand_collider_semantics::HandColliderRole role = hand_collider_semantics::HandColliderRole::PalmFace;
        Vector point{};
        Vector normal{};
        float confidence = 0.0f;
    };

    template <class Vector>
    struct HandContactPivotResult
    {
        bool valid = false;
        Vector point{};
        Vector normal{};
        float confidence = 0.0f;
        const char* reason = "noContact";
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
    inline Vector mul(const Vector& value, float scale)
    {
        return makeVector<Vector>(value.x * scale, value.y * scale, value.z * scale);
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    template <class Vector>
    inline Vector normalizeOrZero(const Vector& value)
    {
        const float lenSq = lengthSquared(value);
        if (lenSq <= 1.0e-8f) {
            return Vector{};
        }
        return mul(value, 1.0f / std::sqrt(lenSq));
    }

    inline constexpr bool isOpposingFingerRole(hand_collider_semantics::HandColliderRole role)
    {
        return role == hand_collider_semantics::HandColliderRole::IndexTip ||
               role == hand_collider_semantics::HandColliderRole::MiddleTip ||
               role == hand_collider_semantics::HandColliderRole::IndexMiddle ||
               role == hand_collider_semantics::HandColliderRole::MiddleMiddle;
    }

    template <class Vector, std::size_t Count>
    inline HandContactPivotResult<Vector> chooseHandContactPivot(
        const std::array<BoneContactPoint<Vector>, Count>& contacts,
        std::size_t contactCount,
        const Vector& fallbackPalmPoint)
    {
        HandContactPivotResult<Vector> result{};
        result.point = fallbackPalmPoint;
        result.normal = {};
        result.confidence = 0.0f;
        result.reason = "fallbackPalm";

        const BoneContactPoint<Vector>* thumb = nullptr;
        const BoneContactPoint<Vector>* opposing = nullptr;
        const BoneContactPoint<Vector>* first = nullptr;
        Vector weightedPoint{};
        Vector weightedNormal{};
        float totalWeight = 0.0f;
        std::size_t validCount = 0;

        const std::size_t clampedCount = (std::min)(contactCount, Count);
        for (std::size_t i = 0; i < clampedCount; ++i) {
            const auto& contact = contacts[i];
            if (!contact.valid || contact.confidence <= 0.0f) {
                continue;
            }

            if (!first) {
                first = &contact;
            }
            if (!thumb && hand_collider_semantics::fingerForRole(contact.role) == hand_collider_semantics::HandFinger::Thumb) {
                thumb = &contact;
            }
            if (!opposing && isOpposingFingerRole(contact.role)) {
                opposing = &contact;
            }

            const float weight = (std::max)(0.05f, contact.confidence);
            weightedPoint = add(weightedPoint, mul(contact.point, weight));
            weightedNormal = add(weightedNormal, mul(contact.normal, weight));
            totalWeight += weight;
            ++validCount;
        }

        if (validCount == 0 || totalWeight <= 0.0f) {
            return result;
        }

        result.valid = true;
        result.normal = normalizeOrZero(weightedNormal);
        if (thumb && opposing) {
            result.point = mul(add(thumb->point, opposing->point), 0.5f);
            result.confidence = (std::min)(1.0f, (thumb->confidence + opposing->confidence) * 0.5f);
            result.reason = "thumbOpposition";
            return result;
        }

        if (validCount == 1 && first) {
            result.point = first->point;
            result.confidence = (std::min)(1.0f, first->confidence);
            result.reason = hand_collider_semantics::isPalmRole(first->role) ? "palmOnly" : "singleFinger";
            return result;
        }

        result.point = mul(weightedPoint, 1.0f / totalWeight);
        result.confidence = (std::min)(1.0f, totalWeight / static_cast<float>(validCount));
        result.reason = "contactCluster";
        return result;
    }
}
