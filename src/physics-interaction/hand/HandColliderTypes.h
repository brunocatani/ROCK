#pragma once

/*
 * Hand collider types are grouped here so semantic IDs, collider geometry, and grab pivot math remain one generated-hand-collider surface.
 */


// ---- HandColliderSemanticTypes.h ----

#include <array>
#include <cstddef>
#include <cstdint>

namespace frik::rock::hand_collider_semantics
{
    /*
     * ROCK now treats the live FRIK-mutated root skeleton as the hand collision
     * source of truth. The legacy box is not a gameplay fallback for this path:
     * Havok still needs one constraint body, but that body is a generated palm
     * anchor with the same semantic table as the finger bodies around it.
     */

    enum class HandColliderRuntimeMode : int
    {
        Disabled = 0,
        BoneDerivedHands = 1
    };

    enum class HandFinger : std::uint8_t
    {
        Thumb = 0,
        Index = 1,
        Middle = 2,
        Ring = 3,
        Pinky = 4,
        None = 255
    };

    enum class HandFingerSegment : std::uint8_t
    {
        Base = 0,
        Middle = 1,
        Tip = 2,
        None = 255
    };

    enum class HandColliderRole : std::uint8_t
    {
        PalmAnchor = 0,
        PalmFace,
        PalmBack,
        PalmHeel,
        ThumbPad,
        ThumbBase,
        ThumbMiddle,
        ThumbTip,
        IndexBase,
        IndexMiddle,
        IndexTip,
        MiddleBase,
        MiddleMiddle,
        MiddleTip,
        RingBase,
        RingMiddle,
        RingTip,
        PinkyBase,
        PinkyMiddle,
        PinkyTip
    };

    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
    inline constexpr std::size_t kHandFingerCount = 5;
    inline constexpr std::size_t kHandFingerSegmentCount = 3;
    inline constexpr std::size_t kHandPalmRoleCount = 5;
    inline constexpr std::size_t kHandFingerRoleCount = kHandFingerCount * kHandFingerSegmentCount;
    inline constexpr std::size_t kHandColliderBodyCountPerHand = kHandPalmRoleCount + kHandFingerRoleCount;
    inline constexpr std::size_t kHandSegmentColliderBodyCountPerHand = kHandColliderBodyCountPerHand - 1;

    inline constexpr std::array<HandColliderRole, kHandColliderBodyCountPerHand> kHandColliderRoles{
        HandColliderRole::PalmAnchor,
        HandColliderRole::PalmFace,
        HandColliderRole::PalmBack,
        HandColliderRole::PalmHeel,
        HandColliderRole::ThumbPad,
        HandColliderRole::ThumbBase,
        HandColliderRole::ThumbMiddle,
        HandColliderRole::ThumbTip,
        HandColliderRole::IndexBase,
        HandColliderRole::IndexMiddle,
        HandColliderRole::IndexTip,
        HandColliderRole::MiddleBase,
        HandColliderRole::MiddleMiddle,
        HandColliderRole::MiddleTip,
        HandColliderRole::RingBase,
        HandColliderRole::RingMiddle,
        HandColliderRole::RingTip,
        HandColliderRole::PinkyBase,
        HandColliderRole::PinkyMiddle,
        HandColliderRole::PinkyTip
    };

    inline constexpr std::array<HandColliderRole, kHandSegmentColliderBodyCountPerHand> kHandNonAnchorColliderRoles{
        HandColliderRole::PalmFace,
        HandColliderRole::PalmBack,
        HandColliderRole::PalmHeel,
        HandColliderRole::ThumbPad,
        HandColliderRole::ThumbBase,
        HandColliderRole::ThumbMiddle,
        HandColliderRole::ThumbTip,
        HandColliderRole::IndexBase,
        HandColliderRole::IndexMiddle,
        HandColliderRole::IndexTip,
        HandColliderRole::MiddleBase,
        HandColliderRole::MiddleMiddle,
        HandColliderRole::MiddleTip,
        HandColliderRole::RingBase,
        HandColliderRole::RingMiddle,
        HandColliderRole::RingTip,
        HandColliderRole::PinkyBase,
        HandColliderRole::PinkyMiddle,
        HandColliderRole::PinkyTip
    };

    inline constexpr std::size_t colliderBodyCountPerHand()
    {
        return kHandColliderBodyCountPerHand;
    }

    inline constexpr std::size_t segmentColliderBodyCountPerHand()
    {
        return kHandSegmentColliderBodyCountPerHand;
    }

    inline constexpr HandColliderRuntimeMode sanitizeHandColliderRuntimeMode(int mode)
    {
        return mode == 0 ? HandColliderRuntimeMode::Disabled : HandColliderRuntimeMode::BoneDerivedHands;
    }

    inline constexpr bool isPalmRole(HandColliderRole role)
    {
        return role == HandColliderRole::PalmAnchor ||
               role == HandColliderRole::PalmFace ||
               role == HandColliderRole::PalmBack ||
               role == HandColliderRole::PalmHeel ||
               role == HandColliderRole::ThumbPad;
    }

    inline constexpr bool isFingerRole(HandColliderRole role)
    {
        return !isPalmRole(role);
    }

    inline constexpr bool isTipRole(HandColliderRole role)
    {
        return role == HandColliderRole::ThumbTip ||
               role == HandColliderRole::IndexTip ||
               role == HandColliderRole::MiddleTip ||
               role == HandColliderRole::RingTip ||
               role == HandColliderRole::PinkyTip;
    }

    inline constexpr HandFinger fingerForRole(HandColliderRole role)
    {
        switch (role) {
        case HandColliderRole::ThumbBase:
        case HandColliderRole::ThumbMiddle:
        case HandColliderRole::ThumbTip:
        case HandColliderRole::ThumbPad:
            return HandFinger::Thumb;
        case HandColliderRole::IndexBase:
        case HandColliderRole::IndexMiddle:
        case HandColliderRole::IndexTip:
            return HandFinger::Index;
        case HandColliderRole::MiddleBase:
        case HandColliderRole::MiddleMiddle:
        case HandColliderRole::MiddleTip:
            return HandFinger::Middle;
        case HandColliderRole::RingBase:
        case HandColliderRole::RingMiddle:
        case HandColliderRole::RingTip:
            return HandFinger::Ring;
        case HandColliderRole::PinkyBase:
        case HandColliderRole::PinkyMiddle:
        case HandColliderRole::PinkyTip:
            return HandFinger::Pinky;
        default:
            return HandFinger::None;
        }
    }

    inline constexpr HandFingerSegment segmentForRole(HandColliderRole role)
    {
        switch (role) {
        case HandColliderRole::ThumbBase:
        case HandColliderRole::IndexBase:
        case HandColliderRole::MiddleBase:
        case HandColliderRole::RingBase:
        case HandColliderRole::PinkyBase:
            return HandFingerSegment::Base;
        case HandColliderRole::ThumbMiddle:
        case HandColliderRole::IndexMiddle:
        case HandColliderRole::MiddleMiddle:
        case HandColliderRole::RingMiddle:
        case HandColliderRole::PinkyMiddle:
            return HandFingerSegment::Middle;
        case HandColliderRole::ThumbTip:
        case HandColliderRole::IndexTip:
        case HandColliderRole::MiddleTip:
        case HandColliderRole::RingTip:
        case HandColliderRole::PinkyTip:
            return HandFingerSegment::Tip;
        default:
            return HandFingerSegment::None;
        }
    }

    inline constexpr HandColliderRole roleForFingerSegment(HandFinger finger, HandFingerSegment segment)
    {
        switch (finger) {
        case HandFinger::Thumb:
            return segment == HandFingerSegment::Base ? HandColliderRole::ThumbBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::ThumbMiddle :
                   HandColliderRole::ThumbTip;
        case HandFinger::Index:
            return segment == HandFingerSegment::Base ? HandColliderRole::IndexBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::IndexMiddle :
                   HandColliderRole::IndexTip;
        case HandFinger::Middle:
            return segment == HandFingerSegment::Base ? HandColliderRole::MiddleBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::MiddleMiddle :
                   HandColliderRole::MiddleTip;
        case HandFinger::Ring:
            return segment == HandFingerSegment::Base ? HandColliderRole::RingBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::RingMiddle :
                   HandColliderRole::RingTip;
        case HandFinger::Pinky:
            return segment == HandFingerSegment::Base ? HandColliderRole::PinkyBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::PinkyMiddle :
                   HandColliderRole::PinkyTip;
        default:
            return HandColliderRole::PalmAnchor;
        }
    }

    inline constexpr const char* roleName(HandColliderRole role)
    {
        switch (role) {
        case HandColliderRole::PalmAnchor: return "PalmAnchor";
        case HandColliderRole::PalmFace: return "PalmFace";
        case HandColliderRole::PalmBack: return "PalmBack";
        case HandColliderRole::PalmHeel: return "PalmHeel";
        case HandColliderRole::ThumbPad: return "ThumbPad";
        case HandColliderRole::ThumbBase: return "ThumbBase";
        case HandColliderRole::ThumbMiddle: return "ThumbMiddle";
        case HandColliderRole::ThumbTip: return "ThumbTip";
        case HandColliderRole::IndexBase: return "IndexBase";
        case HandColliderRole::IndexMiddle: return "IndexMiddle";
        case HandColliderRole::IndexTip: return "IndexTip";
        case HandColliderRole::MiddleBase: return "MiddleBase";
        case HandColliderRole::MiddleMiddle: return "MiddleMiddle";
        case HandColliderRole::MiddleTip: return "MiddleTip";
        case HandColliderRole::RingBase: return "RingBase";
        case HandColliderRole::RingMiddle: return "RingMiddle";
        case HandColliderRole::RingTip: return "RingTip";
        case HandColliderRole::PinkyBase: return "PinkyBase";
        case HandColliderRole::PinkyMiddle: return "PinkyMiddle";
        case HandColliderRole::PinkyTip: return "PinkyTip";
        }
        return "Unknown";
    }
}

// ---- HandBoneColliderGeometryMath.h ----

#include "physics-interaction/debug/DebugAxisMath.h"
#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace frik::rock::hand_bone_collider_geometry_math
{
    /*
     * These helpers convert the validated live bone graph into stable collider
     * frames. Child bone translation defines each segment direction, while bone
     * rotation supplies roll. Palm landmarks are derived from the hand node plus
     * live finger bases so the new constraint anchor follows the visual hand
     * instead of the removed legacy box.
     */

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
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x);
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
    inline Vector normalizeOr(const Vector& value, const Vector& fallback)
    {
        const float len = length(value);
        if (len <= 1.0e-5f || !std::isfinite(len)) {
            return fallback;
        }
        return mul(value, 1.0f / len);
    }

    template <class Vector>
    inline Vector projectOntoPlane(const Vector& value, const Vector& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    template <class Matrix, class Vector>
    inline Matrix matrixFromAxes(const Vector& xAxis, const Vector& yAxis, const Vector& zAxis)
    {
        Matrix matrix{};
        matrix.entry[0][0] = xAxis.x;
        matrix.entry[1][0] = xAxis.y;
        matrix.entry[2][0] = xAxis.z;
        matrix.entry[0][1] = yAxis.x;
        matrix.entry[1][1] = yAxis.y;
        matrix.entry[2][1] = yAxis.z;
        matrix.entry[0][2] = zAxis.x;
        matrix.entry[1][2] = zAxis.y;
        matrix.entry[2][2] = zAxis.z;
        return matrix;
    }

    template <class Transform, class Vector>
    struct BoneColliderFrameInput
    {
        Transform start{};
        Transform end{};
        Transform previous{};
        float radius = 0.5f;
        float convexRadius = 0.1f;
        bool extrapolateFromPrevious = false;
        float extrapolatedLengthScale = 0.65f;
    };

    template <class Transform, class Vector>
    struct BoneColliderFrameResult
    {
        Transform transform{};
        Vector xAxis{};
        Vector yAxis{};
        Vector zAxis{};
        Vector backAxis{};
        float length = 0.0f;
        float radius = 0.0f;
        float convexRadius = 0.0f;
        bool valid = false;
    };

    template <class Transform, class Vector>
    inline BoneColliderFrameResult<Transform, Vector> buildSegmentColliderFrame(const BoneColliderFrameInput<Transform, Vector>& input)
    {
        BoneColliderFrameResult<Transform, Vector> result{};
        result.radius = (std::max)(0.01f, input.radius);
        result.convexRadius = (std::max)(0.0f, input.convexRadius);

        Vector start = input.start.translate;
        Vector end = input.end.translate;
        if (input.extrapolateFromPrevious) {
            const Vector parentToTip = sub(input.start.translate, input.previous.translate);
            const float parentLength = length(parentToTip);
            if (parentLength <= 1.0e-5f) {
                return result;
            }
            end = add(input.start.translate, mul(parentToTip, input.extrapolatedLengthScale));
        }

        const Vector segment = sub(end, start);
        result.length = length(segment);
        if (result.length <= 1.0e-5f) {
            return result;
        }

        const Vector fallbackX = makeVector<Vector>(1.0f, 0.0f, 0.0f);
        const Vector fallbackY = makeVector<Vector>(0.0f, 1.0f, 0.0f);
        const Vector fallbackZ = makeVector<Vector>(0.0f, 0.0f, 1.0f);
        result.xAxis = normalizeOr(segment, fallbackX);

        Vector rollHint = debug_axis_math::rotateNiLocalToWorld(input.start.rotate, fallbackY);
        rollHint = projectOntoPlane(rollHint, result.xAxis);
        result.yAxis = normalizeOr(rollHint, std::fabs(dot(result.xAxis, fallbackY)) < 0.9f ? fallbackY : fallbackZ);
        result.zAxis = normalizeOr(cross(result.xAxis, result.yAxis), fallbackZ);
        result.yAxis = normalizeOr(cross(result.zAxis, result.xAxis), fallbackY);

        result.transform = input.start;
        result.transform.translate = add(start, mul(segment, 0.5f));
        result.transform.rotate = matrixFromAxes<decltype(result.transform.rotate)>(result.xAxis, result.yAxis, result.zAxis);
        result.transform.scale = 1.0f;
        result.valid = true;
        return result;
    }

    template <class Vector>
    inline std::vector<Vector> makeCapsuleLikeHullPoints(float lengthValue, float radius)
    {
        const float half = (std::max)(0.05f, lengthValue) * 0.5f;
        const float r = (std::max)(0.01f, radius);
        std::vector<Vector> points;
        points.reserve(16);
        for (float x : { -half, half }) {
            points.push_back(makeVector<Vector>(x, r, 0.0f));
            points.push_back(makeVector<Vector>(x, -r, 0.0f));
            points.push_back(makeVector<Vector>(x, 0.0f, r));
            points.push_back(makeVector<Vector>(x, 0.0f, -r));
            points.push_back(makeVector<Vector>(x, r * 0.7071067f, r * 0.7071067f));
            points.push_back(makeVector<Vector>(x, -r * 0.7071067f, r * 0.7071067f));
            points.push_back(makeVector<Vector>(x, r * 0.7071067f, -r * 0.7071067f));
            points.push_back(makeVector<Vector>(x, -r * 0.7071067f, -r * 0.7071067f));
        }
        return points;
    }

    template <class Vector>
    inline std::vector<Vector> makeRoundedBoxHullPoints(float lengthValue, float widthValue, float thicknessValue)
    {
        const float hx = (std::max)(0.05f, lengthValue) * 0.5f;
        const float hy = (std::max)(0.05f, widthValue) * 0.5f;
        const float hz = (std::max)(0.05f, thicknessValue) * 0.5f;
        std::vector<Vector> points;
        points.reserve(14);

        for (float x : { -hx, hx }) {
            for (float y : { -hy, hy }) {
                for (float z : { -hz, hz }) {
                    points.push_back(makeVector<Vector>(x, y, z));
                }
            }
        }

        points.push_back(makeVector<Vector>(-hx, 0.0f, 0.0f));
        points.push_back(makeVector<Vector>(hx, 0.0f, 0.0f));
        points.push_back(makeVector<Vector>(0.0f, -hy, 0.0f));
        points.push_back(makeVector<Vector>(0.0f, hy, 0.0f));
        points.push_back(makeVector<Vector>(0.0f, 0.0f, -hz));
        points.push_back(makeVector<Vector>(0.0f, 0.0f, hz));
        return points;
    }

    template <class Transform, class Vector>
    inline BoneColliderFrameResult<Transform, Vector> buildPalmAnchorFrame(
        const Transform& hand,
        const std::array<Vector, 5>& fingerBases,
        const Vector& backOfHandDirection,
        float palmDepth)
    {
        BoneColliderFrameResult<Transform, Vector> result{};
        const Vector fallbackX = makeVector<Vector>(1.0f, 0.0f, 0.0f);
        const Vector fallbackY = makeVector<Vector>(0.0f, 1.0f, 0.0f);
        const Vector fallbackZ = makeVector<Vector>(0.0f, 0.0f, 1.0f);

        Vector fingerCenter{};
        for (const auto& point : fingerBases) {
            fingerCenter = add(fingerCenter, point);
        }
        fingerCenter = mul(fingerCenter, 1.0f / static_cast<float>(fingerBases.size()));
        Vector palmCenter = hand.translate;
        for (const auto& point : fingerBases) {
            palmCenter = add(palmCenter, point);
        }
        palmCenter = mul(palmCenter, 1.0f / static_cast<float>(fingerBases.size() + 1));

        result.xAxis = normalizeOr(sub(fingerCenter, hand.translate), fallbackX);
        result.backAxis = normalizeOr(backOfHandDirection, fallbackZ);
        result.yAxis = normalizeOr(cross(result.backAxis, result.xAxis), fallbackY);
        result.xAxis = normalizeOr(cross(result.yAxis, result.backAxis), fallbackX);
        result.zAxis = result.backAxis;
        result.length = (std::max)(1.0f, length(sub(fingerCenter, hand.translate)));
        result.radius = (std::max)(0.5f, result.length * 0.45f);
        result.convexRadius = 0.1f;

        result.transform = hand;
        const float currentBackOffset = dot(sub(palmCenter, hand.translate), result.backAxis);
        palmCenter = sub(palmCenter, mul(result.backAxis, currentBackOffset));
        result.transform.translate = add(palmCenter, mul(result.backAxis, -std::fabs(palmDepth) / 3.0f));
        result.transform.rotate = matrixFromAxes<decltype(result.transform.rotate)>(result.xAxis, result.yAxis, result.zAxis);
        result.transform.scale = 1.0f;
        result.valid = true;
        return result;
    }
}

// ---- HandBoneGrabPivotMath.h ----


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
