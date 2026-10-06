#pragma once

#include "physics-interaction/VectorMath.h"

/*
 * Grab contact policy is grouped here because contact source, evidence, patch, surface, opposition, and multi-finger contact math are one contact interpretation pipeline.
 */


// ---- GrabContactSourcePolicy.h ----

/*
 * Physics collision identifies the object and body; the rendered mesh supplies
 * the grabbed point when one is found, and the collision point is the fallback.
 * In mesh-only mode a contact patch must snap back to the rendered mesh before
 * it can move the pivot.
 */

namespace rock::grab_contact_source_policy
{
    // A weapon may have one native collision owner and several visual sibling
    // branches. Prove both branches belong to that reference without crossing
    // another collision owner. No node is retained beyond this acquisition.
    template <class Node>
    [[nodiscard]] bool isSingleBodyOwnerlessVisualBranch(
        const Node* root, const Node* bodyOwner, const Node* visualNode)
    {
        if (!root || !bodyOwner || !bodyOwner->collisionObject || !visualNode) {
            return false;
        }
        const Node* ownerAncestor = bodyOwner;
        for (unsigned depth = 0; ownerAncestor && depth < 64; ++depth) {
            if (ownerAncestor == root) {
                for (unsigned visualDepth = 0; visualNode && visualDepth < 64; ++visualDepth) {
                    if (visualNode == root) return true;
                    if (visualNode->collisionObject) return false;
                    visualNode = visualNode->parent;
                }
                return false;
            }
            ownerAncestor = ownerAncestor->parent;
        }
        return false;
    }

}

// ---- GrabContactPatchMath.h ----

/*
 * Small vector helpers shared by the grip support model, plus the sample
 * record the held grab frame stores for contact-shape classification.
 */

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace rock::grab_contact_patch_math
{
    inline constexpr std::size_t kContactPatchProbePatternSampleCount = 9;

    template <class Vector>
    struct GrabContactPatchSample
    {
        std::uint32_t bodyId = 0x7FFF'FFFF;
        Vector point{};
        Vector normal{};
        float fraction = 1.0f;
        bool accepted = false;
        const char* rejectionReason = "none";
    };

    template <class Vector>
    inline Vector add(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
    }

    template <class Vector>
    inline Vector sub(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    template <class Vector>
    inline Vector mul(const Vector& value, float scale)
    {
        return Vector{ value.x * scale, value.y * scale, value.z * scale };
    }

    template <class Vector>
    inline float dot(const Vector& lhs, const Vector& rhs)
    {
        return vector_math::dot(lhs, rhs);
    }

    template <class Vector>
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return vector_math::cross(lhs, rhs);
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return vector_math::lengthSquared(value);
    }

    template <class Vector>
    inline float length(const Vector& value)
    {
        return std::sqrt(lengthSquared(value));
    }

    template <class Vector>
    inline bool finiteVector(const Vector& value)
    {
        return vector_math::hasFiniteComponents(value);
    }

    template <class Vector>
    inline Vector normalizeOrZero(const Vector& value)
    {
        const float len = length(value);
        if (len <= 1.0e-6f || !std::isfinite(len)) {
            return Vector{};
        }
        return mul(value, 1.0f / len);
    }

    template <class Vector>
    inline Vector negate(const Vector& value)
    {
        return Vector{ -value.x, -value.y, -value.z };
    }

    template <class Vector>
    inline Vector projectOntoPlane(const Vector& value, const Vector& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    template <class Vector>
    inline Vector stablePerpendicular(const Vector& normal)
    {
        const Vector xAxis{ 1.0f, 0.0f, 0.0f };
        const Vector yAxis{ 0.0f, 1.0f, 0.0f };
        const Vector zAxis{ 0.0f, 0.0f, 1.0f };
        const Vector hint = std::fabs(dot(normal, xAxis)) < 0.85f ? xAxis : (std::fabs(dot(normal, yAxis)) < 0.85f ? yAxis : zAxis);
        return normalizeOrZero(projectOntoPlane(hint, normal));
    }
}

// ---- GrabSupportModelMath.h ----

/*
 * The support model classifies how a grabbed object sits in the hand (opposed
 * pinch, long handle, palm wrap, same face, single point) from a few mesh
 * probes around the grab point. The held drive reads that class; the grab
 * point itself is chosen once by the surface resolver.
 */

namespace rock::grab_support_model_math
{
    enum class GripSupportKind : std::uint8_t
    {
        None,
        SinglePoint,
        SameSurface,
        OpposedPinch,
        LongHandleAxis,
        PalmWrap
    };

    enum class GripSupportRole : std::uint8_t
    {
        Main,
        ContactPatch,
        AcrossPositive,
        AcrossNegative,
        FingerForward,
        FingerBack,
        ThumbPad,
        IndexPad
    };

    template <class Vector>
    struct GripSupportSample
    {
        Vector point{};
        Vector normal{};
        GripSupportRole role = GripSupportRole::Main;
        bool valid = false;
    };

    template <class Vector>
    struct GripSupportModelInput
    {
        Vector anchorPoint{};
        Vector anchorNormal{};
        Vector palmNormal{};
        Vector acrossPalmAxis{};
        Vector fingerAxis{};
        Vector pinchAxis{};
        Vector objectLongAxis{};
        const GripSupportSample<Vector>* samples = nullptr;
        std::size_t sampleCount = 0;
        float longObjectLeverGameUnits = 0.0f;
        float objectLongAxisSpanGameUnits = 0.0f;
        float smallObjectReferenceLeverGameUnits = 12.0f;
        float longObjectReferenceLeverGameUnits = 24.0f;
        float maxPivotShiftGameUnits = 6.0f;
        float minOpposedSpanGameUnits = 1.0f;
        bool pinchSeat = false;
    };

    template <class Vector>
    struct GripSupportModel
    {
        Vector pivotPoint{};
        Vector supportAxis{};
        Vector supportNormal{};
        GripSupportKind kind = GripSupportKind::None;
        float confidence = 0.0f;
        float supportSpanGameUnits = 0.0f;
        float pivotShiftGameUnits = 0.0f;
        std::size_t acceptedSampleCount = 0;
        bool valid = false;
        bool canAuthorPivot = false;
        const char* reason = "notEvaluated";
    };

    inline const char* gripSupportKindName(GripSupportKind kind)
    {
        switch (kind) {
        case GripSupportKind::SinglePoint:
            return "singlePoint";
        case GripSupportKind::SameSurface:
            return "sameSurface";
        case GripSupportKind::OpposedPinch:
            return "opposedPinch";
        case GripSupportKind::LongHandleAxis:
            return "longHandleAxis";
        case GripSupportKind::PalmWrap:
            return "palmWrap";
        default:
            return "none";
        }
    }

    inline bool rolesAreOpposed(GripSupportRole lhs, GripSupportRole rhs)
    {
        return (lhs == GripSupportRole::AcrossPositive && rhs == GripSupportRole::AcrossNegative) ||
               (lhs == GripSupportRole::AcrossNegative && rhs == GripSupportRole::AcrossPositive) ||
               (lhs == GripSupportRole::FingerForward && rhs == GripSupportRole::FingerBack) ||
               (lhs == GripSupportRole::FingerBack && rhs == GripSupportRole::FingerForward) ||
               (lhs == GripSupportRole::ThumbPad && rhs == GripSupportRole::IndexPad) ||
               (lhs == GripSupportRole::IndexPad && rhs == GripSupportRole::ThumbPad);
    }

    template <class Vector>
    inline Vector normalizedOrFallback(const Vector& value, const Vector& fallback)
    {
        Vector normalized = grab_contact_patch_math::normalizeOrZero(value);
        if (grab_contact_patch_math::lengthSquared(normalized) > 0.0f) {
            return normalized;
        }
        return grab_contact_patch_math::normalizeOrZero(fallback);
    }

    template <class Vector>
    inline float sanitizedPositive(float value, float fallback)
    {
        return std::isfinite(value) && value > 0.0f ? value : fallback;
    }

    template <class Vector>
    inline Vector normalizeSupportNormalForPalm(const Vector& normal, const Vector& palmNormal)
    {
        Vector result = grab_contact_patch_math::normalizeOrZero(normal);
        const Vector palm = grab_contact_patch_math::normalizeOrZero(palmNormal);
        if (grab_contact_patch_math::lengthSquared(result) <= 0.0f ||
            grab_contact_patch_math::lengthSquared(palm) <= 0.0f) {
            return result;
        }
        if (grab_contact_patch_math::dot(result, palm) > 0.0f) {
            result = grab_contact_patch_math::negate(result);
        }
        return result;
    }

    template <class Vector>
    inline bool axisAlignsWithAnyGripAxis(const Vector& axis,
        const Vector& acrossPalmAxis,
        const Vector& fingerAxis,
        const Vector& pinchAxis)
    {
        const Vector normalized = grab_contact_patch_math::normalizeOrZero(axis);
        if (grab_contact_patch_math::lengthSquared(normalized) <= 0.0f) {
            return false;
        }

        const Vector axes[] = {
            grab_contact_patch_math::normalizeOrZero(acrossPalmAxis),
            grab_contact_patch_math::normalizeOrZero(fingerAxis),
            grab_contact_patch_math::normalizeOrZero(pinchAxis),
        };
        for (const auto& candidateAxis : axes) {
            if (grab_contact_patch_math::lengthSquared(candidateAxis) <= 0.0f) {
                continue;
            }
            if (std::fabs(grab_contact_patch_math::dot(normalized, candidateAxis)) >= 0.60f) {
                return true;
            }
        }
        return false;
    }

    template <class Vector>
    inline GripSupportModel<Vector> buildGripSupportModel(const GripSupportModelInput<Vector>& input)
    {
        GripSupportModel<Vector> result{};
        result.pivotPoint = input.anchorPoint;
        result.supportNormal = normalizeSupportNormalForPalm(input.anchorNormal, input.palmNormal);
        if (grab_contact_patch_math::lengthSquared(result.supportNormal) <= 0.0f) {
            result.supportNormal = grab_contact_patch_math::normalizeOrZero(input.palmNormal);
        }
        result.supportAxis = grab_contact_patch_math::stablePerpendicular(
            grab_contact_patch_math::lengthSquared(result.supportNormal) > 0.0f ?
                result.supportNormal :
                Vector{ 0.0f, 0.0f, 1.0f });

        if (!grab_contact_patch_math::finiteVector(input.anchorPoint)) {
            result.reason = "invalidAnchor";
            return result;
        }

        const float smallReference = (std::max)(1.0f, sanitizedPositive<Vector>(input.smallObjectReferenceLeverGameUnits, 12.0f));
        const float longReference = (std::max)(smallReference, sanitizedPositive<Vector>(input.longObjectReferenceLeverGameUnits, 24.0f));
        const float maxPivotShift = (std::max)(0.0f, sanitizedPositive<Vector>(input.maxPivotShiftGameUnits, 6.0f));
        const float minOpposedSpan = (std::max)(0.25f, sanitizedPositive<Vector>(input.minOpposedSpanGameUnits, 1.0f));
        const bool smallObject = input.longObjectLeverGameUnits > 0.0f && input.longObjectLeverGameUnits <= smallReference;
        const bool longObject =
            input.longObjectLeverGameUnits >= longReference * 1.35f ||
            input.objectLongAxisSpanGameUnits >= longReference * 1.35f;

        std::vector<GripSupportSample<Vector>> accepted;
        accepted.reserve(input.sampleCount + 1);
        accepted.push_back(GripSupportSample<Vector>{
            .point = input.anchorPoint,
            .normal = result.supportNormal,
            .role = GripSupportRole::Main,
            .valid = true,
        });
        if (input.samples) {
            for (std::size_t i = 0; i < input.sampleCount; ++i) {
                const auto& sample = input.samples[i];
                if (!sample.valid || !grab_contact_patch_math::finiteVector(sample.point)) {
                    continue;
                }
                auto acceptedSample = sample;
                acceptedSample.normal = normalizeSupportNormalForPalm(sample.normal, input.palmNormal);
                if (grab_contact_patch_math::lengthSquared(acceptedSample.normal) <= 0.0f) {
                    acceptedSample.normal = result.supportNormal;
                }
                accepted.push_back(acceptedSample);
            }
        }
        result.acceptedSampleCount = accepted.size();

        if (accepted.size() <= 1) {
            result.kind = GripSupportKind::SinglePoint;
            result.valid = true;
            result.confidence = 0.25f;
            result.reason = "singleAnchorPoint";
            return result;
        }

        struct Pair
        {
            std::size_t a = 0;
            std::size_t b = 0;
            Vector axis{};
            Vector midpoint{};
            float span = 0.0f;
            float pivotShift = 0.0f;
            bool valid = false;
        };

        Pair majorPair{};
        Pair opposedPair{};
        Vector centroid{};
        for (const auto& sample : accepted) {
            centroid = grab_contact_patch_math::add(centroid, sample.point);
        }
        centroid = grab_contact_patch_math::mul(centroid, 1.0f / static_cast<float>(accepted.size()));

        for (std::size_t a = 0; a < accepted.size(); ++a) {
            for (std::size_t b = a + 1; b < accepted.size(); ++b) {
                const Vector delta = grab_contact_patch_math::sub(accepted[b].point, accepted[a].point);
                const float span = grab_contact_patch_math::length(delta);
                if (!std::isfinite(span) || span <= 0.0001f) {
                    continue;
                }

                Pair pair{};
                pair.a = a;
                pair.b = b;
                pair.axis = grab_contact_patch_math::mul(delta, 1.0f / span);
                pair.midpoint = grab_contact_patch_math::mul(grab_contact_patch_math::add(accepted[a].point, accepted[b].point), 0.5f);
                pair.span = span;
                pair.pivotShift = grab_contact_patch_math::length(grab_contact_patch_math::sub(pair.midpoint, input.anchorPoint));
                pair.valid = true;

                if (!majorPair.valid || pair.span > majorPair.span) {
                    majorPair = pair;
                }

                const bool explicitOpposedRoles = rolesAreOpposed(accepted[a].role, accepted[b].role);
                const bool patchOnlyPair =
                    (accepted[a].role == GripSupportRole::Main || accepted[a].role == GripSupportRole::ContactPatch) &&
                    (accepted[b].role == GripSupportRole::Main || accepted[b].role == GripSupportRole::ContactPatch);
                const Vector normalA = grab_contact_patch_math::normalizeOrZero(accepted[a].normal);
                const Vector normalB = grab_contact_patch_math::normalizeOrZero(accepted[b].normal);
                const bool normalsOpposed =
                    grab_contact_patch_math::lengthSquared(normalA) > 0.0f &&
                    grab_contact_patch_math::lengthSquared(normalB) > 0.0f &&
                    grab_contact_patch_math::dot(normalA, normalB) <= -0.25f;
                const bool axisAligned = axisAlignsWithAnyGripAxis(pair.axis, input.acrossPalmAxis, input.fingerAxis, input.pinchAxis);
                if (pair.span >= minOpposedSpan &&
                    normalsOpposed &&
                    (explicitOpposedRoles || (!patchOnlyPair && axisAligned)) &&
                    pair.pivotShift <= maxPivotShift) {
                    if (!opposedPair.valid ||
                        (explicitOpposedRoles && !rolesAreOpposed(accepted[opposedPair.a].role, accepted[opposedPair.b].role)) ||
                        pair.span > opposedPair.span) {
                        opposedPair = pair;
                    }
                }
            }
        }

        if ((input.pinchSeat || smallObject) && opposedPair.valid) {
            result.kind = GripSupportKind::OpposedPinch;
            result.pivotPoint = opposedPair.midpoint;
            result.supportAxis = opposedPair.axis;
            result.supportSpanGameUnits = opposedPair.span;
            result.pivotShiftGameUnits = opposedPair.pivotShift;
            result.confidence = input.pinchSeat ? 0.95f : 0.85f;
            result.valid = true;
            result.canAuthorPivot = true;
            result.reason = input.pinchSeat ? "pinchOpposedSupport" : "smallObjectOpposedSupport";
            return result;
        }

        if (longObject && majorPair.valid) {
            const Vector objectLongAxis = normalizedOrFallback(input.objectLongAxis, majorPair.axis);
            result.kind = GripSupportKind::LongHandleAxis;
            result.supportAxis = objectLongAxis;
            result.supportSpanGameUnits = (std::max)(majorPair.span, std::isfinite(input.objectLongAxisSpanGameUnits) ? input.objectLongAxisSpanGameUnits : 0.0f);
            result.pivotShiftGameUnits = majorPair.pivotShift;
            result.valid = true;
            result.confidence = opposedPair.valid ? 0.85f : 0.65f;
            result.reason = opposedPair.valid ? "longHandleOpposedCenterline" : "longHandleAxisOnly";
            if (opposedPair.valid) {
                result.pivotPoint = opposedPair.midpoint;
                result.supportAxis = objectLongAxis;
                result.supportSpanGameUnits = (std::max)(result.supportSpanGameUnits, opposedPair.span);
                result.pivotShiftGameUnits = opposedPair.pivotShift;
                result.canAuthorPivot = true;
            }
            return result;
        }

        const Vector acrossAxis = grab_contact_patch_math::normalizeOrZero(input.acrossPalmAxis);
        const Vector fingerAxis = grab_contact_patch_math::normalizeOrZero(input.fingerAxis);
        float minAcross = (std::numeric_limits<float>::max)();
        float maxAcross = -(std::numeric_limits<float>::max)();
        float minFinger = (std::numeric_limits<float>::max)();
        float maxFinger = -(std::numeric_limits<float>::max)();
        bool hasDiverseSupportNormals = false;
        for (std::size_t a = 0; a < accepted.size(); ++a) {
            if (accepted[a].role == GripSupportRole::Main) {
                continue;
            }
            for (std::size_t b = a + 1; b < accepted.size(); ++b) {
                if (accepted[b].role == GripSupportRole::Main) {
                    continue;
                }
                const Vector normalA = grab_contact_patch_math::normalizeOrZero(accepted[a].normal);
                const Vector normalB = grab_contact_patch_math::normalizeOrZero(accepted[b].normal);
                if (grab_contact_patch_math::lengthSquared(normalA) > 0.0f &&
                    grab_contact_patch_math::lengthSquared(normalB) > 0.0f &&
                    grab_contact_patch_math::dot(normalA, normalB) <= 0.55f) {
                    hasDiverseSupportNormals = true;
                }
            }
        }
        for (const auto& sample : accepted) {
            const Vector delta = grab_contact_patch_math::sub(sample.point, input.anchorPoint);
            if (grab_contact_patch_math::lengthSquared(acrossAxis) > 0.0f) {
                const float value = grab_contact_patch_math::dot(delta, acrossAxis);
                minAcross = (std::min)(minAcross, value);
                maxAcross = (std::max)(maxAcross, value);
            }
            if (grab_contact_patch_math::lengthSquared(fingerAxis) > 0.0f) {
                const float value = grab_contact_patch_math::dot(delta, fingerAxis);
                minFinger = (std::min)(minFinger, value);
                maxFinger = (std::max)(maxFinger, value);
            }
        }

        const float acrossSpan = maxAcross > minAcross ? maxAcross - minAcross : 0.0f;
        const float fingerSpan = maxFinger > minFinger ? maxFinger - minFinger : 0.0f;
        const float centroidShift = grab_contact_patch_math::length(grab_contact_patch_math::sub(centroid, input.anchorPoint));
        if (accepted.size() >= 4 &&
            (opposedPair.valid || hasDiverseSupportNormals) &&
            acrossSpan >= minOpposedSpan &&
            fingerSpan >= minOpposedSpan &&
            centroidShift <= maxPivotShift) {
            result.kind = GripSupportKind::PalmWrap;
            result.pivotPoint = centroid;
            result.supportAxis = majorPair.valid ? majorPair.axis : result.supportAxis;
            result.supportSpanGameUnits = majorPair.valid ? majorPair.span : (std::max)(acrossSpan, fingerSpan);
            result.pivotShiftGameUnits = centroidShift;
            result.confidence = 0.80f;
            result.valid = true;
            result.canAuthorPivot = true;
            result.reason = "palmWrapSupport";
            return result;
        }

        result.kind = accepted.size() >= 2 ? GripSupportKind::SameSurface : GripSupportKind::SinglePoint;
        result.supportAxis = majorPair.valid ? majorPair.axis : result.supportAxis;
        result.supportSpanGameUnits = majorPair.valid ? majorPair.span : 0.0f;
        result.pivotShiftGameUnits = 0.0f;
        result.confidence = accepted.size() >= 2 ? 0.55f : 0.25f;
        result.valid = true;
        result.canAuthorPivot = false;
        result.reason = accepted.size() >= 2 ? "sameSurfaceEvidenceOnly" : "singlePointEvidenceOnly";
        return result;
    }
}

// ---- GrabMultiFingerContactMath.h ----

/*
 * Multi-finger grab validation is intentionally separate from the runtime Havok
 * and mesh queries. ROCK preserves a coherent object-in-hand transform once a
 * contact point is chosen. The earlier single semantic/palm fallback could
 * choose that point from incomplete evidence, which let one finger or a
 * palm-plane probe define the whole grab. This layer requires a grouped set of
 * distinct finger surface patches first, then captures the object-to-grip
 * transform so the held object keeps its current orientation instead of snapping
 * to a fingertip or palm axis.
 */

#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
