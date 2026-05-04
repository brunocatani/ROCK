#pragma once

/*
 * Grab contact policy is grouped here because contact source, evidence, patch, surface, opposition, and multi-finger contact math are one contact interpretation pipeline.
 */


// ---- GrabContactSourcePolicy.h ----

/*
 * Mesh-authoritative grab contact keeps ROCK's dynamic object pipeline aligned
 * with HIGGS' graphics-geometry contact model: physics collision identifies the
 * object and body, while the rendered mesh supplies the grabbed point and frame.
 * FO4VR collision bubbles can sit away from the visible surface, so this policy
 * prevents raw hknp hit points from silently becoming pivot B when mesh-only
 * contact is enabled. Contact-patch casts are still allowed to probe the palm
 * region, but mesh-only grabs may use that patch only after it snaps back to the
 * resolved object's rendered mesh.
 */

namespace frik::rock::grab_contact_source_policy
{
    struct GrabContactSourcePolicy
    {
        bool allowCollisionGrabPoint = true;
        bool allowContactPatchPivot = true;
        bool requireContactPatchMeshSnap = false;
        bool requireMeshSurface = false;
        bool failWithoutMesh = false;
        const char* reason = "legacyContactSources";
    };

    inline GrabContactSourcePolicy evaluateGrabContactSourcePolicy(bool meshContactOnly, bool requireMeshContact, bool hasMeshContact, bool hasAuthoredGrabNode)
    {
        GrabContactSourcePolicy policy{};
        policy.allowCollisionGrabPoint = !meshContactOnly;
        policy.allowContactPatchPivot = true;
        policy.requireContactPatchMeshSnap = meshContactOnly && !hasAuthoredGrabNode;
        policy.requireMeshSurface = meshContactOnly && requireMeshContact && !hasAuthoredGrabNode;

        if (hasAuthoredGrabNode) {
            policy.requireContactPatchMeshSnap = false;
            policy.failWithoutMesh = false;
            policy.reason = "authoredGrabNode";
            return policy;
        }

        if (!meshContactOnly) {
            policy.failWithoutMesh = false;
            policy.reason = "legacyContactSources";
            return policy;
        }

        if (hasMeshContact) {
            policy.failWithoutMesh = false;
            policy.reason = "meshContact";
            return policy;
        }

        policy.failWithoutMesh = requireMeshContact;
        policy.reason = requireMeshContact ? "meshContactRequired" : "meshContactMissing";
        return policy;
    }

    inline bool shouldAcceptContactPatchPivot(const GrabContactSourcePolicy& policy, bool patchValid, bool meshSnapped)
    {
        if (!policy.allowContactPatchPivot || !patchValid) {
            return false;
        }
        return !policy.requireContactPatchMeshSnap || meshSnapped;
    }

    inline bool shouldRejectMeshOwnerMismatch(bool meshContactOnly, bool requireMeshContact, bool hasMeshContact, bool hasAuthoredGrabNode, bool ownerMatchesResolvedBody)
    {
        return meshContactOnly && requireMeshContact && hasMeshContact && !hasAuthoredGrabNode && !ownerMatchesResolvedBody;
    }
}

// ---- GrabContactEvidencePolicy.h ----

/*
 * Hybrid grab contact evidence exists because the per-finger Havok contact
 * stream is not guaranteed to contain three fresh semantic contacts on the
 * exact frame the grab button is pressed. HIGGS treats finger geometry as a
 * pose/contact-quality layer around a stable object-in-hand transform, so ROCK
 * keeps reliable mesh contact as the baseline and upgrades the grab when
 * finger/thumb evidence is available instead of letting missing contact events
 * reject otherwise valid grabs.
 */

#include <algorithm>
#include <cstdint>

namespace frik::rock::grab_contact_evidence_policy
{
    enum class GrabContactQualityMode : int
    {
        LegacyPermissive = 0,
        HybridEvidence = 1,
        StrictMultiFinger = 2
    };

    enum class GrabContactEvidenceLevel : std::uint8_t
    {
        Rejected,
        LegacyPermissive,
        BaselinePatch,
        EnhancedFingerPatch,
        HighConfidenceFingerGrip
    };

    struct GrabContactEvidenceInput
    {
        int qualityMode = static_cast<int>(GrabContactQualityMode::HybridEvidence);
        bool multiFingerValidationEnabled = true;
        bool contactPatchAccepted = false;
        bool contactPatchMeshSnapped = false;
        bool contactPatchReliable = false;
        float contactPatchConfidence = 0.0f;
        bool multiFingerGripValid = false;
        std::uint32_t semanticFingerGroups = 0;
        std::uint32_t probeFingerGroups = 0;
        std::uint32_t combinedFingerGroups = 0;
        std::uint32_t minimumFingerGroups = 3;
    };

    struct GrabContactEvidenceDecision
    {
        bool accept = false;
        bool useMultiFingerPivot = false;
        bool strictMultiFingerRequired = false;
        GrabContactEvidenceLevel level = GrabContactEvidenceLevel::Rejected;
        const char* reason = "noContactEvidence";
    };

    inline GrabContactQualityMode sanitizeQualityMode(int value)
    {
        if (value <= static_cast<int>(GrabContactQualityMode::LegacyPermissive)) {
            return GrabContactQualityMode::LegacyPermissive;
        }
        if (value >= static_cast<int>(GrabContactQualityMode::StrictMultiFinger)) {
            return GrabContactQualityMode::StrictMultiFinger;
        }
        return GrabContactQualityMode::HybridEvidence;
    }

    inline std::uint32_t sanitizeMinimumFingerGroups(std::uint32_t value)
    {
        return std::clamp(value, 1u, 5u);
    }

    inline std::uint32_t combinedFingerGroupCount(const GrabContactEvidenceInput& input)
    {
        if (input.combinedFingerGroups > 0) {
            return (std::min)(input.combinedFingerGroups, 5u);
        }
        return (std::min)(input.semanticFingerGroups + input.probeFingerGroups, 5u);
    }

    inline const char* contactQualityModeName(GrabContactQualityMode mode)
    {
        switch (mode) {
        case GrabContactQualityMode::LegacyPermissive:
            return "legacyPermissive";
        case GrabContactQualityMode::HybridEvidence:
            return "hybridEvidence";
        case GrabContactQualityMode::StrictMultiFinger:
            return "strictMultiFinger";
        default:
            return "unknown";
        }
    }

    inline const char* contactEvidenceLevelName(GrabContactEvidenceLevel level)
    {
        switch (level) {
        case GrabContactEvidenceLevel::Rejected:
            return "rejected";
        case GrabContactEvidenceLevel::LegacyPermissive:
            return "legacyPermissive";
        case GrabContactEvidenceLevel::BaselinePatch:
            return "baselinePatch";
        case GrabContactEvidenceLevel::EnhancedFingerPatch:
            return "enhancedFingerPatch";
        case GrabContactEvidenceLevel::HighConfidenceFingerGrip:
            return "highConfidenceFingerGrip";
        default:
            return "unknown";
        }
    }

    inline GrabContactEvidenceDecision evaluateGrabContactEvidence(const GrabContactEvidenceInput& input)
    {
        GrabContactEvidenceDecision decision{};
        const auto mode = sanitizeQualityMode(input.qualityMode);
        const std::uint32_t minimumFingerGroups = sanitizeMinimumFingerGroups(input.minimumFingerGroups);
        const std::uint32_t totalFingerGroups = combinedFingerGroupCount(input);

        if (!input.multiFingerValidationEnabled || mode == GrabContactQualityMode::LegacyPermissive) {
            decision.accept = true;
            decision.level = GrabContactEvidenceLevel::LegacyPermissive;
            decision.reason = input.multiFingerValidationEnabled ? "legacyPermissive" : "multiFingerEvidenceDisabled";
            return decision;
        }

        if (mode == GrabContactQualityMode::StrictMultiFinger) {
            decision.strictMultiFingerRequired = true;
            if (input.multiFingerGripValid && totalFingerGroups >= minimumFingerGroups) {
                decision.accept = true;
                decision.useMultiFingerPivot = true;
                decision.level = GrabContactEvidenceLevel::HighConfidenceFingerGrip;
                decision.reason = "strictMultiFingerSatisfied";
            } else {
                decision.reason = "strictMultiFingerRequired";
            }
            return decision;
        }

        if (input.multiFingerGripValid && totalFingerGroups >= minimumFingerGroups) {
            decision.accept = true;
            decision.useMultiFingerPivot = true;
            decision.level = GrabContactEvidenceLevel::HighConfidenceFingerGrip;
            decision.reason = "highConfidenceFingerGrip";
            return decision;
        }

        if (input.contactPatchAccepted && input.contactPatchMeshSnapped) {
            decision.accept = input.contactPatchReliable || input.contactPatchConfidence >= 0.70f || totalFingerGroups > 0;
            if (decision.accept) {
                decision.level = totalFingerGroups > 0 ? GrabContactEvidenceLevel::EnhancedFingerPatch : GrabContactEvidenceLevel::BaselinePatch;
                decision.reason = totalFingerGroups > 0 ? "fingerEnhancedContactPatch" : "reliableContactPatch";
            } else {
                decision.reason = "weakPatchWithoutFingerEvidence";
            }
            return decision;
        }

        decision.reason = input.contactPatchAccepted ? "contactPatchNotMeshSnapped" : "noAcceptedContactPatch";
        return decision;
    }
}

// ---- GrabContactPatchMath.h ----

/*
 * ROCK needs the touched object point to be captured once and then held in the
 * object's own body space. HIGGS' dynamic grab path keeps one coherent
 * object-in-hand relationship after selection; this layer gives ROCK a richer
 * contact source by fitting a small palm-facing patch, while leaving Havok body
 * memory and FRIK runtime ownership outside the pure math.
 */

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace frik::rock::grab_contact_patch_math
{
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
    struct GrabContactPatchResult
    {
        Vector contactPoint{};
        Vector normal{};
        Vector tangent{};
        Vector bitangent{};
        std::size_t hitCount = 0;
        std::size_t rejectedSampleCount = 0;
        float confidence = 0.0f;
        float meshSnapDeltaGameUnits = 0.0f;
        bool valid = false;
        bool orientationReliable = false;
        const char* fallbackReason = "uninitialized";
    };

    enum class GrabContactPatchPivotSource
    {
        None,
        SelectedHit,
        MeshSnap,
        PatchSample
    };

    template <class Vector>
    struct GrabContactPatchPivotDecision
    {
        Vector point{};
        GrabContactPatchPivotSource source = GrabContactPatchPivotSource::None;
        bool valid = false;
        bool replaceSelectedPoint = false;
        float selectionDeltaGameUnits = (std::numeric_limits<float>::max)();
        const char* reason = "uninitialized";
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
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
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

    template <class Vector>
    inline Vector orientNormalTowardPalm(const Vector& normal, const Vector& palmNormal)
    {
        Vector oriented = normalizeOrZero(normal);
        const Vector palm = normalizeOrZero(palmNormal);
        if (lengthSquared(oriented) <= 0.0f || lengthSquared(palm) <= 0.0f) {
            return oriented;
        }
        if (dot(oriented, palm) > 0.0f) {
            oriented = negate(oriented);
        }
        return oriented;
    }

    inline const char* pivotSourceName(GrabContactPatchPivotSource source)
    {
        switch (source) {
        case GrabContactPatchPivotSource::SelectedHit:
            return "selectedHit";
        case GrabContactPatchPivotSource::MeshSnap:
            return "meshSnap";
        case GrabContactPatchPivotSource::PatchSample:
            return "patchSample";
        default:
            return "none";
        }
    }

    template <class Vector>
    inline Vector choosePrincipalPatchTangent(const std::vector<Vector>& points, const Vector& centroid, const Vector& normal, const Vector& tangentHint)
    {
        Vector axisT = normalizeOrZero(projectOntoPlane(tangentHint, normal));
        if (lengthSquared(axisT) <= 0.0f) {
            axisT = stablePerpendicular(normal);
        }
        Vector axisB = normalizeOrZero(cross(normal, axisT));
        if (lengthSquared(axisB) <= 0.0f) {
            return axisT;
        }

        float uu = 0.0f;
        float uv = 0.0f;
        float vv = 0.0f;
        for (const auto& point : points) {
            const Vector delta = sub(point, centroid);
            const float u = dot(delta, axisT);
            const float v = dot(delta, axisB);
            uu += u * u;
            uv += u * v;
            vv += v * v;
        }

        if (uu + vv <= 1.0e-6f) {
            return axisT;
        }

        const float angle = 0.5f * std::atan2(2.0f * uv, uu - vv);
        Vector tangent = add(mul(axisT, std::cos(angle)), mul(axisB, std::sin(angle)));
        tangent = normalizeOrZero(tangent);
        if (lengthSquared(tangent) <= 0.0f) {
            tangent = axisT;
        }
        return tangent;
    }

    template <class Vector>
    inline Vector clampPointToPatchSpan(const Vector& point, const std::vector<Vector>& samples, const Vector& centroid, const Vector& tangent, const Vector& bitangent)
    {
        if (samples.size() < 2 || lengthSquared(tangent) <= 0.0f || lengthSquared(bitangent) <= 0.0f) {
            return point;
        }

        float minT = (std::numeric_limits<float>::max)();
        float maxT = -(std::numeric_limits<float>::max)();
        float minB = (std::numeric_limits<float>::max)();
        float maxB = -(std::numeric_limits<float>::max)();
        for (const auto& sample : samples) {
            const Vector delta = sub(sample, centroid);
            const float t = dot(delta, tangent);
            const float b = dot(delta, bitangent);
            minT = (std::min)(minT, t);
            maxT = (std::max)(maxT, t);
            minB = (std::min)(minB, b);
            maxB = (std::max)(maxB, b);
        }

        const Vector delta = sub(point, centroid);
        const float clampedT = std::clamp(dot(delta, tangent), minT, maxT);
        const float clampedB = std::clamp(dot(delta, bitangent), minB, maxB);
        return add(centroid, add(mul(tangent, clampedT), mul(bitangent, clampedB)));
    }

    template <class Vector>
    inline GrabContactPatchResult<Vector> fitContactPatch(const std::vector<GrabContactPatchSample<Vector>>& samples,
        const Vector& palmPoint,
        const Vector& palmNormal,
        const Vector& tangentHint,
        float maxNormalAngleDegrees)
    {
        GrabContactPatchResult<Vector> result{};
        result.fallbackReason = "noAcceptedHits";

        std::vector<Vector> points;
        std::vector<Vector> normals;
        points.reserve(samples.size());
        normals.reserve(samples.size());

        for (const auto& sample : samples) {
            if (!sample.accepted) {
                ++result.rejectedSampleCount;
                continue;
            }

            const Vector orientedNormal = orientNormalTowardPalm(sample.normal, palmNormal);
            if (lengthSquared(orientedNormal) <= 0.0f) {
                ++result.rejectedSampleCount;
                continue;
            }

            points.push_back(sample.point);
            normals.push_back(orientedNormal);
        }

        if (points.empty()) {
            return result;
        }

        Vector averageNormal{};
        for (const auto& normal : normals) {
            averageNormal = add(averageNormal, normal);
        }
        averageNormal = normalizeOrZero(averageNormal);
        if (lengthSquared(averageNormal) <= 0.0f) {
            averageNormal = normals.front();
        }

        const float clampedAngle = std::clamp(maxNormalAngleDegrees, 0.0f, 179.0f);
        const float minNormalDot = std::cos(clampedAngle * 3.14159265358979323846f / 180.0f);
        std::vector<Vector> coherentPoints;
        std::vector<Vector> coherentNormals;
        coherentPoints.reserve(points.size());
        coherentNormals.reserve(normals.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (dot(normals[i], averageNormal) >= minNormalDot) {
                coherentPoints.push_back(points[i]);
                coherentNormals.push_back(normals[i]);
            } else {
                ++result.rejectedSampleCount;
            }
        }

        if (coherentPoints.empty()) {
            return result;
        }

        Vector centroid{};
        for (const auto& point : coherentPoints) {
            centroid = add(centroid, point);
        }
        centroid = mul(centroid, 1.0f / static_cast<float>(coherentPoints.size()));

        Vector normal{};
        for (const auto& coherentNormal : coherentNormals) {
            normal = add(normal, coherentNormal);
        }
        normal = normalizeOrZero(normal);
        if (lengthSquared(normal) <= 0.0f) {
            normal = averageNormal;
        }

        if (coherentPoints.size() >= 3) {
            Vector tripletNormalSum = normal;
            for (std::size_t i = 0; i < coherentPoints.size(); ++i) {
                for (std::size_t j = i + 1; j < coherentPoints.size(); ++j) {
                    for (std::size_t k = j + 1; k < coherentPoints.size(); ++k) {
                        Vector tripletNormal = cross(sub(coherentPoints[j], coherentPoints[i]), sub(coherentPoints[k], coherentPoints[i]));
                        const float area = length(tripletNormal);
                        if (area <= 1.0e-5f) {
                            continue;
                        }
                        tripletNormal = mul(tripletNormal, 1.0f / area);
                        if (dot(tripletNormal, normal) < 0.0f) {
                            tripletNormal = negate(tripletNormal);
                        }
                        tripletNormalSum = add(tripletNormalSum, mul(tripletNormal, area));
                    }
                }
            }
            const Vector fittedNormal = normalizeOrZero(tripletNormalSum);
            if (lengthSquared(fittedNormal) > 0.0f) {
                normal = fittedNormal;
            }
        }

        Vector tangent = choosePrincipalPatchTangent(coherentPoints, centroid, normal, tangentHint);
        Vector bitangent = normalizeOrZero(cross(normal, tangent));
        if (lengthSquared(bitangent) <= 0.0f) {
            tangent = stablePerpendicular(normal);
            bitangent = normalizeOrZero(cross(normal, tangent));
        }

        const Vector projectedPalm = sub(palmPoint, mul(normal, dot(sub(palmPoint, centroid), normal)));
        result.contactPoint = clampPointToPatchSpan(projectedPalm, coherentPoints, centroid, tangent, bitangent);
        result.normal = normal;
        result.tangent = tangent;
        result.bitangent = bitangent;
        result.hitCount = coherentPoints.size();
        result.valid = true;

        if (coherentPoints.size() >= 3) {
            result.confidence = 1.0f;
            result.orientationReliable = true;
            result.fallbackReason = "none";
        } else if (coherentPoints.size() == 2) {
            result.confidence = 0.70f;
            result.orientationReliable = lengthSquared(tangent) > 0.0f && lengthSquared(bitangent) > 0.0f;
            result.fallbackReason = result.orientationReliable ? "twoHitPatch" : "twoHitNoTangent";
        } else {
            result.confidence = 0.35f;
            result.orientationReliable = false;
            result.contactPoint = coherentPoints.front();
            result.fallbackReason = "singleHitPivotOnly";
        }

        return result;
    }

    template <class Vector>
    inline bool contactPatchNormalMatchesSelection(const GrabContactPatchResult<Vector>& patch,
        const Vector& selectionNormal,
        bool hasSelectionNormal,
        const Vector& palmNormal,
        float maxNormalAngleDegrees)
    {
        if (!hasSelectionNormal) {
            return true;
        }
        if (!patch.valid) {
            return false;
        }

        const Vector patchNormal = orientNormalTowardPalm(patch.normal, palmNormal);
        const Vector selectedNormal = orientNormalTowardPalm(selectionNormal, palmNormal);
        if (lengthSquared(patchNormal) <= 0.0f || lengthSquared(selectedNormal) <= 0.0f) {
            return false;
        }

        const float clampedAngle = std::clamp(maxNormalAngleDegrees, 0.0f, 179.0f);
        const float minNormalDot = std::cos(clampedAngle * 3.14159265358979323846f / 180.0f);
        return dot(patchNormal, selectedNormal) >= minNormalDot;
    }

    template <class Vector>
    inline GrabContactPatchPivotDecision<Vector> chooseContactPatchPivotPoint(const GrabContactPatchResult<Vector>& patch,
        const std::vector<GrabContactPatchSample<Vector>>& samples,
        const Vector& selectedHitPoint,
        bool hasSelectedHitPoint,
        const Vector& meshSnapPoint,
        bool hasMeshSnapPoint,
        float maxSelectionDeltaGameUnits,
        float maxSampleNormalAngleDegrees = 45.0f)
    {
        GrabContactPatchPivotDecision<Vector> decision{};

        auto useSelected = [&](const char* reason) {
            decision = {};
            decision.reason = reason;
            if (hasSelectedHitPoint) {
                decision.point = selectedHitPoint;
                decision.source = GrabContactPatchPivotSource::SelectedHit;
                decision.valid = true;
                decision.replaceSelectedPoint = false;
                decision.selectionDeltaGameUnits = 0.0f;
            }
            return decision;
        };

        if (!patch.valid) {
            return useSelected("patchInvalid");
        }

        const auto selectionDeltaAllowed = [&](const Vector& point, float& outDelta) {
            outDelta = hasSelectedHitPoint ? length(sub(point, selectedHitPoint)) : 0.0f;
            return !hasSelectedHitPoint || maxSelectionDeltaGameUnits < 0.0f || outDelta <= maxSelectionDeltaGameUnits;
        };

        if (hasMeshSnapPoint) {
            float selectionDelta = 0.0f;
            if (!selectionDeltaAllowed(meshSnapPoint, selectionDelta)) {
                return useSelected("meshSnapTooFarFromSelection");
            }

            decision.point = meshSnapPoint;
            decision.source = GrabContactPatchPivotSource::MeshSnap;
            decision.valid = true;
            decision.replaceSelectedPoint = true;
            decision.selectionDeltaGameUnits = selectionDelta;
            decision.reason = "meshSnap";
            return decision;
        }

        Vector nearestSample{};
        bool hasNearestSample = false;
        float bestDistanceSq = (std::numeric_limits<float>::max)();
        const float clampedAngle = std::clamp(maxSampleNormalAngleDegrees, 0.0f, 179.0f);
        const float minNormalDot = std::cos(clampedAngle * 3.14159265358979323846f / 180.0f);
        const Vector patchNormal = normalizeOrZero(patch.normal);
        for (const auto& sample : samples) {
            if (!sample.accepted) {
                continue;
            }
            const Vector sampleNormal = normalizeOrZero(sample.normal);
            if (lengthSquared(patchNormal) > 0.0f && lengthSquared(sampleNormal) > 0.0f &&
                std::fabs(dot(sampleNormal, patchNormal)) < minNormalDot) {
                continue;
            }

            const float distSq = lengthSquared(sub(sample.point, patch.contactPoint));
            if (distSq < bestDistanceSq) {
                bestDistanceSq = distSq;
                nearestSample = sample.point;
                hasNearestSample = true;
            }
        }

        if (patch.hitCount >= 2 && hasNearestSample) {
            float selectionDelta = 0.0f;
            if (!selectionDeltaAllowed(nearestSample, selectionDelta)) {
                return useSelected("patchSampleTooFarFromSelection");
            }

            decision.point = nearestSample;
            decision.source = GrabContactPatchPivotSource::PatchSample;
            decision.valid = true;
            decision.replaceSelectedPoint = true;
            decision.selectionDeltaGameUnits = selectionDelta;
            decision.reason = "nearestPatchSample";
            return decision;
        }

        return useSelected(patch.hitCount < 2 ? "insufficientPatchHitsForPivot" : "noValidatedPatchSample");
    }

    template <class Transform, class Vector>
    inline Vector freezePivotBBodyLocal(const Transform& bodyWorld, const Vector& surfacePointWorld)
    {
        return transform_math::worldPointToLocal(bodyWorld, surfacePointWorld);
    }
}

// ---- GrabSurfaceFrameMath.h ----

/*
 * Palm-face grab alignment is intentionally solved as pure transform math.
 * HIGGS uses graphics triangles to choose coherent contact/body ownership, then
 * keeps a captured object-in-hand frame. ROCK extends that by optionally rotating
 * the touched surface into a palm-facing frame, but the solver must stay separate
 * from Havok body memory and FRIK runtime state so it can be tested and reused.
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include "physics-interaction/TransformMath.h"

namespace frik::rock::grab_surface_frame_math
{
    enum class GrabOrientationMode
    {
        PreserveObjectRotation = 0,
        SurfaceNormalAuto = 1,
        AuthoredOnly = 2
    };

    enum class GrabSurfaceFaceKind
    {
        Side = 0,
        CapTopBottom = 1,
        NarrowEdge = 2,
        Ambiguous = 3
    };

    enum class GrabSurfaceTangentSource
    {
        TriangleLongestEdge = 0,
        PreservedObjectRoll = 1,
        AuthoredFrame = 2,
        ObjectLongAxis = 3,
        Fallback = 4,
        ContactPatchPrincipal = 5
    };

    enum class GrabSurfaceAlignmentDecision
    {
        Accepted = 0,
        RejectedMode = 1,
        RejectedMissingSelectionHit = 2,
        RejectedOwnerMismatch = 3,
        RejectedPivotDistance = 4,
        RejectedSelectionDistance = 5,
        RejectedLowConfidence = 6,
        RejectedAmbiguousTangent = 7
    };

    template <class Vector>
    struct GrabSurfaceFrame
    {
        Vector normal{};
        Vector tangent{};
        Vector bitangent{};
        GrabSurfaceFaceKind faceKind{ GrabSurfaceFaceKind::Ambiguous };
        GrabSurfaceTangentSource tangentSource{ GrabSurfaceTangentSource::Fallback };
        float confidence = 0.0f;
        const char* fallbackReason = "uninitialized";

        bool usable() const { return confidence > 0.0f && faceKind != GrabSurfaceFaceKind::Ambiguous; }
    };

    template <class Transform>
    struct DesiredObjectFrame
    {
        Transform transform{};
        GrabOrientationMode modeUsed{ GrabOrientationMode::PreserveObjectRotation };
        GrabSurfaceFaceKind faceKind{ GrabSurfaceFaceKind::Ambiguous };
        GrabSurfaceTangentSource tangentSource{ GrabSurfaceTangentSource::Fallback };
        GrabSurfaceAlignmentDecision alignmentDecision{ GrabSurfaceAlignmentDecision::RejectedMode };
        float confidence = 0.0f;
        const char* fallbackReason = "uninitialized";
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
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
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

    template <class Vector>
    inline Vector chooseObjectDerivedTangentHint(const Vector& normal, const Vector& objectLongAxis)
    {
        Vector tangent = normalizeOrZero(projectOntoPlane(objectLongAxis, normal));
        if (lengthSquared(tangent) > 0.0f) {
            return tangent;
        }

        const Vector axes[3] = {
            Vector{ 1.0f, 0.0f, 0.0f },
            Vector{ 0.0f, 1.0f, 0.0f },
            Vector{ 0.0f, 0.0f, 1.0f },
        };
        const Vector n = normalizeOrZero(normal);
        int best = 0;
        float bestDot = (std::numeric_limits<float>::max)();
        for (int i = 0; i < 3; ++i) {
            const float axisDot = std::fabs(dot(axes[i], n));
            if (axisDot < bestDot) {
                bestDot = axisDot;
                best = i;
            }
        }
        return normalizeOrZero(projectOntoPlane(axes[best], normal));
    }

    template <class Vector>
    inline Vector chooseLongestEdgeTangent(
        const Vector& v0,
        const Vector& v1,
        const Vector& v2,
        const Vector& normal,
        float& outLongest,
        float& outSecondLongest)
    {
        const Vector edges[3] = { sub(v1, v0), sub(v2, v1), sub(v0, v2) };
        float lengths[3] = { lengthSquared(edges[0]), lengthSquared(edges[1]), lengthSquared(edges[2]) };
        int best = 0;
        int second = 1;
        if (lengths[1] > lengths[best]) {
            best = 1;
            second = 0;
        }
        if (lengths[2] > lengths[best]) {
            second = best;
            best = 2;
        } else if (lengths[2] > lengths[second]) {
            second = 2;
        }

        outLongest = std::sqrt((std::max)(0.0f, lengths[best]));
        outSecondLongest = std::sqrt((std::max)(0.0f, lengths[second]));
        return normalizeOrZero(projectOntoPlane(edges[best], normal));
    }

    template <class Vector>
    inline GrabSurfaceFrame<Vector> buildSurfaceFrameFromTriangle(const Vector& v0,
        const Vector& v1,
        const Vector& v2,
        const Vector& preservedTangent,
        const Vector& objectLongAxis,
        float capNormalDotThreshold = 0.85f,
        bool preserveRollForCaps = false)
    {
        GrabSurfaceFrame<Vector> result{};
        const Vector normal = normalizeOrZero(cross(sub(v1, v0), sub(v2, v1)));
        if (lengthSquared(normal) <= 0.0f) {
            result.normal = normalizeOrZero(objectLongAxis);
            result.tangent = stablePerpendicular(result.normal);
            result.bitangent = normalizeOrZero(cross(result.normal, result.tangent));
            result.faceKind = GrabSurfaceFaceKind::Ambiguous;
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.0f;
            result.fallbackReason = "degenerateTriangle";
            return result;
        }

        result.normal = normal;
        const Vector longAxis = normalizeOrZero(objectLongAxis);
        const bool capFace = lengthSquared(longAxis) > 0.0f && std::fabs(dot(normal, longAxis)) >= capNormalDotThreshold;
        result.faceKind = capFace ? GrabSurfaceFaceKind::CapTopBottom : GrabSurfaceFaceKind::Side;

        Vector tangent{};
        if (capFace && preserveRollForCaps) {
            tangent = normalizeOrZero(projectOntoPlane(preservedTangent, normal));
            result.tangentSource = GrabSurfaceTangentSource::PreservedObjectRoll;
            result.confidence = 0.50f;
        }

        if (!capFace && lengthSquared(longAxis) > 0.0f) {
            tangent = normalizeOrZero(projectOntoPlane(longAxis, normal));
            if (lengthSquared(tangent) > 0.0f) {
                result.tangentSource = GrabSurfaceTangentSource::ObjectLongAxis;
                result.confidence = 1.0f;
            }
        }

        if (lengthSquared(tangent) <= 0.0f) {
            float longest = 0.0f;
            float secondLongest = 0.0f;
            tangent = chooseLongestEdgeTangent(v0, v1, v2, normal, longest, secondLongest);
            result.tangentSource = GrabSurfaceTangentSource::TriangleLongestEdge;
            result.confidence = 1.0f;
            if (secondLongest > 0.0f && longest / secondLongest < 1.15f) {
                result.faceKind = capFace ? GrabSurfaceFaceKind::CapTopBottom : GrabSurfaceFaceKind::NarrowEdge;
                result.confidence = capFace ? 0.50f : 0.65f;
            }
        }

        if (lengthSquared(tangent) <= 0.0f) {
            tangent = normalizeOrZero(projectOntoPlane(preservedTangent, normal));
            result.tangentSource = GrabSurfaceTangentSource::PreservedObjectRoll;
            result.confidence = 0.35f;
        }
        if (lengthSquared(tangent) <= 0.0f) {
            tangent = stablePerpendicular(normal);
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.20f;
            result.fallbackReason = "noStableTriangleTangent";
        } else {
            result.fallbackReason = "none";
        }

        result.tangent = tangent;
        result.bitangent = normalizeOrZero(cross(result.normal, result.tangent));
        if (lengthSquared(result.bitangent) <= 0.0f) {
            result.faceKind = GrabSurfaceFaceKind::Ambiguous;
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.0f;
            result.fallbackReason = "invalidFrame";
        }
        return result;
    }

    template <class Vector>
    inline GrabSurfaceFrame<Vector> buildSurfaceFrameFromNormal(const Vector& surfaceNormal,
        const Vector& preservedTangent,
        const Vector& objectLongAxis,
        float capNormalDotThreshold = 0.85f,
        bool preserveRollForCaps = false)
    {
        GrabSurfaceFrame<Vector> result{};
        const Vector normal = normalizeOrZero(surfaceNormal);
        if (lengthSquared(normal) <= 0.0f) {
            result.faceKind = GrabSurfaceFaceKind::Ambiguous;
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.0f;
            result.fallbackReason = "invalidCollisionNormal";
            return result;
        }

        result.normal = normal;
        const Vector longAxis = normalizeOrZero(objectLongAxis);
        const bool capFace = lengthSquared(longAxis) > 0.0f && std::fabs(dot(normal, longAxis)) >= capNormalDotThreshold;
        result.faceKind = capFace ? GrabSurfaceFaceKind::CapTopBottom : GrabSurfaceFaceKind::Side;

        Vector tangent{};
        if (capFace && preserveRollForCaps) {
            tangent = normalizeOrZero(projectOntoPlane(preservedTangent, normal));
            result.tangentSource = GrabSurfaceTangentSource::PreservedObjectRoll;
            result.confidence = 0.50f;
        }

        if (!capFace && lengthSquared(longAxis) > 0.0f) {
            tangent = normalizeOrZero(projectOntoPlane(longAxis, normal));
            if (lengthSquared(tangent) > 0.0f) {
                result.tangentSource = GrabSurfaceTangentSource::ObjectLongAxis;
                result.confidence = 1.0f;
            }
        }

        if (lengthSquared(tangent) <= 0.0f) {
            tangent = normalizeOrZero(projectOntoPlane(preservedTangent, normal));
            result.tangentSource = GrabSurfaceTangentSource::PreservedObjectRoll;
            result.confidence = 0.65f;
        }
        if (lengthSquared(tangent) <= 0.0f) {
            tangent = stablePerpendicular(normal);
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.20f;
            result.fallbackReason = "noStableCollisionTangent";
        } else {
            result.fallbackReason = "none";
        }

        result.tangent = tangent;
        result.bitangent = normalizeOrZero(cross(result.normal, result.tangent));
        if (lengthSquared(result.bitangent) <= 0.0f) {
            result.faceKind = GrabSurfaceFaceKind::Ambiguous;
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.0f;
            result.fallbackReason = "invalidFrame";
        }
        return result;
    }

    template <class Vector>
    inline GrabSurfaceAlignmentDecision evaluateSurfaceAlignmentGate(const GrabSurfaceFrame<Vector>& localSurfaceFrame,
        GrabOrientationMode mode,
        bool hasSelectionHit,
        bool resolvedOwnerMatches,
        float pivotToSurfaceDistanceGameUnits,
        float selectionToMeshDistanceGameUnits,
        float maxPivotToSurfaceDistanceGameUnits,
        float maxSelectionToMeshDistanceGameUnits,
        bool requireResolvedOwnerMatch,
        float minConfidence = 0.20f)
    {
        if (mode != GrabOrientationMode::SurfaceNormalAuto) {
            return GrabSurfaceAlignmentDecision::RejectedMode;
        }
        if (!hasSelectionHit) {
            return GrabSurfaceAlignmentDecision::RejectedMissingSelectionHit;
        }
        if (requireResolvedOwnerMatch && !resolvedOwnerMatches) {
            return GrabSurfaceAlignmentDecision::RejectedOwnerMismatch;
        }
        if (maxPivotToSurfaceDistanceGameUnits >= 0.0f && pivotToSurfaceDistanceGameUnits > maxPivotToSurfaceDistanceGameUnits) {
            return GrabSurfaceAlignmentDecision::RejectedPivotDistance;
        }
        if (maxSelectionToMeshDistanceGameUnits >= 0.0f && selectionToMeshDistanceGameUnits > maxSelectionToMeshDistanceGameUnits) {
            return GrabSurfaceAlignmentDecision::RejectedSelectionDistance;
        }
        if (!localSurfaceFrame.usable() || localSurfaceFrame.confidence < minConfidence) {
            return GrabSurfaceAlignmentDecision::RejectedLowConfidence;
        }
        if (localSurfaceFrame.tangentSource == GrabSurfaceTangentSource::Fallback ||
            (localSurfaceFrame.faceKind != GrabSurfaceFaceKind::Side && localSurfaceFrame.faceKind != GrabSurfaceFaceKind::CapTopBottom)) {
            return GrabSurfaceAlignmentDecision::RejectedAmbiguousTangent;
        }
        return GrabSurfaceAlignmentDecision::Accepted;
    }

    template <class Matrix, class Vector>
    inline void setRows(Matrix& matrix, const Vector& row0, const Vector& row1, const Vector& row2)
    {
        matrix.entry[0][0] = row0.x;
        matrix.entry[0][1] = row0.y;
        matrix.entry[0][2] = row0.z;
        matrix.entry[0][3] = 0.0f;
        matrix.entry[1][0] = row1.x;
        matrix.entry[1][1] = row1.y;
        matrix.entry[1][2] = row1.z;
        matrix.entry[1][3] = 0.0f;
        matrix.entry[2][0] = row2.x;
        matrix.entry[2][1] = row2.y;
        matrix.entry[2][2] = row2.z;
        matrix.entry[2][3] = 0.0f;
    }

    template <class Matrix, class Vector>
    inline Matrix buildRotationMappingLocalFrameToWorldFrame(const GrabSurfaceFrame<Vector>& localFrame,
        const Vector& targetNormal,
        const Vector& targetTangent,
        const Vector& targetBitangent)
    {
        Matrix result{};
        const Vector row0 = add(add(mul(targetNormal, localFrame.normal.x), mul(targetTangent, localFrame.tangent.x)), mul(targetBitangent, localFrame.bitangent.x));
        const Vector row1 = add(add(mul(targetNormal, localFrame.normal.y), mul(targetTangent, localFrame.tangent.y)), mul(targetBitangent, localFrame.bitangent.y));
        const Vector row2 = add(add(mul(targetNormal, localFrame.normal.z), mul(targetTangent, localFrame.tangent.z)), mul(targetBitangent, localFrame.bitangent.z));
        setRows(result, row0, row1, row2);
        return result;
    }

    template <class Transform, class Vector>
    inline Transform translateObjectToPutLocalPointAtWorldPoint(Transform objectWorld, const Vector& localPoint, const Vector& worldPoint)
    {
        const Vector localOffset = transform_math::localVectorToWorld(objectWorld, localPoint);
        objectWorld.translate = sub(worldPoint, localOffset);
        return objectWorld;
    }

    template <class Transform, class Vector>
    inline DesiredObjectFrame<Transform> buildDesiredObjectWorldFromSurfaceFrame(const Transform& objectWorld,
        const Vector& localSurfacePoint,
        const GrabSurfaceFrame<Vector>& localSurfaceFrame,
        const Vector& grabPivotWorld,
        const Vector& palmNormalWorld,
        const Vector& palmTangentWorld,
        GrabOrientationMode mode,
        float minConfidence = 0.20f)
    {
        DesiredObjectFrame<Transform> result{};
        result.transform = translateObjectToPutLocalPointAtWorldPoint(objectWorld, localSurfacePoint, grabPivotWorld);
        result.modeUsed = mode == GrabOrientationMode::AuthoredOnly ? GrabOrientationMode::AuthoredOnly : GrabOrientationMode::PreserveObjectRotation;
        result.faceKind = localSurfaceFrame.faceKind;
        result.tangentSource = localSurfaceFrame.tangentSource;
        result.confidence = localSurfaceFrame.confidence;
        result.fallbackReason = "preserveObjectRotation";
        result.alignmentDecision = mode == GrabOrientationMode::SurfaceNormalAuto ? GrabSurfaceAlignmentDecision::Accepted : GrabSurfaceAlignmentDecision::RejectedMode;

        if (mode != GrabOrientationMode::SurfaceNormalAuto) {
            result.alignmentDecision = GrabSurfaceAlignmentDecision::RejectedMode;
            return result;
        }
        if (!localSurfaceFrame.usable() || localSurfaceFrame.confidence < minConfidence) {
            result.modeUsed = GrabOrientationMode::PreserveObjectRotation;
            result.alignmentDecision = GrabSurfaceAlignmentDecision::RejectedLowConfidence;
            result.fallbackReason = localSurfaceFrame.fallbackReason ? localSurfaceFrame.fallbackReason : "surfaceFrameUnusable";
            return result;
        }

        const Vector targetNormal = normalizeOrZero(negate(palmNormalWorld));
        if (lengthSquared(targetNormal) <= 0.0f) {
            result.modeUsed = GrabOrientationMode::PreserveObjectRotation;
            result.alignmentDecision = GrabSurfaceAlignmentDecision::RejectedLowConfidence;
            result.fallbackReason = "invalidPalmNormal";
            return result;
        }

        Vector targetTangent{};
        if (localSurfaceFrame.faceKind == GrabSurfaceFaceKind::CapTopBottom && localSurfaceFrame.tangentSource == GrabSurfaceTangentSource::PreservedObjectRoll) {
            const Vector preservedWorldTangent = transform_math::rotateLocalVectorToWorld(objectWorld.rotate, localSurfaceFrame.tangent);
            targetTangent = normalizeOrZero(projectOntoPlane(preservedWorldTangent, targetNormal));
        }
        if (lengthSquared(targetTangent) <= 0.0f) {
            targetTangent = normalizeOrZero(projectOntoPlane(palmTangentWorld, targetNormal));
        }
        if (lengthSquared(targetTangent) <= 0.0f) {
            targetTangent = stablePerpendicular(targetNormal);
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
        }

        const Vector targetBitangent = normalizeOrZero(cross(targetNormal, targetTangent));
        if (lengthSquared(targetBitangent) <= 0.0f) {
            result.modeUsed = GrabOrientationMode::PreserveObjectRotation;
            result.alignmentDecision = GrabSurfaceAlignmentDecision::RejectedLowConfidence;
            result.fallbackReason = "invalidPalmFrame";
            return result;
        }

        result.transform = objectWorld;
        result.transform.rotate = buildRotationMappingLocalFrameToWorldFrame<decltype(objectWorld.rotate), Vector>(
            localSurfaceFrame, targetNormal, targetTangent, targetBitangent);
        result.transform = translateObjectToPutLocalPointAtWorldPoint(result.transform, localSurfacePoint, grabPivotWorld);
        result.modeUsed = GrabOrientationMode::SurfaceNormalAuto;
        result.alignmentDecision = GrabSurfaceAlignmentDecision::Accepted;
        result.fallbackReason = "none";
        return result;
    }
}

// ---- GrabOppositionFrameMath.h ----

/*
 * ROCK's opposition grab frame uses hand/finger colliders as semantic contact
 * evidence without letting raw collider collision fight the held-object
 * constraint. HIGGS captures one coherent object-in-hand frame and computes
 * finger curves from mesh geometry; this layer adds ROCK's bone-derived
 * thumb-versus-finger convention before that frame is captured, so the object
 * is seated between the live thumb and opposing finger when both contacts are
 * trustworthy.
 */

#include "physics-interaction/TransformMath.h"

#include <cmath>

namespace frik::rock::grab_opposition_frame_math
{
    template <class Transform, class Vector>
    struct OppositionFrameInput
    {
        bool enabled = false;
        Transform objectWorld{};
        Vector thumbObjectLocal{};
        Vector opposingObjectLocal{};
        Vector thumbHandWorld{};
        Vector opposingHandWorld{};
        Vector objectRollAxisLocal{};
        Vector handRollAxisWorld{};
    };

    template <class Transform, class Vector>
    struct OppositionFrameResult
    {
        bool valid = false;
        Transform desiredObjectWorld{};
        Vector pivotWorld{};
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
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x);
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

    template <class Vector>
    inline Vector projectOntoPlane(const Vector& value, const Vector& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    template <class Vector>
    inline Vector stablePerpendicular(const Vector& normal)
    {
        const Vector xAxis = makeVector<Vector>(1.0f, 0.0f, 0.0f);
        const Vector yAxis = makeVector<Vector>(0.0f, 1.0f, 0.0f);
        const Vector zAxis = makeVector<Vector>(0.0f, 0.0f, 1.0f);
        const Vector hint = std::fabs(dot(normal, xAxis)) < 0.85f ? xAxis : (std::fabs(dot(normal, yAxis)) < 0.85f ? yAxis : zAxis);
        return normalizeOrZero(projectOntoPlane(hint, normal));
    }

    template <class Matrix, class Vector>
    inline void setRows(Matrix& matrix, const Vector& row0, const Vector& row1, const Vector& row2)
    {
        matrix.entry[0][0] = row0.x;
        matrix.entry[0][1] = row0.y;
        matrix.entry[0][2] = row0.z;
        matrix.entry[0][3] = 0.0f;
        matrix.entry[1][0] = row1.x;
        matrix.entry[1][1] = row1.y;
        matrix.entry[1][2] = row1.z;
        matrix.entry[1][3] = 0.0f;
        matrix.entry[2][0] = row2.x;
        matrix.entry[2][1] = row2.y;
        matrix.entry[2][2] = row2.z;
        matrix.entry[2][3] = 0.0f;
    }

    template <class Matrix, class Vector>
    inline Matrix buildRotationFromBasisMap(const Vector& localOpposition,
        const Vector& localRoll,
        const Vector& localPalm,
        const Vector& targetOpposition,
        const Vector& targetRoll,
        const Vector& targetPalm)
    {
        Matrix result{};
        const Vector row0 = add(add(mul(targetOpposition, localOpposition.x), mul(targetRoll, localRoll.x)), mul(targetPalm, localPalm.x));
        const Vector row1 = add(add(mul(targetOpposition, localOpposition.y), mul(targetRoll, localRoll.y)), mul(targetPalm, localPalm.y));
        const Vector row2 = add(add(mul(targetOpposition, localOpposition.z), mul(targetRoll, localRoll.z)), mul(targetPalm, localPalm.z));
        setRows(result, row0, row1, row2);
        return result;
    }

    template <class Transform, class Vector>
    inline OppositionFrameResult<Transform, Vector> buildOppositionDesiredObjectWorld(const OppositionFrameInput<Transform, Vector>& input)
    {
        OppositionFrameResult<Transform, Vector> result{};
        result.desiredObjectWorld = input.objectWorld;
        result.pivotWorld = mul(add(input.thumbHandWorld, input.opposingHandWorld), 0.5f);
        if (!input.enabled) {
            result.reason = "disabled";
            return result;
        }

        const Vector localOpposition = normalizeOrZero(sub(input.opposingObjectLocal, input.thumbObjectLocal));
        const Vector targetOpposition = normalizeOrZero(sub(input.opposingHandWorld, input.thumbHandWorld));
        if (lengthSquared(targetOpposition) <= 0.0f) {
            result.reason = "degenerateHandSpan";
            return result;
        }
        if (lengthSquared(localOpposition) <= 0.0f) {
            result.reason = "degenerateObjectSpan";
            return result;
        }

        Vector localRoll = normalizeOrZero(projectOntoPlane(input.objectRollAxisLocal, localOpposition));
        if (lengthSquared(localRoll) <= 0.0f) {
            localRoll = stablePerpendicular(localOpposition);
        }
        Vector targetRoll = normalizeOrZero(projectOntoPlane(input.handRollAxisWorld, targetOpposition));
        if (lengthSquared(targetRoll) <= 0.0f) {
            targetRoll = stablePerpendicular(targetOpposition);
        }

        const Vector localPalm = normalizeOrZero(cross(localOpposition, localRoll));
        const Vector targetPalm = normalizeOrZero(cross(targetOpposition, targetRoll));
        if (lengthSquared(localPalm) <= 0.0f) {
            result.reason = "invalidObjectBasis";
            return result;
        }
        if (lengthSquared(targetPalm) <= 0.0f) {
            result.reason = "invalidHandBasis";
            return result;
        }

        result.desiredObjectWorld = input.objectWorld;
        result.desiredObjectWorld.rotate = buildRotationFromBasisMap<decltype(input.objectWorld.rotate), Vector>(
            localOpposition,
            localRoll,
            localPalm,
            targetOpposition,
            targetRoll,
            targetPalm);
        result.desiredObjectWorld.translate = sub(result.pivotWorld,
            transform_math::localVectorToWorld(result.desiredObjectWorld, mul(add(input.thumbObjectLocal, input.opposingObjectLocal), 0.5f)));
        result.valid = true;
        result.reason = "thumbOpposition";
        return result;
    }
}

// ---- GrabMultiFingerContactMath.h ----

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

#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/TransformMath.h"

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
