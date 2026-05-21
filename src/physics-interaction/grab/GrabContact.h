#pragma once

/*
 * Grab contact policy is grouped here because contact source, evidence, patch, surface, opposition, and multi-finger contact math are one contact interpretation pipeline.
 */


// ---- GrabContactSourcePolicy.h ----

/*
 * Mesh-authoritative grab contact keeps ROCK's dynamic object pipeline aligned
 * with rendered geometry: physics collision identifies the object and body,
 * while the rendered mesh supplies the grabbed point and frame. FO4VR collision
 * bubbles can sit away from the visible surface, so this policy prevents raw
 * hknp hit points from silently becoming pivot B when mesh-only contact is
 * enabled. Contact-patch casts are still allowed to probe the palm region, but
 * mesh-only grabs may use that patch only after it snaps back to the resolved
 * object's rendered mesh.
 */

namespace rock::grab_contact_source_policy
{
    struct GrabContactSourcePolicy
    {
        bool allowCollisionGrabPoint = true;
        bool allowContactPatchPivot = true;
        bool requireContactPatchMeshSnap = false;
        bool requireMeshSurface = false;
        bool failWithoutMesh = false;
        const char* reason = "compatContactSources";
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
            policy.reason = "compatContactSources";
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
 * stream is not guaranteed to contain three fresh semantic contacts on the exact
 * frame the grab button is pressed. ROCK treats finger geometry as a
 * pose/contact-quality layer around a stable object-in-hand transform, keeping
 * reliable mesh contact as the baseline and upgrading the grab when
 * finger/thumb evidence is available instead of letting missing contact events
 * reject otherwise valid grabs.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rock::grab_contact_evidence_policy
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
        BaselineMeshSurface,
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
        bool contactPatchNormalTrusted = false;
        bool contactPatchPositionOnly = false;
        float contactPatchConfidence = 0.0f;
        bool meshSurfacePivotAccepted = false;
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
            return "permissiveFallback";
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
            return "permissiveFallback";
        case GrabContactEvidenceLevel::BaselineMeshSurface:
            return "baselineMeshSurface";
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
            decision.reason = input.multiFingerValidationEnabled ? "permissiveFallback" : "multiFingerEvidenceDisabled";
            return decision;
        }

        if (mode == GrabContactQualityMode::StrictMultiFinger) {
            decision.strictMultiFingerRequired = true;
            const bool hasObjectPivotAuthority =
                input.meshSurfacePivotAccepted || (input.contactPatchAccepted && input.contactPatchMeshSnapped);
            if (!hasObjectPivotAuthority) {
                decision.reason = "strictObjectPivotAuthorityRequired";
            } else if (input.multiFingerGripValid && totalFingerGroups >= minimumFingerGroups) {
                decision.accept = true;
                decision.level = GrabContactEvidenceLevel::HighConfidenceFingerGrip;
                decision.reason = "strictMultiFingerSatisfied";
            } else {
                decision.reason = "strictMultiFingerRequired";
            }
            return decision;
        }

        if (input.meshSurfacePivotAccepted) {
            /*
             * Patch position and patch orientation are different contracts.
             * A position-only patch can keep the grip point out of a palm-ray
             * fallback, but it must not promote itself as reliable normal
             * evidence for pose/held authority.
             */
            const bool patchHasTrustedOrientation =
                input.contactPatchReliable ||
                (!input.contactPatchPositionOnly &&
                    input.contactPatchNormalTrusted &&
                    input.contactPatchConfidence >= 0.70f);
            decision.accept = true;
            if (input.multiFingerGripValid && totalFingerGroups >= minimumFingerGroups) {
                decision.level = GrabContactEvidenceLevel::HighConfidenceFingerGrip;
                decision.reason = "meshSurfaceFingerGrip";
            } else if (input.contactPatchAccepted && input.contactPatchMeshSnapped &&
                       patchHasTrustedOrientation) {
                decision.level = GrabContactEvidenceLevel::BaselinePatch;
                decision.reason = "meshSurfaceContactPatch";
            } else {
                decision.level = GrabContactEvidenceLevel::BaselineMeshSurface;
                decision.reason = "meshSurfacePivot";
            }
            return decision;
        }

        if (input.contactPatchAccepted && input.contactPatchMeshSnapped) {
            const bool patchHasTrustedOrientation =
                input.contactPatchReliable ||
                (!input.contactPatchPositionOnly &&
                    input.contactPatchNormalTrusted &&
                    input.contactPatchConfidence >= 0.70f);
            const bool patchHasFingerSupport = totalFingerGroups > 0;
            decision.accept = patchHasTrustedOrientation || patchHasFingerSupport;
            if (decision.accept) {
                decision.level = totalFingerGroups > 0 ? GrabContactEvidenceLevel::EnhancedFingerPatch : GrabContactEvidenceLevel::BaselinePatch;
                decision.reason = totalFingerGroups > 0 ? "fingerEnhancedContactPatch" : "reliableContactPatch";
            } else if (input.contactPatchPositionOnly) {
                decision.reason = "positionOnlyPatchNeedsFingerOrSeat";
            } else {
                decision.reason = "weakPatchWithoutFingerEvidence";
            }
            return decision;
        }

        decision.reason = input.contactPatchAccepted ? "contactPatchNotMeshSnapped" : "noAcceptedContactPatch";
        return decision;
    }
}

// ---- GrabPivotAuthorityPolicy.h ----

/*
 * Contact-patch pivot authority is narrower than contact-patch evidence.
 * The patch may move the BODY-local pivot point only when it is mesh-snapped
 * to the resolved object, still coherent with selection, close to the palm
 * pocket, and materially better than the currently selected mesh pivot. It is
 * position-only authority; normals remain pose/evidence data unless another
 * policy explicitly promotes them.
 */

namespace rock::grab_pivot_authority_policy
{
    struct MeshBackedPatchPivotAuthorityInput
    {
        bool baselineValid = false;
        bool patchComparable = false;
        bool patchValid = false;
        bool patchMeshSnapped = false;
        float selectedPivotToPocketGameUnits = (std::numeric_limits<float>::max)();
        float patchPivotToPocketGameUnits = (std::numeric_limits<float>::max)();
        float selectedScore = (std::numeric_limits<float>::max)();
        float patchScore = (std::numeric_limits<float>::max)();
        float patchAuthorityDeltaGameUnits = (std::numeric_limits<float>::max)();
        float patchSelectionDeltaGameUnits = 0.0f;
        float probeSpacingGameUnits = 3.0f;
        float meshSnapMaxDistanceGameUnits = 6.0f;
        float alignmentMaxSelectionDeltaGameUnits = 8.0f;
    };

    struct MeshBackedPatchPivotAuthorityDecision
    {
        const char* reason = "notEvaluated";
        float baseAuthorityDeltaGameUnits = 0.0f;
        float extendedAuthorityDeltaGameUnits = 0.0f;
        float pocketImprovementGameUnits = 0.0f;
        float scoreImprovement = 0.0f;
        bool acceptPatchPivot = false;
        bool positionOnlyAuthority = true;
    };

    inline float finitePositiveOr(float value, float fallback)
    {
        return std::isfinite(value) && value > 0.0f ? value : fallback;
    }

    inline MeshBackedPatchPivotAuthorityDecision chooseMeshBackedPatchPivotAuthority(
        const MeshBackedPatchPivotAuthorityInput& input)
    {
        MeshBackedPatchPivotAuthorityDecision decision{};
        const float baseDelta = (std::max)(1.0f, finitePositiveOr(input.probeSpacingGameUnits, 3.0f));
        const float configuredExtendedDelta = (std::max)(
            baseDelta,
            (std::max)(
                finitePositiveOr(input.meshSnapMaxDistanceGameUnits, baseDelta) + baseDelta,
                finitePositiveOr(input.alignmentMaxSelectionDeltaGameUnits, baseDelta)));
        decision.baseAuthorityDeltaGameUnits = baseDelta;
        decision.extendedAuthorityDeltaGameUnits = (std::min)(configuredExtendedDelta, baseDelta * 4.0f);

        if (input.baselineValid && input.patchValid) {
            decision.pocketImprovementGameUnits =
                input.selectedPivotToPocketGameUnits - input.patchPivotToPocketGameUnits;
            decision.scoreImprovement = input.selectedScore - input.patchScore;
        }

        if (!input.baselineValid) {
            decision.reason = "invalidBaselinePivot";
            return decision;
        }
        if (!input.patchComparable || !input.patchValid) {
            decision.reason = input.patchValid ? "patchNotEligible" : "invalidPatchCandidate";
            return decision;
        }
        if (!input.patchMeshSnapped) {
            decision.reason = "patchNotMeshSnappedForAuthority";
            return decision;
        }
        if (!std::isfinite(input.patchAuthorityDeltaGameUnits) ||
            input.patchAuthorityDeltaGameUnits > decision.extendedAuthorityDeltaGameUnits) {
            decision.reason = "patchTooFarFromSelectedAuthority";
            return decision;
        }

        const float selectionDeltaLimit = finitePositiveOr(input.alignmentMaxSelectionDeltaGameUnits, baseDelta) + baseDelta;
        if (!std::isfinite(input.patchSelectionDeltaGameUnits) ||
            input.patchSelectionDeltaGameUnits > selectionDeltaLimit) {
            decision.reason = "patchSelectionNotCoherent";
            return decision;
        }

        const float maxPatchPocketDistance = decision.extendedAuthorityDeltaGameUnits + baseDelta;
        if (!std::isfinite(input.patchPivotToPocketGameUnits) ||
            input.patchPivotToPocketGameUnits > maxPatchPocketDistance) {
            decision.reason = "patchOutsidePalmPocket";
            return decision;
        }

        const float meaningfulImprovement = (std::max)(1.0f, baseDelta * 0.5f);
        if (decision.pocketImprovementGameUnits < meaningfulImprovement) {
            decision.reason = "patchSeatImprovementTooSmall";
            return decision;
        }
        if (decision.scoreImprovement < meaningfulImprovement) {
            decision.reason = "patchScoreImprovementTooSmall";
            return decision;
        }

        decision.acceptPatchPivot = true;
        decision.positionOnlyAuthority = true;
        decision.reason = "meshBackedPatchPivotPositionOnly";
        return decision;
    }
}

// ---- GrabContactPatchMath.h ----

/*
 * ROCK needs the touched object point to be captured once and then held in the
 * object's own body space. This layer keeps one coherent object-in-hand
 * relationship after selection and gives ROCK a richer contact source by fitting
 * a small palm-facing patch, while leaving Havok body memory and FRIK runtime
 * ownership outside the pure math.
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

    struct ContactPatchProbeGeometry
    {
        float spacingGameUnits = 0.0f;
        float radiusGameUnits = 0.1f;
        float scale = 1.0f;
        const char* reason = "configured";
    };

    inline ContactPatchProbeGeometry computeContactPatchProbeGeometry(
        float configuredSpacingGameUnits,
        float configuredRadiusGameUnits,
        float objectLeverGameUnits,
        float smallObjectReferenceLeverGameUnits,
        float longObjectReferenceLeverGameUnits)
    {
        ContactPatchProbeGeometry result{};
        const float configuredSpacing = std::isfinite(configuredSpacingGameUnits) && configuredSpacingGameUnits > 0.0f ?
            configuredSpacingGameUnits :
            0.0f;
        const float configuredRadius = std::isfinite(configuredRadiusGameUnits) && configuredRadiusGameUnits > 0.0f ?
            configuredRadiusGameUnits :
            0.1f;
        const float smallReference = std::isfinite(smallObjectReferenceLeverGameUnits) && smallObjectReferenceLeverGameUnits > 1.0f ?
            smallObjectReferenceLeverGameUnits :
            12.0f;
        const float longReference = std::isfinite(longObjectReferenceLeverGameUnits) && longObjectReferenceLeverGameUnits > smallReference ?
            longObjectReferenceLeverGameUnits :
            smallReference * 2.0f;
        const float lever = std::isfinite(objectLeverGameUnits) && objectLeverGameUnits > 0.0f ? objectLeverGameUnits : 0.0f;

        if (lever > 0.0f && lever <= smallReference) {
            result.scale = std::clamp(lever / smallReference, 0.45f, 1.0f);
            result.reason = "smallObjectLever";
        } else if (lever >= longReference * 1.35f) {
            result.scale = std::clamp(longReference / lever, 0.55f, 0.85f);
            result.reason = "longObjectLever";
        }

        result.spacingGameUnits = configuredSpacing > 0.0f ? configuredSpacing * result.scale : 0.0f;
        const float radiusScale = std::clamp(result.scale, 0.60f, 1.0f);
        result.radiusGameUnits = (std::max)(0.1f, configuredRadius * radiusScale);
        return result;
    }

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
    struct GrabContactPatchSameSurfaceClusterResult
    {
        std::vector<GrabContactPatchSample<Vector>> samples;
        std::size_t rawAcceptedCount = 0;
        std::size_t anchorRejectedCount = 0;
        std::size_t clusterRejectedCount = 0;
        float maxDepthSpreadGameUnits = 0.0f;
        float maxLateralDistanceGameUnits = 0.0f;
        bool valid = false;
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
    inline bool finiteVector(const Vector& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
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

    template <class Vector, std::size_t Count>
    inline std::size_t buildContactPatchProbeOffsets(std::array<Vector, Count>& offsets, const Vector& palmTangent, const Vector& palmBitangent, float spacingGameUnits)
    {
        offsets = {};

        std::size_t count = 0;
        auto append = [&](const Vector& offset) {
            if (count < offsets.size()) {
                offsets[count++] = offset;
            }
        };

        append(Vector{});
        if constexpr (Count == 0) {
            return count;
        }

        const float spacing = std::isfinite(spacingGameUnits) && spacingGameUnits > 0.0f ? spacingGameUnits : 0.0f;
        const Vector tangent = mul(normalizeOrZero(palmTangent), spacing);
        const Vector bitangent = mul(normalizeOrZero(palmBitangent), spacing);
        const bool hasTangent = lengthSquared(tangent) > 0.0f;
        const bool hasBitangent = lengthSquared(bitangent) > 0.0f;

        if (hasTangent) {
            append(tangent);
            append(negate(tangent));
        }
        if (hasBitangent) {
            append(bitangent);
            append(negate(bitangent));
        }
        if (hasTangent && hasBitangent) {
            append(add(tangent, bitangent));
            append(sub(tangent, bitangent));
            append(add(negate(tangent), bitangent));
            append(negate(add(tangent, bitangent)));
        }

        return count;
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
    inline GrabContactPatchSameSurfaceClusterResult<Vector> filterContactPatchSameSurfaceCluster(
        const std::vector<GrabContactPatchSample<Vector>>& samples,
        const Vector& anchorPoint,
        const Vector& palmNormal,
        float maxAnchorDepthGameUnits,
        float maxClusterDepthSpreadGameUnits,
        float maxAnchorLateralGameUnits,
        float maxNormalAngleDegrees)
    {
        /*
         * Contact patch samples are palm probes, not independent pivot
         * authorities. Anchor the patch to the already selected seated point and
         * keep only one same-surface cluster; otherwise a probe that wraps around
         * a tray corner can stretch the patch across two faces and manufacture a
         * rotation that the user's hand never asked for.
         */
        GrabContactPatchSameSurfaceClusterResult<Vector> result{};
        result.reason = "noAcceptedHits";

        const Vector palm = normalizeOrZero(palmNormal);
        if (!finiteVector(anchorPoint) || lengthSquared(palm) <= 0.0f) {
            result.reason = "invalidAnchorFrame";
            return result;
        }

        const float anchorDepthLimit = (std::max)(0.0f, std::isfinite(maxAnchorDepthGameUnits) ? maxAnchorDepthGameUnits : 0.0f);
        const float clusterDepthLimit = (std::max)(0.0f, std::isfinite(maxClusterDepthSpreadGameUnits) ? maxClusterDepthSpreadGameUnits : 0.0f);
        const float lateralLimit = (std::max)(0.0f, std::isfinite(maxAnchorLateralGameUnits) ? maxAnchorLateralGameUnits : 0.0f);
        const float clampedAngle = std::clamp(maxNormalAngleDegrees, 0.0f, 179.0f);
        const float minNormalDot = std::cos(clampedAngle * 3.14159265358979323846f / 180.0f);

        struct Candidate
        {
            GrabContactPatchSample<Vector> sample{};
            Vector normal{};
            float depth = 0.0f;
            float lateral = 0.0f;
            float score = 0.0f;
            std::size_t inputIndex = 0;
        };

        std::vector<Candidate> candidates;
        candidates.reserve(samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const auto& sample = samples[i];
            if (!sample.accepted) {
                continue;
            }
            ++result.rawAcceptedCount;
            if (!finiteVector(sample.point)) {
                ++result.anchorRejectedCount;
                continue;
            }

            const Vector delta = sub(sample.point, anchorPoint);
            const float depth = dot(delta, palm);
            const Vector lateralVector = projectOntoPlane(delta, palm);
            const float lateral = length(lateralVector);
            if (!std::isfinite(depth) || !std::isfinite(lateral) ||
                std::fabs(depth) > anchorDepthLimit ||
                lateral > lateralLimit) {
                ++result.anchorRejectedCount;
                continue;
            }

            Vector normal = orientNormalTowardPalm(sample.normal, palm);
            if (lengthSquared(normal) <= 0.0f) {
                normal = palm;
            }

            Candidate candidate{};
            candidate.sample = sample;
            candidate.normal = normal;
            candidate.depth = depth;
            candidate.lateral = lateral;
            candidate.score = std::fabs(depth) + lateral * 0.25f;
            candidate.inputIndex = i;
            candidates.push_back(candidate);
        }

        if (candidates.empty()) {
            result.reason = result.rawAcceptedCount > 0 ? "anchorGateRejectedAll" : "noAcceptedHits";
            return result;
        }

        std::vector<std::size_t> bestCluster;
        float bestClusterScore = (std::numeric_limits<float>::max)();
        for (std::size_t seed = 0; seed < candidates.size(); ++seed) {
            std::vector<std::size_t> cluster;
            float clusterScore = 0.0f;
            for (std::size_t index = 0; index < candidates.size(); ++index) {
                const auto& candidate = candidates[index];
                const auto& seedCandidate = candidates[seed];
                if (std::fabs(candidate.depth - seedCandidate.depth) > clusterDepthLimit) {
                    continue;
                }
                if (lengthSquared(candidate.normal) > 0.0f && lengthSquared(seedCandidate.normal) > 0.0f &&
                    dot(candidate.normal, seedCandidate.normal) < minNormalDot) {
                    continue;
                }

                cluster.push_back(index);
                clusterScore += candidate.score;
            }

            if (cluster.empty()) {
                continue;
            }
            const bool betterByCount = cluster.size() > bestCluster.size();
            const bool betterByScore = cluster.size() == bestCluster.size() && clusterScore < bestClusterScore;
            if (betterByCount || betterByScore) {
                bestCluster = std::move(cluster);
                bestClusterScore = clusterScore;
            }
        }

        if (bestCluster.empty()) {
            result.reason = "noSameSurfaceCluster";
            return result;
        }

        std::vector<bool> selected(candidates.size(), false);
        for (const auto index : bestCluster) {
            if (index < selected.size()) {
                selected[index] = true;
            }
        }

        float minDepth = (std::numeric_limits<float>::max)();
        float maxDepth = -(std::numeric_limits<float>::max)();
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (!selected[i]) {
                continue;
            }
            result.samples.push_back(candidates[i].sample);
            minDepth = (std::min)(minDepth, candidates[i].depth);
            maxDepth = (std::max)(maxDepth, candidates[i].depth);
            result.maxLateralDistanceGameUnits = (std::max)(result.maxLateralDistanceGameUnits, candidates[i].lateral);
        }

        result.clusterRejectedCount = (candidates.size() - result.samples.size()) + result.anchorRejectedCount;
        result.maxDepthSpreadGameUnits = result.samples.size() > 1 ? maxDepth - minDepth : 0.0f;
        result.valid = !result.samples.empty();
        result.reason = result.clusterRejectedCount > 0 ? "sameSurfaceClusterRejectedOutliers" : "sameSurfaceCluster";
        return result;
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
    inline Vector freezePivotBBodyLocal(const Transform& bodyWorld, const Vector& gripPointWorld)
    {
        return transform_math::worldPointToLocal(bodyWorld, gripPointWorld);
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

namespace rock::grab_multi_finger_contact_math
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
