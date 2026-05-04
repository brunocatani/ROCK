#pragma once

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
