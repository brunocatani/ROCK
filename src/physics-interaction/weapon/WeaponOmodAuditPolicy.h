#pragma once

#include <cstdint>

namespace rock::weapon_omod_audit_policy
{
    enum class CoverageVerdict : std::uint8_t
    {
        Disabled,
        UnresolvedForm,
        NoModel,
        Ok,
        OkRecordPaired,
        NodePresentNoCollider,
        NodeHiddenNoCollider,
        NodeNotFound,
    };

    struct CoverageInput
    {
        bool disabled = false;
        bool resolved = false;
        bool hasModelToken = false;
        bool hasEvidenceUnderMatch = false;
        bool hasPairedBody = false;
        bool hasNodeMatch = false;
        bool anyNodeMatchVisible = false;
    };

    struct CoverageDecision
    {
        CoverageVerdict verdict = CoverageVerdict::UnresolvedForm;
        bool selfHealCandidate = false;
    };

    [[nodiscard]] inline constexpr bool publishedBodyEvidenceMatchesAudit(
        std::uint64_t auditedEquippedKey,
        std::uint64_t publishedEquippedKey,
        bool hasPublishedBodySet) noexcept
    {
        return hasPublishedBodySet &&
               auditedEquippedKey != 0 &&
               auditedEquippedKey == publishedEquippedKey;
    }

    [[nodiscard]] inline constexpr CoverageDecision decideCoverage(const CoverageInput& input) noexcept
    {
        if (input.disabled) {
            return { .verdict = CoverageVerdict::Disabled };
        }
        if (!input.resolved) {
            return { .verdict = CoverageVerdict::UnresolvedForm };
        }
        if (!input.hasModelToken) {
            return { .verdict = input.hasPairedBody ? CoverageVerdict::OkRecordPaired : CoverageVerdict::NoModel };
        }
        if (input.hasEvidenceUnderMatch) {
            return { .verdict = CoverageVerdict::Ok };
        }
        if (input.hasPairedBody) {
            return { .verdict = CoverageVerdict::OkRecordPaired };
        }
        if (input.hasNodeMatch) {
            return { .verdict = input.anyNodeMatchVisible ? CoverageVerdict::NodePresentNoCollider : CoverageVerdict::NodeHiddenNoCollider };
        }
        return { .verdict = CoverageVerdict::NodeNotFound, .selfHealCandidate = true };
    }

    [[nodiscard]] inline constexpr const char* coverageVerdictName(CoverageVerdict verdict) noexcept
    {
        switch (verdict) {
        case CoverageVerdict::Disabled:
            return "DISABLED";
        case CoverageVerdict::UnresolvedForm:
            return "UNRESOLVED_FORM";
        case CoverageVerdict::NoModel:
            return "NO_MODEL";
        case CoverageVerdict::Ok:
            return "OK";
        case CoverageVerdict::OkRecordPaired:
            return "OK_RECORD_PAIRED";
        case CoverageVerdict::NodePresentNoCollider:
            return "NODE_PRESENT_NO_COLLIDER";
        case CoverageVerdict::NodeHiddenNoCollider:
            return "NODE_HIDDEN_NO_COLLIDER";
        case CoverageVerdict::NodeNotFound:
            return "NODE_NOT_FOUND";
        default:
            return "UNKNOWN";
        }
    }
}
