#include "physics-interaction/grab/GrabContact.h"

#include <cstdio>
#include <string_view>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectReason(const char* label, const char* actual, std::string_view expected)
    {
        if (actual && actual == expected) {
            return true;
        }
        std::printf("%s expected reason %.*s got %s\n",
            label,
            static_cast<int>(expected.size()),
            expected.data(),
            actual ? actual : "(null)");
        return false;
    }
}

int main()
{
    using namespace rock::grab_contact_evidence_policy;

    bool ok = true;

    GrabContactEvidenceInput strictNoAuthority{};
    strictNoAuthority.qualityMode = static_cast<int>(GrabContactQualityMode::StrictMultiFinger);
    strictNoAuthority.multiFingerValidationEnabled = true;
    strictNoAuthority.multiFingerGripValid = true;
    strictNoAuthority.combinedFingerGroups = 2;
    strictNoAuthority.minimumFingerGroups = 2;

    auto decision = evaluateGrabContactEvidence(strictNoAuthority);
    ok &= expectFalse("strict mode rejects finger-only evidence", decision.accept);
    ok &= expectTrue("strict mode records strict requirement", decision.strictMultiFingerRequired);
    ok &= expectReason("strict mode requires object pivot authority", decision.reason, "strictObjectPivotAuthorityRequired");

    auto strictMeshAuthority = strictNoAuthority;
    strictMeshAuthority.meshSurfacePivotAccepted = true;
    decision = evaluateGrabContactEvidence(strictMeshAuthority);
    ok &= expectTrue("strict mode accepts mesh authority plus fingers", decision.accept);
    ok &= expectReason("strict mesh authority reason", decision.reason, "strictMultiFingerSatisfied");

    auto strictPatchAuthority = strictNoAuthority;
    strictPatchAuthority.contactPatchAccepted = true;
    strictPatchAuthority.contactPatchMeshSnapped = true;
    decision = evaluateGrabContactEvidence(strictPatchAuthority);
    ok &= expectTrue("strict mode accepts mesh-snapped patch authority plus fingers", decision.accept);
    ok &= expectReason("strict patch authority reason", decision.reason, "strictMultiFingerSatisfied");

    GrabContactEvidenceInput hybridMesh{};
    hybridMesh.qualityMode = static_cast<int>(GrabContactQualityMode::HybridEvidence);
    hybridMesh.multiFingerValidationEnabled = true;
    hybridMesh.meshSurfacePivotAccepted = true;
    decision = evaluateGrabContactEvidence(hybridMesh);
    ok &= expectTrue("hybrid mode still accepts baseline mesh surface authority", decision.accept);
    ok &= expectReason("hybrid mesh baseline reason", decision.reason, "meshSurfacePivot");

    return ok ? 0 : 1;
}
