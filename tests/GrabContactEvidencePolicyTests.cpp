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

    auto hybridTrustedPatch = hybridMesh;
    hybridTrustedPatch.contactPatchAccepted = true;
    hybridTrustedPatch.contactPatchMeshSnapped = true;
    hybridTrustedPatch.contactPatchNormalTrusted = true;
    hybridTrustedPatch.contactPatchConfidence = 0.80f;
    decision = evaluateGrabContactEvidence(hybridTrustedPatch);
    ok &= expectTrue("hybrid mesh accepts trusted patch orientation", decision.accept);
    ok &= expectReason("hybrid trusted patch reason", decision.reason, "meshSurfaceContactPatch");

    auto hybridPositionOnlyPatch = hybridTrustedPatch;
    hybridPositionOnlyPatch.contactPatchNormalTrusted = false;
    hybridPositionOnlyPatch.contactPatchPositionOnly = true;
    decision = evaluateGrabContactEvidence(hybridPositionOnlyPatch);
    ok &= expectTrue("hybrid mesh keeps position-only patch as mesh baseline", decision.accept);
    ok &= expectReason("hybrid position-only patch does not become orientation evidence", decision.reason, "meshSurfacePivot");

    GrabContactEvidenceInput patchOnlyPositionOnly{};
    patchOnlyPositionOnly.qualityMode = static_cast<int>(GrabContactQualityMode::HybridEvidence);
    patchOnlyPositionOnly.multiFingerValidationEnabled = true;
    patchOnlyPositionOnly.contactPatchAccepted = true;
    patchOnlyPositionOnly.contactPatchMeshSnapped = true;
    patchOnlyPositionOnly.contactPatchPositionOnly = true;
    patchOnlyPositionOnly.contactPatchConfidence = 1.0f;
    decision = evaluateGrabContactEvidence(patchOnlyPositionOnly);
    ok &= expectFalse("position-only patch alone is not reliable orientation evidence", decision.accept);
    ok &= expectReason("position-only patch alone reason", decision.reason, "positionOnlyPatchNeedsFingerOrSeat");

    // Exercise actual ancestry/collision traversal, including the modular
    // single-body layout that cannot resolve a visual owner by ancestry alone.
    struct Node {
        Node* parent = nullptr;
        const void* collisionObject = nullptr;
    };
    using rock::grab_contact_source_policy::isSingleBodyOwnerlessVisualBranch;
    Node root{}, receiver{ &root, &root }, attachment{ &root }, mesh{ &attachment };
    ok &= expectTrue("single collision owner accepts a visual sibling",
        isSingleBodyOwnerlessVisualBranch(&root, &receiver, &mesh));
    Node otherRoot{}, foreignBody{ &otherRoot, &otherRoot }, foreignMesh{ &otherRoot };
    ok &= expectFalse("foreign body owner cannot authorize this reference",
        isSingleBodyOwnerlessVisualBranch(&root, &foreignBody, &mesh));
    ok &= expectFalse("foreign mesh cannot attach to selected body",
        isSingleBodyOwnerlessVisualBranch(&root, &receiver, &foreignMesh));
    attachment.collisionObject = &attachment;
    ok &= expectFalse("an independently colliding attachment is not ownerless",
        isSingleBodyOwnerlessVisualBranch(&root, &receiver, &mesh));
    attachment.collisionObject = nullptr;
    mesh.collisionObject = &mesh;
    ok &= expectFalse("a colliding surface cannot borrow another body's authority",
        isSingleBodyOwnerlessVisualBranch(&root, &receiver, &mesh));
    mesh.collisionObject = nullptr;
    receiver.collisionObject = nullptr;
    ok &= expectFalse("missing body collision owner fails closed",
        isSingleBodyOwnerlessVisualBranch(&root, &receiver, &mesh));
    receiver.collisionObject = &root;
    attachment.parent = &mesh;
    ok &= expectFalse("cyclic visual ancestry terminates without acceptance",
        isSingleBodyOwnerlessVisualBranch(&root, &receiver, &mesh));
    attachment.parent = &root;
    receiver.parent = &receiver;
    ok &= expectFalse("cyclic body ancestry terminates without acceptance",
        isSingleBodyOwnerlessVisualBranch(&root, &receiver, &mesh));
    receiver.parent = &root;
    ok &= expectFalse("missing reference root fails closed",
        isSingleBodyOwnerlessVisualBranch(static_cast<Node*>(nullptr), &receiver, &mesh));

    return ok ? 0 : 1;
}
