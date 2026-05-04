#pragma once

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
