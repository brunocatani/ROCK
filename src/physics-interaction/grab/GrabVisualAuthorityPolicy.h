#pragma once

#include <string_view>

/*
 * ROCK's grab pivot must be driven by the live tracking hand, not by the
 * object-reverse-aligned visual hand. HIGGS keeps raw hand state and adjusted
 * hand visuals as separate concepts; this policy makes that split explicit so
 * generic object grabs cannot feed the held object's transform back into the
 * hand frame that computes pivot A. The object-reverse solve is only safe when
 * the captured object point is an authored anchor, opposition frame, or
 * mesh-validated contact patch; a plain fallback mesh point can make FRIK solve
 * the wrist around the wrong pivot even when the physics grab itself is stable.
 *
 * The validated-contact state is intentionally finalized at the end of grab
 * frame construction. Authored nodes and opposition frames are discovered after
 * early mesh/contact fallback setup, so writing the frame latch too early makes
 * the physical grab use a good source while the visual IK path rejects it.
 */

namespace rock::grab_visual_authority_policy
{
    struct VisualAuthorityContactState
    {
        bool valid = false;
        const char* reason = "unvalidatedContact";
    };

    inline const char* fallbackReasonForGrabPointMode(const char* grabPointMode)
    {
        const std::string_view mode = grabPointMode ? std::string_view(grabPointMode) : std::string_view{};
        if (mode.find("semantic") != std::string_view::npos) {
            return "unvalidatedSemanticContact";
        }
        if (mode.find("meshSurface") != std::string_view::npos) {
            return "unvalidatedMeshSurface";
        }
        return "unvalidatedContact";
    }

    inline void markValidatedContact(VisualAuthorityContactState& state, const char* reason)
    {
        state.valid = true;
        state.reason = reason && reason[0] ? reason : "validatedContact";
    }

    inline void assignFallbackReasonIfInvalid(VisualAuthorityContactState& state, const char* grabPointMode)
    {
        if (!state.valid) {
            state.reason = fallbackReasonForGrabPointMode(grabPointMode);
        }
    }

    inline bool shouldApplyObjectReverseAlignedExternalHandTransform(bool enabled, bool hasValidAdjustedTarget)
    {
        return enabled && hasValidAdjustedTarget;
    }

    inline bool shouldApplyObjectReverseAlignedExternalHandTransform(bool enabled, bool hasValidAdjustedTarget, bool hasValidatedContact)
    {
        return enabled && hasValidAdjustedTarget && hasValidatedContact;
    }

    inline bool shouldUseObjectReverseAlignedHandForFingerPose(bool enabled, bool hasValidAdjustedTarget)
    {
        return enabled && hasValidAdjustedTarget;
    }

    inline bool shouldUseObjectReverseAlignedHandForFingerPose(bool enabled, bool hasValidAdjustedTarget, bool hasValidatedContact)
    {
        return enabled && hasValidAdjustedTarget && hasValidatedContact;
    }
}
