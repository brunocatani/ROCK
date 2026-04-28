#pragma once

#include <cstdint>
#include <string_view>

namespace frik::rock::grab_interaction_policy
{
    /*
     * HIGGS treats a far-selected object as an active interaction candidate and
     * transitions it into pull/grab handling from that state. ROCK's object-tree
     * prep already owns the FO4VR-specific recursive dynamic conversion, so this
     * policy only decides whether a selected object is allowed to enter that
     * active path. The important invariant is that configured far selection must
     * not be made inert by a second hard-coded distance gate.
     */
    inline bool canAttemptSelectedObjectGrab(bool isFarSelection, float selectionDistance, float configuredFarRange)
    {
        if (!isFarSelection) {
            return true;
        }

        return configuredFarRange > 0.0f && selectionDistance >= 0.0f && selectionDistance <= configuredFarRange;
    }

    /*
     * Selection and activation must share one gameplay guard. ROCK can select via
     * either close collision hits or far swept-sphere hits, but both paths feed the
     * same dynamic-object preparation. Keeping these exclusions here prevents far
     * pull from bypassing the close-grab checks for doors, furniture, fixed
     * activators, and live actors.
     */
    inline bool shouldBlockSelectedObjectInteraction(std::string_view formType, bool isLiveNpc, bool hasMotionProps, std::uint16_t motionPropsId)
    {
        if (formType == "DOOR" || formType == "CONT" || formType == "TERM" || formType == "FURN") {
            return true;
        }

        if (formType == "ACTI" && hasMotionProps) {
            const auto motionPreset = static_cast<std::uint8_t>(motionPropsId & 0xFF);
            if (motionPreset == 0 || motionPreset == 2) {
                return true;
            }
        }

        return formType == "NPC_" && isLiveNpc;
    }
}
