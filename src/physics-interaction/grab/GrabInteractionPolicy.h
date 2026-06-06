#pragma once

#include <cstdint>
#include <string_view>

#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/object/GrabTargetKind.h"

namespace rock::grab_interaction_policy
{
    struct LooseObjectSelectionDecision
    {
        bool blocked = false;
        const char* reason = "none";
    };

    /*
     * ROCK treats a far-selected object as an active interaction candidate and
     * transitions it into pull/grab handling from that state. Object-tree prep
     * already owns the FO4VR-specific recursive dynamic conversion, so this
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

    inline bool shouldBlockSelectedObjectInteractionForTarget(
        grab_target::Kind targetKind,
        std::string_view formType,
        bool isLiveNpc,
        bool hasMotionProps,
        std::uint16_t motionPropsId)
    {
        if (targetKind == grab_target::Kind::ActorEquipment ||
            targetKind == grab_target::Kind::LiveActorScissors ||
            targetKind == grab_target::Kind::BlockedWholeActorBody) {
            return true;
        }

        return shouldBlockSelectedObjectInteraction(formType, isLiveNpc, hasMotionProps, motionPropsId);
    }

    inline bool looseObjectSelectionLayerCanReachActiveGrab(std::uint32_t layer) noexcept
    {
        return collision_layer_policy::isDynamicPropInteractionLayer(layer) || layer == collision_layer_policy::FO4_LAYER_PROPS;
    }

    inline LooseObjectSelectionDecision evaluateLooseObjectSelectionCandidate(
        std::string_view formType,
        bool hasMotionProps,
        std::uint16_t motionPropsId,
        bool hasCollisionLayer,
        std::uint32_t collisionLayer)
    {
        if (shouldBlockSelectedObjectInteraction(formType, false, hasMotionProps, motionPropsId)) {
            return { .blocked = true, .reason = "blocked-selected-interaction" };
        }

        if (hasCollisionLayer && !looseObjectSelectionLayerCanReachActiveGrab(collisionLayer)) {
            return { .blocked = true, .reason = "unsupported-active-grab-layer" };
        }

        return {};
    }
}
