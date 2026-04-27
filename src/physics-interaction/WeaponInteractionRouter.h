#pragma once

#include "WeaponSemanticTypes.h"

namespace frik::rock
{
    /*
     * Left-hand weapon behavior needs one authority. The previous boolean
     * offhand-touch path could only say "some weapon body touched", which is
     * not enough once support grip, reload sockets, and action parts all share
     * collision. This router is the small deterministic layer that turns a
     * semantic body contact into the interaction ROCK should run this frame.
     */

    inline WeaponInteractionDecision routeWeaponInteraction(const WeaponInteractionContact& contact, const WeaponReloadRuntimeState& reloadState)
    {
        WeaponInteractionDecision decision{};
        if (!contact.valid) {
            return decision;
        }

        decision.partKind = contact.partKind;
        decision.gripPose = contact.fallbackGripPose;
        decision.bodyId = contact.bodyId;

        if (reloadState.isReloadActive()) {
            if (contact.reloadRole == WeaponReloadRole::MagazineBody) {
                decision.kind = WeaponInteractionKind::RemoveMagazine;
                return decision;
            }
            if (contact.socketRole != WeaponSocketRole::None) {
                decision.kind = WeaponInteractionKind::SocketInsert;
                return decision;
            }
            if (contact.actionRole != WeaponActionRole::None) {
                decision.kind = WeaponInteractionKind::ManipulateAction;
                return decision;
            }
        }

        if (reloadState.supportGripAllowed && contact.supportGripRole != WeaponSupportGripRole::None) {
            decision.kind = WeaponInteractionKind::SupportGrip;
            return decision;
        }

        if (contact.actionRole != WeaponActionRole::None) {
            decision.kind = WeaponInteractionKind::ManipulateAction;
            return decision;
        }

        decision.kind = WeaponInteractionKind::PassiveTouch;
        return decision;
    }
}
