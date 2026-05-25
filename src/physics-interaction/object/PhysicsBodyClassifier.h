#pragma once

#include <cstdint>

#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/object/GrabTargetKind.h"

namespace rock::physics_body_classifier
{
    enum class BodyMotionType : std::uint8_t
    {
        Unknown,
        Static,
        Dynamic,
        Keyframed,
        Other,
    };

    enum class InteractionMode : std::uint8_t
    {
        PassivePush,
        ActiveGrab,
    };

    enum class BodyRejectReason : std::uint8_t
    {
        None,
        InvalidBodyId,
        InvalidMotionId,
        DeletedOrDisabledReference,
        RockHandBody,
        RockGeneratedBody,
        RockWeaponSourceBody,
        HeldBySameHand,
        PlayerBody,
        CharacterController,
        ActorLayer,
        NonCollidableLayer,
        UnsupportedLayer,
        StaticMotion,
        KeyframedPassive,
        NotDynamicAfterActivePrep,
    };

    struct BodyClassificationInput
    {
        std::uint32_t bodyId = 0x7FFF'FFFF;
        std::uint32_t motionId = 0;
        std::uint32_t layer = 0;
        std::uint32_t filterInfo = 0;
        BodyMotionType motionType = BodyMotionType::Unknown;
        grab_target::Kind targetKind = grab_target::Kind::LooseObject;
        std::uint32_t bodyFlags = 0;
        bool referenceDeletedOrDisabled = false;
        bool isRockHandBody = false;
        bool isRockGeneratedBody = false;
        bool isRockWeaponSourceBody = false;
        bool isHeldBySameHand = false;
        bool isPlayerBody = false;
    };

    struct BodyClassificationResult
    {
        bool accepted = false;
        BodyRejectReason reason = BodyRejectReason::None;
    };

    inline constexpr const char* rejectReasonName(BodyRejectReason reason)
    {
        switch (reason) {
        case BodyRejectReason::None:
            return "accepted";
        case BodyRejectReason::InvalidBodyId:
            return "invalid-body-id";
        case BodyRejectReason::InvalidMotionId:
            return "invalid-motion-id";
        case BodyRejectReason::DeletedOrDisabledReference:
            return "deleted-or-disabled-ref";
        case BodyRejectReason::RockHandBody:
            return "rock-hand-body";
        case BodyRejectReason::RockGeneratedBody:
            return "rock-generated-body";
        case BodyRejectReason::RockWeaponSourceBody:
            return "rock-weapon-source-body";
        case BodyRejectReason::HeldBySameHand:
            return "held-by-same-hand";
        case BodyRejectReason::PlayerBody:
            return "player-body";
        case BodyRejectReason::CharacterController:
            return "character-controller";
        case BodyRejectReason::ActorLayer:
            return "actor-layer";
        case BodyRejectReason::NonCollidableLayer:
            return "non-collidable-layer";
        case BodyRejectReason::UnsupportedLayer:
            return "unsupported-layer";
        case BodyRejectReason::StaticMotion:
            return "static-motion";
        case BodyRejectReason::KeyframedPassive:
            return "keyframed-passive";
        case BodyRejectReason::NotDynamicAfterActivePrep:
            return "not-dynamic-after-active-prep";
        }
        return "unknown";
    }

    inline constexpr BodyClassificationResult reject(BodyRejectReason reason) { return BodyClassificationResult{ .accepted = false, .reason = reason }; }

    inline constexpr BodyClassificationResult accept() { return BodyClassificationResult{ .accepted = true, .reason = BodyRejectReason::None }; }

    /*
     * FO4VR's hknpBody flags are the authoritative runtime motion-state bits.
     * Logs from active grab prep show valid movable props with motion-properties
     * ids such as 5 while the body flags encode DYNAMIC. ROCK therefore
     * classifies by runtime body state first and uses motion-properties ids only
     * as secondary diagnostics/fallbacks.
     */
    inline constexpr BodyMotionType motionTypeFromBodyFlags(std::uint32_t bodyFlags)
    {
        if ((bodyFlags & 0x01) != 0) {
            return BodyMotionType::Static;
        }
        if ((bodyFlags & 0x06) == 0x06) {
            return BodyMotionType::Keyframed;
        }
        if ((bodyFlags & 0x06) == 0x02) {
            return BodyMotionType::Dynamic;
        }
        return BodyMotionType::Unknown;
    }

    inline constexpr BodyMotionType motionTypeFromMotionPropertiesId(std::uint16_t motionPropertiesId)
    {
        switch (motionPropertiesId & 0xFF) {
        case 0:
            return BodyMotionType::Static;
        case 1:
            return BodyMotionType::Dynamic;
        case 2:
            return BodyMotionType::Keyframed;
        default:
            return BodyMotionType::Other;
        }
    }

    inline constexpr BodyClassificationResult classifyBody(const BodyClassificationInput& input, InteractionMode mode)
    {
        if (input.bodyId == 0x7FFF'FFFF) {
            return reject(BodyRejectReason::InvalidBodyId);
        }
        if (input.motionId == 0) {
            return reject(BodyRejectReason::InvalidMotionId);
        }
        if (input.referenceDeletedOrDisabled) {
            return reject(BodyRejectReason::DeletedOrDisabledReference);
        }
        if (input.isRockHandBody) {
            return reject(BodyRejectReason::RockHandBody);
        }
        if (input.isRockWeaponSourceBody) {
            return reject(BodyRejectReason::RockWeaponSourceBody);
        }
        if (input.isRockGeneratedBody) {
            return reject(BodyRejectReason::RockGeneratedBody);
        }
        if (input.isHeldBySameHand) {
            return reject(BodyRejectReason::HeldBySameHand);
        }
        if (input.isPlayerBody) {
            return reject(BodyRejectReason::PlayerBody);
        }
        if (input.layer == collision_layer_policy::FO4_LAYER_CHARCONTROLLER) {
            return reject(BodyRejectReason::CharacterController);
        }
        const bool activeDetachedGoreLayer =
            mode == InteractionMode::ActiveGrab &&
            input.targetKind == grab_target::Kind::DetachedGore &&
            grab_target::isDetachedGoreLayer(input.layer);
        const bool activeDynamicMovableStatic =
            mode == InteractionMode::ActiveGrab &&
            input.targetKind == grab_target::Kind::DynamicMovableStatic;
        /*
         * Some pickup refs, including consumables, keep a native support/props
         * layer on their hknp body even after active-prep converts the body
         * motion. The target-kind classifier has already proven this is a
         * loose-form ref, so the layer gate allows it through and the motion
         * checks below still fail closed unless prep produced a dynamic body.
         */
        const bool activeLoosePickupLayer =
            mode == InteractionMode::ActiveGrab &&
            input.targetKind == grab_target::Kind::LooseObject &&
            (collision_layer_policy::isWorldSurfaceLayer(input.layer) || input.layer == collision_layer_policy::FO4_LAYER_PROPS);
        const bool activeDeadActorBody =
            mode == InteractionMode::ActiveGrab &&
            input.targetKind == grab_target::Kind::DeadActorBody &&
            collision_layer_policy::isActorOrBipedLayer(input.layer);
        if (mode != InteractionMode::PassivePush && collision_layer_policy::isActorOrBipedLayer(input.layer) && !activeDetachedGoreLayer && !activeDynamicMovableStatic && !activeDeadActorBody) {
            return reject(BodyRejectReason::ActorLayer);
        }
        if (input.layer == collision_layer_policy::FO4_LAYER_NONCOLLIDABLE) {
            return reject(BodyRejectReason::NonCollidableLayer);
        }
        const bool interactionLayer =
            mode == InteractionMode::PassivePush ? collision_layer_policy::isPassivePushInteractionLayer(input.layer) :
                                                   (collision_layer_policy::isDynamicPropInteractionLayer(input.layer) || activeDetachedGoreLayer || activeDynamicMovableStatic || activeLoosePickupLayer || activeDeadActorBody);
        if (!interactionLayer) {
            return reject(BodyRejectReason::UnsupportedLayer);
        }
        if (input.motionType == BodyMotionType::Static) {
            return reject(BodyRejectReason::StaticMotion);
        }
        if (mode == InteractionMode::PassivePush && input.motionType == BodyMotionType::Keyframed) {
            return reject(BodyRejectReason::KeyframedPassive);
        }
        if (mode == InteractionMode::ActiveGrab && input.motionType != BodyMotionType::Dynamic) {
            return reject(BodyRejectReason::NotDynamicAfterActivePrep);
        }
        if (mode == InteractionMode::PassivePush && input.motionType != BodyMotionType::Dynamic) {
            return reject(BodyRejectReason::NotDynamicAfterActivePrep);
        }
        return accept();
    }
}
