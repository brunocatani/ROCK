#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/object/FarSelectionBlacklistPolicy.h"

#include <cstdio>
#include <string_view>

namespace
{
    rock::physics_body_classifier::BodyClassificationInput makeInput(
        rock::grab_target::Kind targetKind,
        std::uint32_t layer,
        rock::physics_body_classifier::BodyMotionType motionType)
    {
        rock::physics_body_classifier::BodyClassificationInput input{};
        input.bodyId = 1001u;
        input.motionId = 77u;
        input.layer = layer;
        input.motionType = motionType;
        input.targetKind = targetKind;
        return input;
    }

    bool expectAccepted(const char* label, const rock::physics_body_classifier::BodyClassificationResult& result)
    {
        if (result.accepted) {
            return true;
        }
        std::printf("%s expected accepted, got reason=%s\n", label, rock::physics_body_classifier::rejectReasonName(result.reason));
        return false;
    }

    bool expectRejected(
        const char* label,
        const rock::physics_body_classifier::BodyClassificationResult& result,
        rock::physics_body_classifier::BodyRejectReason reason)
    {
        if (!result.accepted && result.reason == reason) {
            return true;
        }
        std::printf("%s expected rejection=%s, got accepted=%s reason=%s\n",
            label,
            rock::physics_body_classifier::rejectReasonName(reason),
            result.accepted ? "yes" : "no",
            rock::physics_body_classifier::rejectReasonName(result.reason));
        return false;
    }

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

    bool expectPlayerControllerDecision(
        const char* label,
        const rock::collision_layer_policy::PlayerCharacterControllerContactPolicyDecision& decision,
        bool suppress,
        std::string_view reason)
    {
        if (decision.suppress == suppress && std::string_view(decision.reason) == reason) {
            return true;
        }
        std::printf("%s expected suppress=%s reason=%.*s, got suppress=%s reason=%s\n",
            label,
            suppress ? "true" : "false",
            static_cast<int>(reason.size()),
            reason.data(),
            decision.suppress ? "true" : "false",
            decision.reason);
        return false;
    }
}

int main()
{
    using namespace rock;
    using physics_body_classifier::BodyMotionType;
    using physics_body_classifier::BodyRejectReason;
    using physics_body_classifier::InteractionMode;

    bool ok = true;

    ok &= expectAccepted("dynamic movable static on static layer",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::DynamicMovableStatic, collision_layer_policy::FO4_LAYER_STATIC, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab));

    ok &= expectAccepted("dynamic movable static on deadbip layer",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::DynamicMovableStatic, collision_layer_policy::FO4_LAYER_DEADBIP, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab));

    ok &= expectRejected("static movable static remains blocked",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::DynamicMovableStatic, collision_layer_policy::FO4_LAYER_STATIC, BodyMotionType::Static),
            InteractionMode::ActiveGrab),
        BodyRejectReason::StaticMotion);

    ok &= expectRejected("ordinary loose object on static layer remains blocked",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_STATIC, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab),
        BodyRejectReason::UnsupportedLayer);

    ok &= expectAccepted("dead actor body on biped layer",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::DeadActorBody, collision_layer_policy::FO4_LAYER_BIPED, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab));

    ok &= expectRejected("static dead actor body remains blocked",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::DeadActorBody, collision_layer_policy::FO4_LAYER_BIPED, BodyMotionType::Static),
            InteractionMode::ActiveGrab),
        BodyRejectReason::StaticMotion);

    ok &= expectTrue("dynamic movable statics require hand pocket", grab_target::requiresHandPocketGrab(grab_target::Kind::DynamicMovableStatic));
    ok &= expectTrue("detached gore requires hand pocket", grab_target::requiresHandPocketGrab(grab_target::Kind::DetachedGore));
    ok &= expectTrue("dead actor bodies require hand pocket", grab_target::requiresHandPocketGrab(grab_target::Kind::DeadActorBody));
    ok &= expectFalse("ordinary loose objects may use normal grab evidence", grab_target::requiresHandPocketGrab(grab_target::Kind::LooseObject));
    ok &= expectFalse("dynamic movable statics cannot be far selected", grab_target::canUseFarSelection(grab_target::Kind::DynamicMovableStatic));
    ok &= expectFalse("detached gore cannot be far selected", grab_target::canUseFarSelection(grab_target::Kind::DetachedGore));
    ok &= expectFalse("dead actor bodies cannot be far selected", grab_target::canUseFarSelection(grab_target::Kind::DeadActorBody));
    ok &= expectTrue("ordinary loose objects may use far selection", grab_target::canUseFarSelection(grab_target::Kind::LooseObject));
    ok &= expectFalse("dynamic movable statics cannot use dynamic pull", grab_target::canUseRockDynamicPull(grab_target::Kind::DynamicMovableStatic));
    ok &= expectFalse("detached gore cannot use dynamic pull", grab_target::canUseRockDynamicPull(grab_target::Kind::DetachedGore));
    ok &= expectFalse("dead actor bodies cannot use dynamic pull", grab_target::canUseRockDynamicPull(grab_target::Kind::DeadActorBody));
    ok &= expectTrue("ordinary loose objects may use dynamic pull", grab_target::canUseRockDynamicPull(grab_target::Kind::LooseObject));
    ok &= expectTrue("far blacklist blocks reference form id",
        far_selection_blacklist_policy::evaluateFarSelectionBlacklist(far_selection_blacklist_policy::FarSelectionBlacklistInput{
            .isFarSelection = true,
            .referenceFormId = 0x00056FB4,
            .blockedReferenceFormIds = "00056FB4",
        }).blocked);
    ok &= expectTrue("far blacklist blocks base form id",
        far_selection_blacklist_policy::evaluateFarSelectionBlacklist(far_selection_blacklist_policy::FarSelectionBlacklistInput{
            .isFarSelection = true,
            .baseFormId = 0x000AA001,
            .blockedBaseFormIds = "0x000AA001",
        }).blocked);
    ok &= expectTrue("far blacklist blocks form type case-insensitively",
        far_selection_blacklist_policy::evaluateFarSelectionBlacklist(far_selection_blacklist_policy::FarSelectionBlacklistInput{
            .isFarSelection = true,
            .formType = "ACTI",
            .blockedFormTypes = "door, acti",
        }).blocked);
    ok &= expectTrue("far blacklist blocks animstatic layer by name",
        far_selection_blacklist_policy::evaluateFarSelectionBlacklist(far_selection_blacklist_policy::FarSelectionBlacklistInput{
            .isFarSelection = true,
            .collisionLayer = collision_layer_policy::FO4_LAYER_ANIMSTATIC,
            .blockedLayers = "ANIMSTATIC",
        }).blocked);
    ok &= expectFalse("far blacklist ignores close selections",
        far_selection_blacklist_policy::evaluateFarSelectionBlacklist(far_selection_blacklist_policy::FarSelectionBlacklistInput{
            .isFarSelection = false,
            .referenceFormId = 0x00056FB4,
            .blockedReferenceFormIds = "00056FB4",
        }).blocked);

    ok &= expectPlayerControllerDecision("static support layer remains preserved for player controller",
        collision_layer_policy::evaluatePlayerCharacterControllerContact(collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
            .filterEnabled = true,
            .playerController = true,
            .targetLayerKnown = true,
            .targetLayer = collision_layer_policy::FO4_LAYER_STATIC,
        }),
        false,
        "supportLayer");

    ok &= expectPlayerControllerDecision("dynamic movable static on static support layer is suppressed",
        collision_layer_policy::evaluatePlayerCharacterControllerContact(collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
            .filterEnabled = true,
            .playerController = true,
            .targetLayerKnown = true,
            .targetLayer = collision_layer_policy::FO4_LAYER_STATIC,
            .targetIsMovableStatic = true,
        }),
        true,
        "movableStaticSupportLayer");

    ok &= expectPlayerControllerDecision("dynamic movable static on animstatic support layer is suppressed",
        collision_layer_policy::evaluatePlayerCharacterControllerContact(collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
            .filterEnabled = true,
            .playerController = true,
            .targetLayerKnown = true,
            .targetLayer = collision_layer_policy::FO4_LAYER_ANIMSTATIC,
            .targetIsMovableStatic = true,
        }),
        true,
        "movableStaticSupportLayer");

    ok &= expectPlayerControllerDecision("dynamic movable static flag does not suppress non-player controllers",
        collision_layer_policy::evaluatePlayerCharacterControllerContact(collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
            .filterEnabled = true,
            .playerController = false,
            .targetLayerKnown = true,
            .targetLayer = collision_layer_policy::FO4_LAYER_STATIC,
            .targetIsMovableStatic = true,
        }),
        false,
        "nonPlayerController");

    return ok ? 0 : 1;
}
