#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/grab/GrabInteractionPolicy.h"
#include "physics-interaction/object/FarSelectionBlacklistPolicy.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/grab/GrabHeldObject.h"

#include <array>
#include <cstdint>
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

    using CollisionMatrix = std::array<std::uint64_t, rock::collision_layer_policy::FO4_LAYER_MATRIX_ADDRESSABLE_COUNT>;

    CollisionMatrix makeFullyEnabledMatrix()
    {
        CollisionMatrix matrix{};
        matrix.fill(rock::collision_layer_policy::allMatrixAddressableLayerBits());
        return matrix;
    }

    bool expectLayerPair(
        const char* label,
        const CollisionMatrix& matrix,
        std::uint32_t layerA,
        std::uint32_t layerB,
        bool expectedEnabled)
    {
        const bool matches =
            rock::collision_layer_policy::layerPairSymmetricMatches(matrix.data(), layerA, layerB, expectedEnabled);
        if (matches) {
            return true;
        }
        std::printf("%s expected layer pair %u<->%u enabled=%s\n",
            label,
            layerA,
            layerB,
            expectedEnabled ? "true" : "false");
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

    ok &= expectRejected("active-prepped loose pickup on static layer remains blocked",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_STATIC, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab),
        BodyRejectReason::UnsupportedLayer);

    ok &= expectRejected("active-prepped loose pickup on animstatic layer remains blocked",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_ANIMSTATIC, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab),
        BodyRejectReason::UnsupportedLayer);

    ok &= expectAccepted("active-prepped loose pickup on props layer is accepted",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_PROPS, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab));

    ok &= expectRejected("static loose pickup on static layer remains blocked at the layer gate",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_STATIC, BodyMotionType::Static),
            InteractionMode::ActiveGrab),
        BodyRejectReason::UnsupportedLayer);

    ok &= expectRejected("static loose pickup on props layer remains blocked by motion",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_PROPS, BodyMotionType::Static),
            InteractionMode::ActiveGrab),
        BodyRejectReason::StaticMotion);

    ok &= expectRejected("passive push still treats static-layer loose pickups as support",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_STATIC, BodyMotionType::Dynamic),
            InteractionMode::PassivePush),
        BodyRejectReason::UnsupportedLayer);

    ok &= expectRejected("passive push still rejects props-layer loose pickups",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_PROPS, BodyMotionType::Dynamic),
            InteractionMode::PassivePush),
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

    for (auto layer : {collision_layer_policy::FO4_LAYER_CLUTTER,
             collision_layer_policy::FO4_LAYER_DEBRIS_SMALL, collision_layer_policy::FO4_LAYER_DEBRIS_LARGE}) {
        for (bool outsideRoot : {false, true}) {
            const auto partKind = grab_target::classifyDeadActorPhysicalTarget(true, layer, outsideRoot);
            ok &= expectTrue("dynamic NPC fragment is a detached part regardless of actor ancestry", partKind == grab_target::Kind::DetachedGore);
            ok &= expectAccepted("detached NPC fragment reaches active grab admission",
                physics_body_classifier::classifyBody(makeInput(partKind, layer, BodyMotionType::Dynamic), InteractionMode::ActiveGrab));
            ok &= expectFalse("NPC fragment does not become a whole-actor far pull", grab_target::canUseRockDynamicPull(partKind));
        }
        ok &= expectTrue("non-dynamic NPC fragment remains blocked",
            grab_target::classifyDeadActorPhysicalTarget(false, layer, false) == grab_target::Kind::BlockedWholeActorBody);
    }
    ok &= expectTrue("connected dead-biped retains corpse handling",
        grab_target::classifyDeadActorPhysicalTarget(true, collision_layer_policy::FO4_LAYER_DEADBIP, false) == grab_target::Kind::DeadActorBody);
    ok &= expectTrue("detached dead-biped retains detached-part handling",
        grab_target::classifyDeadActorPhysicalTarget(true, collision_layer_policy::FO4_LAYER_DEADBIP, true) == grab_target::Kind::DetachedGore);
    ok &= expectTrue("actor character-controller is not a detached physical part",
        grab_target::classifyDeadActorPhysicalTarget(true, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, true) == grab_target::Kind::BlockedWholeActorBody);
    ok &= expectTrue("unrelated actor projectile is not admitted as gore",
        grab_target::classifyDeadActorPhysicalTarget(true, collision_layer_policy::FO4_LAYER_PROJECTILE, false) == grab_target::Kind::BlockedWholeActorBody);
    ok &= expectTrue("unrelated actor shell casing is not admitted as gore",
        grab_target::classifyDeadActorPhysicalTarget(true, collision_layer_policy::FO4_LAYER_SHELLCASING, false) == grab_target::Kind::BlockedWholeActorBody);

    ok &= expectTrue("dynamic movable statics require hand pocket", grab_target::requiresHandPocketGrab(grab_target::Kind::DynamicMovableStatic));
    ok &= expectTrue("detached gore requires hand pocket", grab_target::requiresHandPocketGrab(grab_target::Kind::DetachedGore));
    ok &= expectTrue("dead actor bodies require hand pocket", grab_target::requiresHandPocketGrab(grab_target::Kind::DeadActorBody));
    ok &= expectFalse("ordinary loose objects may use normal grab evidence", grab_target::requiresHandPocketGrab(grab_target::Kind::LooseObject));
    ok &= expectFalse("car grab is blocked outside power armor",
        car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = true,
            .playerInPowerArmor = false,
        }).allowed);
    ok &= expectTrue("car grab is allowed in power armor",
        car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = true,
            .playerInPowerArmor = true,
        }).allowed);
    ok &= expectTrue("close car selection remains available for dynamic-world collision outside power armor",
        car_interaction_policy::evaluateSelection(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = true,
            .playerInPowerArmor = false,
        }, false).allowed);
    ok &= expectFalse("far car selection remains blocked outside power armor",
        car_interaction_policy::evaluateSelection(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = true,
            .playerInPowerArmor = false,
        }, true).allowed);
    ok &= expectTrue("non-car movable static grab is unaffected outside power armor",
        car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = false,
            .playerInPowerArmor = false,
        }).allowed);
    ok &= expectFalse("dynamic movable statics cannot be far selected", grab_target::canUseFarSelection(grab_target::Kind::DynamicMovableStatic));
    ok &= expectFalse("detached gore cannot be far selected", grab_target::canUseFarSelection(grab_target::Kind::DetachedGore));
    ok &= expectFalse("dead actor bodies cannot be far selected", grab_target::canUseFarSelection(grab_target::Kind::DeadActorBody));
    ok &= expectTrue("ordinary loose objects may use far selection", grab_target::canUseFarSelection(grab_target::Kind::LooseObject));
    ok &= expectFalse("dynamic movable statics cannot use dynamic pull", grab_target::canUseRockDynamicPull(grab_target::Kind::DynamicMovableStatic));
    ok &= expectFalse("detached gore cannot use dynamic pull", grab_target::canUseRockDynamicPull(grab_target::Kind::DetachedGore));
    ok &= expectFalse("dead actor bodies cannot use dynamic pull", grab_target::canUseRockDynamicPull(grab_target::Kind::DeadActorBody));
    ok &= expectTrue("ordinary loose objects may use dynamic pull", grab_target::canUseRockDynamicPull(grab_target::Kind::LooseObject));
    ok &= expectTrue("static activator is not a loose selection candidate",
        grab_interaction_policy::evaluateLooseObjectSelectionCandidate(
            "ACTI",
            true,
            0,
            true,
            collision_layer_policy::FO4_LAYER_PROPS)
            .blocked);
    ok &= expectTrue("keyframed activator is not a loose selection candidate",
        grab_interaction_policy::evaluateLooseObjectSelectionCandidate(
            "ACTI",
            true,
            2,
            true,
            collision_layer_policy::FO4_LAYER_PROPS)
            .blocked);
    ok &= expectTrue("animstatic loose-form candidate cannot be selected for active grab",
        grab_interaction_policy::evaluateLooseObjectSelectionCandidate(
            "ACTI",
            false,
            0,
            true,
            collision_layer_policy::FO4_LAYER_ANIMSTATIC)
            .blocked);
    ok &= expectFalse("dynamic prop loose candidate remains selectable",
        grab_interaction_policy::evaluateLooseObjectSelectionCandidate(
            "MISC",
            true,
            1,
            true,
            collision_layer_policy::FO4_LAYER_CLUTTER)
            .blocked);
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

    ok &= expectPlayerControllerDecision("car on clutter keeps native player collision",
        collision_layer_policy::evaluatePlayerCharacterControllerContact(collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
            .filterEnabled = true,
            .playerController = true,
            .targetLayerKnown = true,
            .targetLayer = collision_layer_policy::FO4_LAYER_CLUTTER,
            .targetIsMovableStatic = true,
            .targetIsCar = true,
        }),
        false,
        "carCollision");

    ok &= expectPlayerControllerDecision("car on large clutter keeps native player collision",
        collision_layer_policy::evaluatePlayerCharacterControllerContact(collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
            .filterEnabled = true,
            .playerController = true,
            .targetLayerKnown = true,
            .targetLayer = collision_layer_policy::FO4_LAYER_CLUTTER_LARGE,
            .targetIsMovableStatic = true,
            .targetIsCar = true,
        }),
        false,
        "carCollision");

    ok &= expectPlayerControllerDecision("ordinary clutter remains suppressed by body-aware filtering",
        collision_layer_policy::evaluatePlayerCharacterControllerContact(collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
            .filterEnabled = true,
            .playerController = true,
            .targetLayerKnown = true,
            .targetLayer = collision_layer_policy::FO4_LAYER_CLUTTER,
        }),
        true,
        "nonSupportLayer");

    ok &= expectPlayerControllerDecision("car identity overrides movable-static support suppression",
        collision_layer_policy::evaluatePlayerCharacterControllerContact(collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
            .filterEnabled = true,
            .playerController = true,
            .targetLayerKnown = true,
            .targetLayer = collision_layer_policy::FO4_LAYER_STATIC,
            .targetIsMovableStatic = true,
            .targetIsCar = true,
        }),
        false,
        "carCollision");

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

    auto generatedWeaponMatrix = makeFullyEnabledMatrix();
    collision_layer_policy::applyRockGeneratedLayerPolicies(
        generatedWeaponMatrix.data(),
        true,
        false,
        false);
    ok &= expectLayerPair("ROCK weapon keeps world weapon-layer collision",
        generatedWeaponMatrix,
        collision_layer_policy::ROCK_LAYER_WEAPON,
        collision_layer_policy::FO4_LAYER_WEAPON,
        true);
    ok &= expectLayerPair("ROCK weapon preserves clutter collision",
        generatedWeaponMatrix,
        collision_layer_policy::ROCK_LAYER_WEAPON,
        collision_layer_policy::FO4_LAYER_CLUTTER,
        true);
    ok &= expectLayerPair("ROCK weapon preserves NPC biped collision",
        generatedWeaponMatrix,
        collision_layer_policy::ROCK_LAYER_WEAPON,
        collision_layer_policy::FO4_LAYER_BIPED,
        true);
    ok &= expectTrue("VRMeleeImpact drop set covers every ROCK-owned layer",
        collision_layer_policy::isRockOwnedMatrixLayer(collision_layer_policy::ROCK_LAYER_HAND) &&
            collision_layer_policy::isRockOwnedMatrixLayer(collision_layer_policy::ROCK_LAYER_WEAPON) &&
            collision_layer_policy::isRockOwnedMatrixLayer(collision_layer_policy::ROCK_LAYER_BODY) &&
            collision_layer_policy::isRockOwnedMatrixLayer(collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY) &&
            collision_layer_policy::isRockOwnedMatrixLayer(collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY));
    ok &= expectFalse("VRMeleeImpact drop set keeps the native weapon layer visible",
        collision_layer_policy::isRockOwnedMatrixLayer(collision_layer_policy::FO4_LAYER_WEAPON));
    ok &= expectFalse("VRMeleeImpact drop set keeps the NPC biped layer visible",
        collision_layer_policy::isRockOwnedMatrixLayer(collision_layer_policy::FO4_LAYER_BIPED));

    ok &= expectTrue("clutter cars map to a dedicated dynamic-world layer",
        collision_layer_policy::dynamicWorldCarLayerForNativeLayer(collision_layer_policy::FO4_LAYER_CLUTTER) ==
            collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER);
    ok &= expectTrue("large-clutter cars map to a distinct dynamic-world layer",
        collision_layer_policy::dynamicWorldCarLayerForNativeLayer(collision_layer_policy::FO4_LAYER_CLUTTER_LARGE) ==
            collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER);
    ok &= expectTrue("car clutter layer restores to native clutter",
        collision_layer_policy::nativeLayerForDynamicWorldCarLayer(collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER) ==
            collision_layer_policy::FO4_LAYER_CLUTTER);
    ok &= expectTrue("car large-clutter layer restores to native large clutter",
        collision_layer_policy::nativeLayerForDynamicWorldCarLayer(collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER) ==
            collision_layer_policy::FO4_LAYER_CLUTTER_LARGE);

    CollisionMatrix dynamicCarMatrix{};
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER, collision_layer_policy::FO4_LAYER_CLUTTER, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER, collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER, collision_layer_policy::FO4_LAYER_STATIC, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER, collision_layer_policy::FO4_LAYER_ITEMPICK, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, collision_layer_policy::FO4_LAYER_ANIMSTATIC, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, collision_layer_policy::FO4_LAYER_ITEMPICK, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER, collision_layer_policy::ROCK_LAYER_HAND, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER, collision_layer_policy::ROCK_LAYER_WEAPON, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER, collision_layer_policy::ROCK_LAYER_BODY, true);
    collision_layer_policy::setPair(dynamicCarMatrix.data(), collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, collision_layer_policy::ROCK_LAYER_BODY, true);
    collision_layer_policy::applyRockDynamicHandProxyLayerPolicies(dynamicCarMatrix.data());
    collision_layer_policy::applyRockDynamicWeaponProxyLayerPolicy(dynamicCarMatrix.data());
    collision_layer_policy::applyRockDynamicWorldCarLayerPolicies(dynamicCarMatrix.data());

    ok &= expectLayerPair("dynamic hand proxy still excludes ordinary clutter", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY, collision_layer_policy::FO4_LAYER_CLUTTER, false);
    ok &= expectLayerPair("dynamic hand proxy still excludes ordinary large clutter", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY, collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, false);
    ok &= expectLayerPair("dynamic hand proxy includes tagged clutter cars", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY, collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, true);
    ok &= expectLayerPair("dynamic hand proxy includes tagged large-clutter cars", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY, collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER, true);
    ok &= expectLayerPair("dynamic weapon proxy still excludes ordinary clutter", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY, collision_layer_policy::FO4_LAYER_CLUTTER, false);
    ok &= expectLayerPair("dynamic weapon proxy still excludes ordinary large clutter", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY, collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, false);
    ok &= expectLayerPair("dynamic weapon proxy includes tagged clutter cars", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY, collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, true);
    ok &= expectLayerPair("dynamic weapon proxy includes tagged large-clutter cars", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY, collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER, true);
    ok &= expectLayerPair("tagged clutter car preserves native static collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, collision_layer_policy::FO4_LAYER_STATIC, true);
    ok &= expectLayerPair("tagged large-clutter car preserves native animstatic collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER, collision_layer_policy::FO4_LAYER_ANIMSTATIC, true);
    ok &= expectLayerPair("tagged car layers preserve native clutter cross-collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER, true);
    ok &= expectLayerPair("tagged clutter car remains visible to selection queries", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, collision_layer_policy::FO4_LAYER_ITEMPICK, true);
    ok &= expectLayerPair("tagged car preserves native player-controller collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, true);
    ok &= expectLayerPair("tagged clutter car does not inherit unrelated actor collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, collision_layer_policy::FO4_LAYER_BIPED, false);
    ok &= expectLayerPair("tagged car does not inherit generated hand collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, collision_layer_policy::ROCK_LAYER_HAND, false);
    ok &= expectLayerPair("tagged car does not inherit generated weapon collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, collision_layer_policy::ROCK_LAYER_WEAPON, false);
    ok &= expectLayerPair("tagged car does not inherit generated body or leg collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, collision_layer_policy::ROCK_LAYER_BODY, false);
    ok &= expectLayerPair("tagged large car does not inherit generated body or leg collision", dynamicCarMatrix,
        collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER, collision_layer_policy::ROCK_LAYER_BODY, false);



    {
        alignas(void*) char manifold[sizeof(char*) + sizeof(int)]{};
        alignas(void*) char simplex[held_grab_cc_policy::kGeneratedConstraintCountOffset + sizeof(int)]{};
        alignas(void*) char constraintRows[held_grab_cc_policy::kGeneratedContactStride * 2]{};
        *reinterpret_cast<char**>(simplex + held_grab_cc_policy::kGeneratedConstraintRowsOffset) = constraintRows;
        auto* count = reinterpret_cast<int*>(simplex + held_grab_cc_policy::kGeneratedConstraintCountOffset);
        *count = 2;
        const auto view = held_grab_cc_policy::makeGeneratedContactBufferView(manifold, simplex);
        bool predicateCalled = false;
        const auto filtered = held_grab_cc_policy::filterGeneratedContactBuffers(view, [&](std::uint32_t) {
            predicateCalled = true;
            return true;
        });
        ok &= expectFalse("unidentified constraint-only contacts cannot be classified", filtered.valid);
        ok &= expectFalse("missing body identities never reach the suppression predicate", predicateCalled);
        ok &= expectTrue("native support and attack rows survive missing identities", *count == 2);
    }

    // An unmatched native tail is not identified by the paired-prefix filter.
    // Test both directions, with the removed row in the middle of the prefix.
    for (bool manifoldHasTail : { false, true }) {
        constexpr auto stride = held_grab_cc_policy::kGeneratedContactStride;
        std::array<char, stride * 4> manifoldRows{}, constraintRows{};
        for (int i = 0; i < 4; ++i) {
            const std::uint32_t id = 100 + i;
            const std::uint32_t tag = 200 + i;
            std::memcpy(manifoldRows.data() + i * stride + held_grab_cc_policy::kGeneratedContactBodyIdOffset, &id, sizeof(id));
            std::memcpy(constraintRows.data() + i * stride, &tag, sizeof(tag));
        }
        int manifoldCount = manifoldHasTail ? 4 : 3;
        int constraintCount = manifoldHasTail ? 3 : 4;
        const held_grab_cc_policy::GeneratedContactBufferView view{
            .valid = true, .manifoldEntries = manifoldRows.data(), .constraintEntries = constraintRows.data(),
            .manifoldCountPtr = &manifoldCount, .constraintCountPtr = &constraintCount,
            .manifoldCount = manifoldCount, .constraintCount = constraintCount, .pairCount = 3,
        };
        const auto result = held_grab_cc_policy::filterGeneratedContactBuffers(view,
            [](std::uint32_t id) { return id == 101; });
        std::uint32_t keptBody = 0, keptConstraint = 0, tail = 0;
        std::memcpy(&keptBody, manifoldRows.data() + stride + held_grab_cc_policy::kGeneratedContactBodyIdOffset, sizeof(keptBody));
        std::memcpy(&keptConstraint, constraintRows.data() + stride, sizeof(keptConstraint));
        const auto* tailRow = manifoldHasTail ?
            manifoldRows.data() + 2 * stride + held_grab_cc_policy::kGeneratedContactBodyIdOffset :
            constraintRows.data() + 2 * stride;
        std::memcpy(&tail, tailRow, sizeof(tail));
        ok &= expectTrue("identified contact removal preserves paired row alignment",
            result.removedPairCount == 1 && keptBody == 102 && keptConstraint == 202);
        ok &= expectTrue("only identified rows are removed from each native buffer",
            manifoldCount == (manifoldHasTail ? 3 : 2) && constraintCount == (manifoldHasTail ? 2 : 3));
        ok &= expectTrue("unmatched native support/constraint tail survives compaction",
            tail == (manifoldHasTail ? 103u : 203u));
    }

    return ok ? 0 : 1;
}
