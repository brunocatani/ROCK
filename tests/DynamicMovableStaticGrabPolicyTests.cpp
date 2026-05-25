#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/object/FarSelectionBlacklistPolicy.h"
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

    ok &= expectAccepted("active-prepped loose pickup on static layer is accepted",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_STATIC, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab));

    ok &= expectAccepted("active-prepped loose pickup on animstatic layer is accepted",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_ANIMSTATIC, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab));

    ok &= expectAccepted("active-prepped loose pickup on props layer is accepted",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_PROPS, BodyMotionType::Dynamic),
            InteractionMode::ActiveGrab));

    ok &= expectRejected("static loose pickup on static layer remains blocked",
        physics_body_classifier::classifyBody(
            makeInput(grab_target::Kind::LooseObject, collision_layer_policy::FO4_LAYER_STATIC, BodyMotionType::Static),
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

    auto matrix = makeFullyEnabledMatrix();
    const auto originalControllerMask = matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER];
    collision_layer_policy::applyNativeCharacterControllerObjectSuppressionPolicy(matrix.data(), true, originalControllerMask);
    const auto expectedSuppressedControllerMask =
        collision_layer_policy::nativeCharacterControllerExpectedMask(originalControllerMask, true);

    ok &= expectLayerPair("native controller no longer hits clutter", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_CLUTTER, false);
    ok &= expectLayerPair("native controller no longer hits weapon objects", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_WEAPON, false);
    ok &= expectLayerPair("native controller no longer hits small debris", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_DEBRIS_SMALL, false);
    ok &= expectLayerPair("native controller no longer hits large debris", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_DEBRIS_LARGE, false);
    ok &= expectLayerPair("native controller no longer hits shell casings", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_SHELLCASING, false);
    ok &= expectLayerPair("native controller no longer hits large clutter", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_CLUTTER_LARGE, false);
    ok &= expectLayerPair("native controller preserves static support", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_STATIC, true);
    ok &= expectLayerPair("native controller preserves animstatic support", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_ANIMSTATIC, true);
    ok &= expectLayerPair("native controller preserves terrain support", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_TERRAIN, true);
    ok &= expectLayerPair("native controller preserves ground support", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_GROUND, true);
    ok &= expectTrue("native controller suppression matrix matches expected object pairs",
        collision_layer_policy::nativeCharacterControllerObjectPairsMatch(matrix.data(), expectedSuppressedControllerMask));
    ok &= expectFalse("native controller object suppression does not manage ROCK hand layer",
        collision_layer_policy::isNativeCharacterControllerObjectSuppressionLayer(collision_layer_policy::ROCK_LAYER_HAND));
    ok &= expectFalse("native controller object suppression does not manage ROCK weapon layer",
        collision_layer_policy::isNativeCharacterControllerObjectSuppressionLayer(collision_layer_policy::ROCK_LAYER_WEAPON));
    ok &= expectFalse("native controller object suppression does not manage ROCK body layer",
        collision_layer_policy::isNativeCharacterControllerObjectSuppressionLayer(collision_layer_policy::ROCK_LAYER_BODY));

    collision_layer_policy::applyNativeCharacterControllerObjectSuppressionPolicy(matrix.data(), false, originalControllerMask);
    ok &= expectLayerPair("disabled native controller policy restores clutter", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_CLUTTER, true);
    ok &= expectLayerPair("disabled native controller policy restores weapon objects", matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::FO4_LAYER_WEAPON, true);
    ok &= expectTrue("disabled native controller policy matches original object pairs",
        collision_layer_policy::nativeCharacterControllerObjectPairsMatch(
            matrix.data(),
            collision_layer_policy::nativeCharacterControllerExpectedMask(originalControllerMask, false)));

    {
        alignas(void*) char manifold[sizeof(char*) + sizeof(int)]{};
        alignas(void*) char simplex[held_grab_cc_policy::kGeneratedConstraintCountOffset + sizeof(int)]{};
        alignas(void*) char constraintRows[held_grab_cc_policy::kGeneratedContactStride * 2]{};

        *reinterpret_cast<char**>(manifold) = nullptr;
        *reinterpret_cast<int*>(manifold + sizeof(char*)) = 0;
        *reinterpret_cast<char**>(simplex + held_grab_cc_policy::kGeneratedConstraintRowsOffset) = constraintRows;
        *reinterpret_cast<int*>(simplex + held_grab_cc_policy::kGeneratedConstraintCountOffset) = 2;

        const auto view = held_grab_cc_policy::makeGeneratedContactBufferView(manifold, simplex);
        ok &= expectFalse("constraint-only player controller view is not body-identifiable", view.valid);
        const auto cleared = held_grab_cc_policy::clearGeneratedConstraintOnlyContacts(view);
        ok &= expectTrue("constraint-only player controller contacts can be cleared fail-closed", cleared.valid);
        ok &= expectTrue("constraint-only player controller contacts remove every unidentified row", cleared.removedPairCount == 2);
        ok &= expectTrue("constraint-only player controller clear zeros native constraint count",
            *reinterpret_cast<int*>(simplex + held_grab_cc_policy::kGeneratedConstraintCountOffset) == 0);
    }

    return ok ? 0 : 1;
}
