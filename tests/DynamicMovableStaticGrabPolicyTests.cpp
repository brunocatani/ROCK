#include "physics-interaction/object/PhysicsBodyClassifier.h"

#include <cstdio>

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

    return ok ? 0 : 1;
}
