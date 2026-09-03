#pragma once

#include <cstdint>
#include <string_view>

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

/*
 * Controller-space transport of the rendered hand chains under FRIK API v2.
 *
 * FRIK solves the whole arm to a ROCK claim, so on a claimed frame every bone
 * of that hand in the root flattened tree (forearm, hand, fingers) is ROCK's
 * previous target, not the controller. TrackedHandIsolationPolicy recovers the
 * hand root; consumers that measure against the controller (hand colliders and
 * their palm anchor, grab and support-grip fingers, finger chains) need the
 * rest of the chain where the tracked solve would have put it.
 *
 * FRIK's hand pose lives in locals below the hand root, so carrying the root
 * and everything under it by the rigid delta between the isolated controller
 * root and the rendered root is exact for the hand and fingers. The forearm
 * bones above the root take the same delta: the hand compound keeps the shape
 * it was rendered with and only its elbow end differs by the solver's small
 * elbow shift for the claim offset. Upper arm, collarbone and twists stay
 * rendered; body colliders collide where the body draws.
 */
namespace rock::rendered_bone_transport_policy
{
    enum class HandChainSide : std::uint8_t
    {
        None,
        Right,
        Left,
    };

    // A delta below these is the claim-free case (isolated root == rendered root).
    inline constexpr float kIdentityTranslationEpsilonGameUnits = 0.0005f;
    inline constexpr float kIdentityRotationEpsilonDegrees = 0.005f;

    struct HandTransport
    {
        RE::NiTransform delta{};
        bool active = false;
    };

    [[nodiscard]] inline HandChainSide chainSideForBone(const std::string_view name) noexcept
    {
        HandChainSide side = HandChainSide::None;
        if (name.starts_with("RArm_")) {
            side = HandChainSide::Right;
        } else if (name.starts_with("LArm_")) {
            side = HandChainSide::Left;
        } else {
            return HandChainSide::None;
        }
        const std::string_view tail = name.substr(5);
        if (tail == "Hand" || tail == "ForeArm1" || tail == "ForeArm2" || tail == "ForeArm3" || tail.starts_with("Finger")) {
            return side;
        }
        return HandChainSide::None;
    }

    /*
     * delta = controllerRoot * inverse(renderedRoot). Inactive when either root
     * is missing or when the two coincide, so claim-free frames pass through.
     */
    [[nodiscard]] inline HandTransport makeHandTransport(
        const RE::NiTransform& controllerRoot,
        const bool controllerRootValid,
        const RE::NiTransform& renderedRoot,
        const bool renderedRootValid) noexcept
    {
        namespace isolation = tracked_hand_isolation_policy;
        HandTransport transport{};
        if (!controllerRootValid || !renderedRootValid ||
            !isolation::isFiniteTransform(controllerRoot) || !isolation::isFiniteTransform(renderedRoot)) {
            return transport;
        }
        const RE::NiTransform delta = transform_math::composeTransforms(
            controllerRoot,
            transform_math::invertTransform(renderedRoot));
        if (!isolation::isFiniteTransform(delta)) {
            return transport;
        }
        const RE::NiTransform identity = transform_math::makeIdentityTransform<RE::NiTransform>();
        if (isolation::translationGameUnits(delta, identity) <= kIdentityTranslationEpsilonGameUnits &&
            isolation::rotationDegrees(delta, identity) <= kIdentityRotationEpsilonDegrees) {
            return transport;
        }
        transport.delta = delta;
        transport.active = true;
        return transport;
    }

    [[nodiscard]] inline RE::NiTransform transportWorld(const HandTransport& transport, const RE::NiTransform& renderedWorld) noexcept
    {
        if (!transport.active) {
            return renderedWorld;
        }
        return transform_math::composeTransforms(transport.delta, renderedWorld);
    }
}
