#pragma once

#include <cmath>
#include <cstdint>

#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiTransform.h"

/*
 * Controller-hand isolation under FRIK API v2.
 *
 * ROCK reads its raw controller hand from the body skeleton's flattened hand
 * bone. FRIK solves that bone to the tracked controller hand, or, while a
 * ROCK claim is registered, to the claim itself - one frame after ROCK
 * published it. Reading the bone as controller input during a claim therefore
 * feeds ROCK its own previous output.
 *
 * FRIK's tracked target is the first-person skeleton hand node: FRIK reads
 * `findNode(fpSkeleton, "RArm_Hand")->world` after the game's own first-person
 * arm update and solves the body arm (looked up under the common node) to it.
 * Nothing writes the first-person hand afterwards, so it is the controller
 * hand whether or not a claim is active, dampening and native kick included.
 *
 * On claim-free frames the flattened bone stays the raw hand, exactly as
 * before, and the relation between the first-person hand and the body hand
 * node (solver residual, skeleton scale) is measured. On claimed frames the
 * raw hand is the first-person hand carried through that relation, plus the
 * palm pitch/yaw blend FRIK applies to the flattened bone on top of its
 * refNode, which is readable every frame as flattened-versus-node.
 */
namespace rock::tracked_hand_isolation_policy
{
    // The solver reaches its target exactly; anything larger is a different
    // skeleton state (scope-menu collapse, missing arm) and is not a relation.
    inline constexpr float kMaxRelationTranslationGameUnits = 4.0f;
    inline constexpr float kMaxRelationRotationDegrees = 12.0f;
    inline constexpr float kMinRelationScale = 0.25f;
    inline constexpr float kMaxRelationScale = 4.0f;

    struct RelationState
    {
        RE::NiTransform firstPersonToBodyHand{};
        bool valid = false;
        std::uint32_t acceptedSamples = 0;
        std::uint32_t rejectedSamples = 0;
    };

    struct FrameInput
    {
        RE::NiTransform firstPersonHandWorld{};
        bool firstPersonHandValid = false;
        // The flattened hand bone's refNode: the body hand FRIK's solver wrote.
        RE::NiTransform bodyHandNodeWorld{};
        bool bodyHandNodeValid = false;
        // The flattened bone ROCK has always read (refNode plus palm blend).
        RE::NiTransform flattenedHandWorld{};
        bool flattenedHandValid = false;
        // FRIK solved this hand to a ROCK claim in the frame just rendered.
        bool claimConsumed = false;
        // False while FRIK composes a native recoil kick onto the tracked
        // target: the body hand then differs from the first-person hand by
        // the kick, which is not a relation.
        bool calibrationAllowed = true;
    };

    enum class RawHandSource : std::uint8_t
    {
        Unavailable,
        Flattened,
        Reconstructed,
        // A claim was consumed but the reconstruction inputs are missing:
        // the flattened bone is ROCK's previous output.
        FlattenedContaminated,
    };

    struct FrameResult
    {
        RE::NiTransform rawHandWorld{};
        RawHandSource source = RawHandSource::Unavailable;
        bool valid = false;
        // Claim-free frames only: reconstruction versus the flattened bone.
        bool probeValid = false;
        float probeTranslationGameUnits = 0.0f;
        float probeRotationDegrees = 0.0f;
    };

    [[nodiscard]] inline bool isFiniteTransform(const RE::NiTransform& transform) noexcept
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return std::isfinite(transform.translate.x) &&
               std::isfinite(transform.translate.y) &&
               std::isfinite(transform.translate.z) &&
               std::isfinite(transform.scale) &&
               std::fabs(transform.scale) > 0.000001f;
    }

    [[nodiscard]] inline float translationGameUnits(const RE::NiTransform& lhs, const RE::NiTransform& rhs) noexcept
    {
        const float x = lhs.translate.x - rhs.translate.x;
        const float y = lhs.translate.y - rhs.translate.y;
        const float z = lhs.translate.z - rhs.translate.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    /*
     * Angle of lhs^T * rhs from both its cosine (trace) and sine (skew part),
     * so a sub-degree difference is exact instead of acos-noise near 1.
     */
    [[nodiscard]] inline float rotationDegrees(const RE::NiTransform& lhs, const RE::NiTransform& rhs) noexcept
    {
        float relative[3][3]{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                float sum = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    sum += lhs.rotate.entry[k][row] * rhs.rotate.entry[k][column];
                }
                relative[row][column] = sum;
            }
        }
        const float trace = relative[0][0] + relative[1][1] + relative[2][2];
        const float skewX = relative[2][1] - relative[1][2];
        const float skewY = relative[0][2] - relative[2][0];
        const float skewZ = relative[1][0] - relative[0][1];
        const float sine = 0.5f * std::sqrt(skewX * skewX + skewY * skewY + skewZ * skewZ);
        const float cosine = 0.5f * (trace - 1.0f);
        return std::atan2(sine, cosine) * 57.29577951308232f;
    }

    /*
     * Measure body-hand-node = firstPersonHand * relation on a claim-free
     * frame. Rejected samples keep the previous relation.
     */
    [[nodiscard]] inline bool calibrateRelation(
        RelationState& state,
        const RE::NiTransform& firstPersonHandWorld,
        const RE::NiTransform& bodyHandNodeWorld) noexcept
    {
        if (!isFiniteTransform(firstPersonHandWorld) || !isFiniteTransform(bodyHandNodeWorld)) {
            ++state.rejectedSamples;
            return false;
        }
        const RE::NiTransform relation = transform_math::composeTransforms(
            transform_math::invertTransform(firstPersonHandWorld),
            bodyHandNodeWorld);
        const RE::NiTransform identity = transform_math::makeIdentityTransform<RE::NiTransform>();
        if (!isFiniteTransform(relation) ||
            translationGameUnits(relation, identity) > kMaxRelationTranslationGameUnits ||
            rotationDegrees(relation, identity) > kMaxRelationRotationDegrees ||
            relation.scale < kMinRelationScale || relation.scale > kMaxRelationScale) {
            ++state.rejectedSamples;
            return false;
        }
        state.firstPersonToBodyHand = relation;
        state.valid = true;
        ++state.acceptedSamples;
        return true;
    }

    /*
     * The palm pitch/yaw blend FRIK applied to the flattened bone this frame,
     * as flattened = bodyHandNode * blend. Independent of who owns the arm.
     */
    [[nodiscard]] inline RE::NiTransform makePalmBlend(
        const RE::NiTransform& bodyHandNodeWorld,
        const RE::NiTransform& flattenedHandWorld) noexcept
    {
        return transform_math::composeTransforms(
            transform_math::invertTransform(bodyHandNodeWorld),
            flattenedHandWorld);
    }

    [[nodiscard]] inline bool reconstruct(
        const RelationState& state,
        const FrameInput& input,
        RE::NiTransform& outRawHandWorld) noexcept
    {
        outRawHandWorld = {};
        if (!state.valid || !input.firstPersonHandValid || !input.bodyHandNodeValid || !input.flattenedHandValid ||
            !isFiniteTransform(input.firstPersonHandWorld) || !isFiniteTransform(input.bodyHandNodeWorld) ||
            !isFiniteTransform(input.flattenedHandWorld)) {
            return false;
        }
        const RE::NiTransform controllerBodyHand = transform_math::composeTransforms(
            input.firstPersonHandWorld,
            state.firstPersonToBodyHand);
        const RE::NiTransform raw = transform_math::orthonormalizedTransform(transform_math::composeTransforms(
            controllerBodyHand,
            makePalmBlend(input.bodyHandNodeWorld, input.flattenedHandWorld)));
        if (!isFiniteTransform(raw)) {
            return false;
        }
        outRawHandWorld = raw;
        return true;
    }

    /*
     * Every input basis is orthonormalized first. The rendered bones carry
     * float drift from FRIK's matrix chains, and on claimed frames the body
     * bones are ROCK's own previous target: a drifted basis that is inverted
     * as a transpose here and rendered again next frame squares the drift
     * every frame until the hand stretches and flies off.
     */
    [[nodiscard]] inline FrameResult resolveFrame(RelationState& state, const FrameInput& rawInput) noexcept
    {
        FrameInput input = rawInput;
        input.firstPersonHandWorld = transform_math::orthonormalizedTransform(rawInput.firstPersonHandWorld);
        input.bodyHandNodeWorld = transform_math::orthonormalizedTransform(rawInput.bodyHandNodeWorld);
        input.flattenedHandWorld = transform_math::orthonormalizedTransform(rawInput.flattenedHandWorld);
        FrameResult result{};
        if (!input.claimConsumed) {
            if (input.flattenedHandValid && isFiniteTransform(input.flattenedHandWorld)) {
                result.rawHandWorld = input.flattenedHandWorld;
                result.source = RawHandSource::Flattened;
                result.valid = true;
            }
            if (input.calibrationAllowed && input.firstPersonHandValid && input.bodyHandNodeValid) {
                (void)calibrateRelation(state, input.firstPersonHandWorld, input.bodyHandNodeWorld);
            }
            RE::NiTransform reconstructed{};
            if (result.valid && reconstruct(state, input, reconstructed)) {
                result.probeValid = true;
                result.probeTranslationGameUnits = translationGameUnits(reconstructed, input.flattenedHandWorld);
                result.probeRotationDegrees = rotationDegrees(reconstructed, input.flattenedHandWorld);
            }
            return result;
        }

        if (reconstruct(state, input, result.rawHandWorld)) {
            result.source = RawHandSource::Reconstructed;
            result.valid = true;
            return result;
        }
        if (input.flattenedHandValid && isFiniteTransform(input.flattenedHandWorld)) {
            result.rawHandWorld = input.flattenedHandWorld;
            result.source = RawHandSource::FlattenedContaminated;
            result.valid = true;
        }
        return result;
    }
}
