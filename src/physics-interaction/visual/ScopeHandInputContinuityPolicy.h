#pragma once

#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"
#include "physics-interaction/hand/HandVisual.h"

namespace rock::scope_hand_input_continuity_policy
{
    using hand_world_claim_registry_policy::DriverSample;

    struct Damping
    {
        float translation = 0.0f;
        float rotation = 0.0f;
        bool enabled = false;
        bool inScope = false;
        bool valid = false;
    };

    // These are accepted input observations, never rendered hand claims.
    struct State
    {
        RE::NiTransform handInDriver{};
        RE::NiTransform previousDriver{};
        RE::NiPoint3 previousCamera{};
        std::uint64_t sequence = 0;
        bool relationValid = false;
        bool previousValid = false;
        bool suspended = false;
        bool recovering = false;
    };

    struct Result
    {
        DriverSample driver{};
        DriverSample hand{};
        bool corrected = false;
        float residualGameUnits = 0.0f;
        float residualDegrees = 0.0f;
    };

    [[nodiscard]] inline bool usable(const RE::NiTransform& world)
    {
        return hand_world_claim_registry_policy::isFiniteTransform(world) &&
            world.scale > 0.000001f && hand_world_claim_registry_policy::isUsableTargetRotation(world);
    }

    [[nodiscard]] inline RE::NiTransform dampen(const RE::NiTransform& raw,
        const RE::NiTransform& previous, const RE::NiPoint3& cameraDelta, const Damping& config)
    {
        auto result = raw;
        result.translate = raw.translate - (raw.translate - previous.translate - cameraDelta) * config.translation;
        result.rotate = hand_visual_lerp_math::quaternionToMatrix<RE::NiMatrix3>(
            hand_visual_lerp_math::slerp(hand_visual_lerp_math::matrixToQuaternion(previous.rotate),
                hand_visual_lerp_math::matrixToQuaternion(raw.rotate), 1.0f - config.rotation));
        return result;
    }

    // FRIK freezes its damping history when scope damping is disabled. Keep
    // observing the undamped driver there, then resume the same filter from
    // the last scoped frame until native first-person input agrees again.
    // The native WeaponOffset world may have had recoil restored after the
    // arm update, so convergence is measured against FirstPersonHand instead.
    [[nodiscard]] inline Result resolve(State& state, const Damping& config, bool scoped,
        std::uint64_t sequence, const DriverSample& raw, const DriverSample& nativeDriver,
        const DriverSample& nativeHand, const RE::NiPoint3& camera, bool cameraValid)
    {
        namespace registry = hand_world_claim_registry_policy;
        Result result{ nativeDriver, nativeHand };
        const bool suspended = config.valid && config.enabled && !config.inScope && scoped;
        state.recovering = config.valid && config.enabled && !scoped && (state.recovering || state.suspended);
        state.suspended = suspended;
        const bool rawValid = raw.valid && usable(raw.world);
        if (suspended) {
            // Both inputs are from this arm update. Do not learn this relation
            // from native history on the exit edge, or from a rendered wrist.
            if (rawValid && nativeHand.valid && usable(nativeHand.world)) {
                const auto relation = transform_math::orthonormalizedTransform(transform_math::composeTransforms(
                    transform_math::invertTransform(transform_math::orthonormalizedTransform(raw.world)), nativeHand.world));
                state.relationValid = usable(relation) &&
                    relation.translate.Length() <= 30.0f && relation.scale >= 0.25f && relation.scale <= 4.0f;
                if (state.relationValid) state.handInDriver = relation;
            }
            state.previousDriver = raw.world;
            state.previousCamera = camera;
            state.previousValid = rawValid && cameraValid;
            state.sequence = sequence;
        } else if (state.recovering) {
            result = {};
            result.corrected = true;
            if (rawValid && cameraValid && state.relationValid) {
                // A skipped frame cannot supply filter history. Current input
                // remains usable; stale history must never be replayed.
                const bool consecutive = state.previousValid && state.sequence != 0 &&
                    sequence > state.sequence && sequence - state.sequence == 1;
                const auto driver = consecutive ?
                    dampen(raw.world, state.previousDriver, camera - state.previousCamera, config) : raw.world;
                const auto hand = transform_math::orthonormalizedTransform(
                    transform_math::composeTransforms(driver, state.handInDriver));
                if (usable(driver) && usable(hand)) {
                    result.driver = { driver, true };
                    result.hand = { hand, true };
                    if (nativeHand.valid && usable(nativeHand.world)) {
                        result.residualGameUnits = registry::translationDeltaGameUnits(hand, nativeHand.world);
                        result.residualDegrees = registry::rotationDeltaDegrees(hand, nativeHand.world);
                        if (result.residualGameUnits <= 0.05f && result.residualDegrees <= 0.05f &&
                            std::fabs(hand.scale - nativeHand.world.scale) <= 0.0001f) {
                            state.recovering = false;
                            result = { nativeDriver, nativeHand, false, result.residualGameUnits, result.residualDegrees };
                        }
                    }
                }
            }
            state.previousDriver = result.driver.world;
            state.previousCamera = camera;
            state.previousValid = result.driver.valid && cameraValid;
            state.sequence = sequence;
        }
        return result;
    }
}
