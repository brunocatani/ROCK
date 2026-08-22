#pragma once

#include <cmath>
#include <cstdint>

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

namespace rock::weapon_intent_stability_policy
{
    /*
     * Stability of the weapon-to-driver LOCAL relation across frames.
     *
     * During an equip transition the engine attaches the weapon far from the
     * controller and converges over several frames. World-space checks cannot
     * separate that transient from normal locomotion, so the metric is the
     * weapon expressed in the weapon-offset driver's frame: player movement
     * cancels out and only real attach-flight or graph churn moves it.
     *
     * Batch 0 measures and traces this. Batch 4 gates dynamic weapon
     * collision intent admission on it.
     */
    inline constexpr float kDefaultTranslationTolerancePerFrameGameUnits = 0.75f;
    inline constexpr float kDefaultRotationTolerancePerFrameDegrees = 2.0f;
    /*
     * Three consecutive steady frames, about 33 ms at 90 Hz. An equip flight
     * crosses hundreds of game units over many more frames than that, so the
     * gate closes for the whole transient while costing an ordinary draw only
     * a few frames before its collision body starts.
     */
    inline constexpr std::uint32_t kRequiredStableFrameCount = 3;

    struct State
    {
        RE::NiTransform weaponInDriverLocal{};
        std::uint64_t weaponGenerationKey = 0;
        std::uint32_t stableFrameCount = 0;
        bool hasPrevious = false;
    };

    struct Sample
    {
        float translationDeltaGameUnits = -1.0f;
        float rotationDeltaDegrees = -1.0f;
        std::uint32_t stableFrameCount = 0;
        bool valid = false;
    };

    inline void reset(State& state) noexcept
    {
        state = {};
    }

    /*
     * Whether the weapon may be admitted as dynamic collision intent. An
     * unsettled weapon is still flying to its attach point, and driving a
     * collision body along that flight sweeps the proxy through the world and
     * manufactures contacts the player never made.
     */
    [[nodiscard]] inline constexpr bool isAdmissibleCollisionIntent(
        const Sample& sample,
        const std::uint32_t requiredStableFrameCount =
            kRequiredStableFrameCount) noexcept
    {
        return sample.valid &&
            sample.stableFrameCount >= requiredStableFrameCount;
    }

    [[nodiscard]] inline Sample update(
        State& state,
        const bool driverWorldValid,
        const RE::NiTransform& driverWorld,
        const bool weaponWorldValid,
        const RE::NiTransform& weaponWorld,
        const std::uint64_t weaponGenerationKey,
        const float translationTolerancePerFrameGameUnits =
            kDefaultTranslationTolerancePerFrameGameUnits,
        const float rotationTolerancePerFrameDegrees =
            kDefaultRotationTolerancePerFrameDegrees) noexcept
    {
        Sample sample{};
        if (!driverWorldValid || !weaponWorldValid ||
            weaponGenerationKey == 0 ||
            !prefrik_hand_authority_policy::isUsableTransform(driverWorld) ||
            !prefrik_hand_authority_policy::isUsableTransform(weaponWorld)) {
            reset(state);
            return sample;
        }

        const RE::NiTransform weaponInDriverLocal =
            transform_math::composeTransforms(
                transform_math::invertTransform(driverWorld),
                weaponWorld);
        if (!prefrik_hand_authority_policy::isUsableTransform(
                weaponInDriverLocal)) {
            reset(state);
            return sample;
        }

        // A weapon-generation edge is a new attach flight: restart counting.
        if (state.weaponGenerationKey != weaponGenerationKey ||
            !state.hasPrevious) {
            state.weaponInDriverLocal = weaponInDriverLocal;
            state.weaponGenerationKey = weaponGenerationKey;
            state.stableFrameCount = 0;
            state.hasPrevious = true;
            sample.valid = true;
            return sample;
        }

        sample.translationDeltaGameUnits =
            prefrik_hand_authority_policy::translationDeltaGameUnits(
                state.weaponInDriverLocal,
                weaponInDriverLocal);
        sample.rotationDeltaDegrees =
            prefrik_hand_authority_policy::rotationDeltaDegrees(
                state.weaponInDriverLocal,
                weaponInDriverLocal);
        const bool stableThisFrame =
            std::isfinite(sample.translationDeltaGameUnits) &&
            std::isfinite(sample.rotationDeltaDegrees) &&
            sample.translationDeltaGameUnits <=
                translationTolerancePerFrameGameUnits &&
            sample.rotationDeltaDegrees <= rotationTolerancePerFrameDegrees;
        state.stableFrameCount =
            stableThisFrame ? state.stableFrameCount + 1 : 0;
        state.weaponInDriverLocal = weaponInDriverLocal;
        sample.stableFrameCount = state.stableFrameCount;
        sample.valid = true;
        return sample;
    }
}
