#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"

namespace rock::weapon_input_filter
{
    struct Tuning { float rotationSeconds; float translationSeconds; };

    [[nodiscard]] inline Tuning tuning(int preset)
    {
        switch (preset) {
        case 1: return { 0.012f, 0.006f };
        case 2: return { 0.025f, 0.012f };
        case 3: return { 0.045f, 0.022f };
        case 4: return { 0.070f, 0.035f };
        default: return {};
        }
    }

    [[nodiscard]] inline bool usable(const RE::NiTransform& value)
    {
        return hand_world_claim_registry_policy::isFiniteTransform(value) && value.scale > 0.000001f &&
            hand_world_claim_registry_policy::isUsableTargetRotation(value);
    }

    struct State
    {
        RE::NiTransform filteredLocal{};
        RE::NiPoint3 filteredPivotLocal{};
        RE::NiPoint3 pivotInTarget{};
        RE::NiTransform output{};
        std::uint64_t sequence = 0;
        bool valid = false;
    };

    // State is in room space: locomotion and artificial turning transport it
    // without filtering. The caller resets on ownership/tracking discontinuity.
    [[nodiscard]] inline bool update(State& state, const RE::NiTransform& target,
        const RE::NiTransform& room, const RE::NiPoint3& pivotInTarget,
        std::uint64_t sequence, float dt, bool discontinuity, Tuning config,
        RE::NiTransform& output)
    {
        output = {};
        if (!sequence || !usable(target) || !usable(room) ||
            !std::isfinite(pivotInTarget.x) || !std::isfinite(pivotInTarget.y) || !std::isfinite(pivotInTarget.z)) {
            state = {};
            return false;
        }
        if (config.rotationSeconds <= 0.0f && config.translationSeconds <= 0.0f) {
            state = {};
            output = target;
            return true;
        }
        if (state.valid && sequence == state.sequence) {
            output = state.output;
            return true;
        }
        const auto local = transform_math::composeTransforms(transform_math::invertTransform(room), target);
        const bool seed = !state.valid || discontinuity || sequence != state.sequence + 1;
        if (seed) state.pivotInTarget = pivotInTarget;
        // Keep the physical controller pivot fixed for this input session.
        // Current animation/recoil offsets must not enter tracking history.
        const auto pivot = transform_math::localPointToWorld(local, state.pivotInTarget);
        if (seed) {
            state.filteredLocal = local;
            state.filteredPivotLocal = pivot;
        } else if (std::isfinite(dt) && dt > 0.0f) {
            const auto alpha = [dt](float tau) { return tau > 0.0f ? -std::expm1(-dt / tau) : 1.0f; };
            state.filteredPivotLocal += (pivot - state.filteredPivotLocal) * alpha(config.translationSeconds);
            state.filteredLocal.rotate = hand_visual_lerp_math::quaternionToMatrix<RE::NiMatrix3>(
                hand_visual_lerp_math::slerp(hand_visual_lerp_math::matrixToQuaternion(state.filteredLocal.rotate),
                    hand_visual_lerp_math::matrixToQuaternion(local.rotate), alpha(config.rotationSeconds)));
            state.filteredLocal.scale = local.scale;
        }
        state.filteredLocal.translate = {};
        state.filteredLocal.translate = state.filteredPivotLocal -
            transform_math::localPointToWorld(state.filteredLocal, state.pivotInTarget);
        state.output = transform_math::composeTransforms(room, state.filteredLocal);
        state.sequence = sequence;
        state.valid = usable(state.output);
        output = state.output;
        return state.valid;
    }
}
