#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"

namespace rock::weapon_input_filter
{
    // releaseDegreesPerSecond > 0 makes the smoothing speed-adaptive: each
    // time constant scales by release / (release + rotation speed), full while
    // the weapon holds still and fading as it swings.
    struct Tuning { float rotationSeconds; float translationSeconds; float releaseDegreesPerSecond = 0.0f; };

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

    // Reframe the existing input by controller motion only. An animated offset
    // node is not a controller frame: FRIK may restore native recoil there after
    // solving the neutral hand, so its inverse would subtract recoil from input.
    [[nodiscard]] inline RE::NiTransform applyTrackingCorrection(
        const RE::NiTransform& input, const RE::NiTransform& trackedWand,
        const RE::NiTransform& filteredWand)
    {
        return transform_math::composeTransforms(filteredWand,
            transform_math::composeTransforms(transform_math::invertTransform(trackedWand), input));
    }

    // Generated grab proxies store column axes; TransformMath uses row axes.
    // Correct the row view, then restore proxy storage before driving Havok.
    [[nodiscard]] inline RE::NiTransform applyProxyTrackingCorrection(
        RE::NiTransform proxy, const RE::NiTransform& trackedWand,
        const RE::NiTransform& filteredWand)
    {
        proxy.rotate = transform_math::transposeRotation(proxy.rotate);
        proxy = applyTrackingCorrection(proxy, trackedWand, filteredWand);
        proxy.rotate = transform_math::transposeRotation(proxy.rotate);
        return proxy;
    }

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

    /*
     * Equipped weapon pose filter. Smooths the final solved weapon pose, so
     * one hand, two hands, gunstock, left firing and the second weapon all
     * get the same smoothing from the same tuning. Works in room space
     * (locomotion and turning pass unfiltered) around a weapon-local pivot,
     * the firing palm. Every write in a frame steps from the previous frame's
     * result and the last write commits, so a pre-write and a final write in
     * one frame never advance the filter twice.
     */
    class PoseFilter
    {
    public:
        void reset() noexcept { *this = {}; }

        // The tuning turned off while the output still trailed the input:
        // the output eases onto the input instead of jumping.
        [[nodiscard]] bool settling() const noexcept { return _settling; }

        // holdOffset keeps the previous frame's offset rigid in the weapon
        // frame (a recoil kick passes through unsmoothed and filtering resumes
        // from there). Returns false and outputs the target when unusable.
        bool apply(const RE::NiTransform& target, const RE::NiTransform& room, const void* roomSource,
            const void* identity, const RE::NiPoint3& pivotLocal, std::uint64_t frame, float dt,
            bool discontinuity, bool holdOffset, Tuning config, RE::NiTransform& output)
        {
            output = target;
            if (!frame || !usable(target) || !usable(room) ||
                !std::isfinite(pivotLocal.x) || !std::isfinite(pivotLocal.y) || !std::isfinite(pivotLocal.z)) {
                reset();
                return false;
            }
            if (frame != _pendingFrame) {
                const bool contiguous = _pendingValid && frame == _pendingFrame + 1 && !discontinuity &&
                    identity == _identity && roomSource == _roomSource;
                _committedValid = contiguous;
                if (contiguous) {
                    _committed = _pending;
                    _committedRaw = _pendingRaw;
                }
                _pendingFrame = frame;
                _pendingValid = false;
            }
            _identity = identity;
            _roomSource = roomSource;
            const auto local = transform_math::composeTransforms(transform_math::invertTransform(room), target);
            auto filtered = local;
            _settling = false;
            if (_committedValid && holdOffset) {
                filtered = transform_math::composeTransforms(local,
                    transform_math::composeTransforms(transform_math::invertTransform(_committedRaw), _committed));
            } else if (_committedValid && std::isfinite(dt) && dt > 0.0f) {
                if (config.rotationSeconds <= 0.0f && config.translationSeconds <= 0.0f) {
                    constexpr float kSettledGameUnits = 0.01f;
                    constexpr float kSettledDegrees = 0.01f;
                    _settling = hand_world_claim_registry_policy::translationDeltaGameUnits(_committed, _committedRaw) > kSettledGameUnits ||
                        hand_world_claim_registry_policy::rotationDeltaDegrees(_committed, _committedRaw) > kSettledDegrees;
                    if (_settling) config = { 0.05f, 0.025f, 0.0f };
                }
                if (config.rotationSeconds > 0.0f || config.translationSeconds > 0.0f) {
                    float scale = 1.0f;
                    if (config.releaseDegreesPerSecond > 0.0f) {
                        const float speed = hand_world_claim_registry_policy::rotationDeltaDegrees(_committedRaw, local) / dt;
                        if (std::isfinite(speed)) scale = config.releaseDegreesPerSecond / (config.releaseDegreesPerSecond + speed);
                    }
                    const auto alpha = [dt, scale](float tau) { return tau > 0.0f ? -std::expm1(-dt / (tau * scale)) : 1.0f; };
                    const auto previousPivot = transform_math::localPointToWorld(_committed, pivotLocal);
                    const auto pivot = previousPivot +
                        (transform_math::localPointToWorld(local, pivotLocal) - previousPivot) * alpha(config.translationSeconds);
                    filtered.rotate = hand_visual_lerp_math::quaternionToMatrix<RE::NiMatrix3>(
                        hand_visual_lerp_math::slerp(hand_visual_lerp_math::matrixToQuaternion(_committed.rotate),
                            hand_visual_lerp_math::matrixToQuaternion(local.rotate), alpha(config.rotationSeconds)));
                    filtered.translate = {};
                    filtered.translate = pivot - transform_math::localPointToWorld(filtered, pivotLocal);
                }
            }
            const auto world = transform_math::composeTransforms(room, filtered);
            if (!usable(world)) {
                reset();
                return false;
            }
            _pending = filtered;
            _pendingRaw = local;
            _pendingValid = true;
            output = world;
            return true;
        }

    private:
        RE::NiTransform _committed{};
        RE::NiTransform _committedRaw{};
        RE::NiTransform _pending{};
        RE::NiTransform _pendingRaw{};
        const void* _identity = nullptr;
        const void* _roomSource = nullptr;
        std::uint64_t _pendingFrame = 0;
        bool _committedValid = false;
        bool _pendingValid = false;
        bool _settling = false;
    };
}
