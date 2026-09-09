#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "physics-interaction/hand/HandVisual.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

/*
 * Prediction of FRIK's hand dampening for the pre-FRIK claim rebase.
 *
 * FRIK (Skeleton::dampenHand) filters the weapon offset node's world at its
 * frame start: the rotation is slerped from the previous dampened world
 * toward the raw world by (1 - rotation factor); the translation moves by
 * (1 - translation factor) of the raw step with the camera's own step
 * removed first (player movement is not dampened). The seats ROCK publishes
 * are read from that dampened node after FRIK's frame, so a claim rebased
 * by the raw chain hands FRIK a target one raw step ahead of the seat. The
 * pass predicts the dampened node with the same filter from the node ROCK
 * read last frame; the claim FRIK consumes then equals the seat up to
 * prediction error. Measured on 2026-09-08: FRIK's input is ROCK's composed
 * raw sample (mean 0.008 gu), and this formula reproduces its output.
 */
namespace rock::dampened_driver_prediction_policy
{
    struct DampenFactors
    {
        bool enabled = false;
        // FRIK keeps this fraction of the previous value (0.6 = FRIK's default feel).
        float translation = 0.0f;
        float rotation = 0.0f;
    };

    // FRIK's DampenHands* configuration: the normal factors and the vanilla scope menu factors.
    struct FrikDampenConfig
    {
        DampenFactors normal{};
        DampenFactors vanillaScope{};
        bool valid = false;
    };

    // One post-FRIK observation and the camera sample from that same pass.
    // Keep observing while dampening is off: a pre-scope world is not a
    // previous-frame pose after walking in the scope.
    struct ObservedDriver
    {
        RE::NiTransform world{};
        RE::NiPoint3 camera{};
        std::uint64_t sequence = 0;
        bool dampeningEnabled = false;
        bool valid = false;
    };

    enum class PredictionMode : std::uint8_t
    {
        RawUnavailable,
        RawDisabled,
        RawDiscontinuous,
        RawResuming,
        Dampened,
    };

    [[nodiscard]] inline const char* predictionModeName(const PredictionMode mode) noexcept
    {
        switch (mode) {
        case PredictionMode::RawDisabled: return "raw-disabled";
        case PredictionMode::RawDiscontinuous: return "raw-discontinuous";
        case PredictionMode::RawResuming: return "raw-resuming";
        case PredictionMode::Dampened: return "dampened";
        default: return "raw-unavailable";
        }
    }

    struct Prediction
    {
        RE::NiTransform world{};
        PredictionMode mode = PredictionMode::RawUnavailable;
    };

    [[nodiscard]] inline bool isUsableFactor(const float factor) noexcept
    {
        return std::isfinite(factor) && factor >= 0.0f && factor < 0.999f;
    }

    [[nodiscard]] inline DampenFactors selectFactors(const FrikDampenConfig& config, const bool inVanillaScopeMenu) noexcept
    {
        if (!config.valid) {
            return {};
        }
        return inVanillaScopeMenu ? config.vanillaScope : config.normal;
    }

    /*
     * The dampened world FRIK will write for a raw world, given the dampened
     * world it wrote last frame and the camera step since then. The raw world
     * when the factors are off.
     */
    [[nodiscard]] inline RE::NiTransform predictDampened(
        const RE::NiTransform& raw,
        const RE::NiTransform& previousDampened,
        const RE::NiPoint3& cameraDelta,
        const DampenFactors& factors)
    {
        RE::NiTransform result = raw;
        if (!factors.enabled) {
            return result;
        }
        const float translationFactor = std::clamp(factors.translation, 0.0f, 0.999f);
        const float rotationFactor = std::clamp(factors.rotation, 0.0f, 0.999f);
        result.translate.x = raw.translate.x - translationFactor * (raw.translate.x - previousDampened.translate.x - cameraDelta.x);
        result.translate.y = raw.translate.y - translationFactor * (raw.translate.y - previousDampened.translate.y - cameraDelta.y);
        result.translate.z = raw.translate.z - translationFactor * (raw.translate.z - previousDampened.translate.z - cameraDelta.z);
        if (rotationFactor > 0.0f) {
            namespace lerp_math = hand_visual_lerp_math;
            result.rotate = lerp_math::quaternionToMatrix<RE::NiMatrix3>(
                lerp_math::slerp(lerp_math::matrixToQuaternion(previousDampened.rotate), lerp_math::matrixToQuaternion(raw.rotate), 1.0f - rotationFactor));
        }
        return result;
    }

    // Inputs must be finite. Only an immediately preceding observation can
    // supply filter history. September 9 runtime traces show the resumed
    // driver following raw input on the first frame with dampening enabled.
    [[nodiscard]] inline Prediction predictFromObservation(
        const RE::NiTransform& raw,
        const RE::NiPoint3& camera,
        const std::uint64_t sequence,
        const DampenFactors& factors,
        const ObservedDriver& previous)
    {
        if (!factors.enabled) {
            return { raw, PredictionMode::RawDisabled };
        }
        if (!previous.valid || previous.sequence == 0 || sequence <= previous.sequence || sequence - previous.sequence != 1) {
            return { raw, PredictionMode::RawDiscontinuous };
        }
        if (!previous.dampeningEnabled) {
            return { raw, PredictionMode::RawResuming };
        }
        return { predictDampened(raw, previous.world, camera - previous.camera, factors), PredictionMode::Dampened };
    }
}
