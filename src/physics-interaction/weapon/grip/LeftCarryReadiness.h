#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include "physics-interaction/weapon/AuthoredSupportGrabPolicy.h"
#include "physics-interaction/weapon/PipboyEquipPolicy.h"

#include <cmath>
#include <cstdint>

// Shared readiness sampling for handing the equipped weapon carrier to the
// left firing hand. Every driver of beginPersistentEquippedCarry must observe
// the same two facts through the same witnesses:
//
// 1. hFRIK owns the native-right local offset (custom, melee, PA, in-session
//    values). ROCK never duplicates its placement rules; it rebase-samples
//    weaponNode->local until that live authority holds stable for two
//    consecutive frames, then trusts it as the canonical capture baseline.
// 2. TwoHandedGrip reports the generation-bound left takeover as ready.
//
// The witnesses also carry the last observed takeover readiness so callers
// can log on change only, keeping per-driver log context at the call site.

namespace rock::left_carry_readiness
{
    [[nodiscard]] inline bool finiteTransform(
        const RE::NiTransform& transform) noexcept
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
               std::abs(transform.scale) > 0.0001f;
    }

    [[nodiscard]] inline bool approximatelySameLocalOffset(
        const RE::NiTransform& live,
        const RE::NiTransform& expected) noexcept
    {
        constexpr float kMaximumTranslationError = 0.05f;
        constexpr float kMaximumRotationElementError = 0.001f;
        constexpr float kMaximumScaleError = 0.001f;
        if (!finiteTransform(live) || !finiteTransform(expected)) {
            return false;
        }

        const float dx = live.translate.x - expected.translate.x;
        const float dy = live.translate.y - expected.translate.y;
        const float dz = live.translate.z - expected.translate.z;
        if (dx * dx + dy * dy + dz * dz >
                kMaximumTranslationError * kMaximumTranslationError ||
            std::abs(live.scale - expected.scale) > kMaximumScaleError) {
            return false;
        }

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (std::abs(live.rotate.entry[row][column] -
                        expected.rotate.entry[row][column]) >
                    kMaximumRotationElementError) {
                    return false;
                }
            }
        }
        return true;
    }

    struct NativeOffsetWitness
    {
        RE::NiTransform sample{};
        bool sampleValid{ false };
        std::uint8_t matchingFrames{ 0 };
    };

    // Advances the stable-offset observation against the live native-right
    // local transform. Returns true once the offset has held stable long
    // enough (per pipboy_equip_policy) to trust as the canonical baseline.
    [[nodiscard]] inline bool advanceNativeOffset(
        NativeOffsetWitness& witness,
        const RE::NiTransform& liveLocal) noexcept
    {
        const bool liveFinite = finiteTransform(liveLocal);
        bool liveMatches = witness.sampleValid && liveFinite &&
            approximatelySameLocalOffset(liveLocal, witness.sample);
        if (liveFinite && !liveMatches) {
            witness.sample = liveLocal;
            witness.sampleValid = true;
            witness.matchingFrames = 0;
            liveMatches = true;
        }
        return pipboy_equip_policy::advanceNativeOffsetReadiness(
            witness.sampleValid,
            liveMatches,
            witness.matchingFrames);
    }

    struct TakeoverWitness
    {
        authored_support_grab_policy::LeftFiringTakeoverReadiness last{
            authored_support_grab_policy::LeftFiringTakeoverReadiness::
                NotRequired
        };

        // Records the observation; returns true when it changed so the
        // caller can emit its one log-on-change diagnostic.
        [[nodiscard]] bool observe(
            const authored_support_grab_policy::LeftFiringTakeoverReadiness
                current) noexcept
        {
            if (last == current) {
                return false;
            }
            last = current;
            return true;
        }
    };
}
