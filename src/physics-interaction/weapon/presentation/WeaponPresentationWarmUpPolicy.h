#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::weapon_presentation_warm_up_policy
{
    /*
     * Whether a dynamic weapon collision correction may reach presentation.
     *
     * Collision bodies are allowed to exist before this is true. Hand claims
     * and weapon writes are not. Two distinct warm-up windows sit between
     * "the weapon is equipped" and "a correction means something":
     *
     *  1. The equip handover. Until the native weapon is renderable and the
     *     hand pose handoff has completed, the weapon and the hands are still
     *     being placed, and a correction describes neither.
     *
     *  2. The hand/weapon pair suppression. A hand that owns the weapon must
     *     not also solve against it. Until that per-pair lease is live the
     *     owning hand's proxy overlaps the weapon proxy and produces a
     *     correction no real contact caused.
     *
     * A hand's publication channel must also be open, or the claim that
     * carries the correction cannot be honoured at all.
     *
     * Index 0 is the left hand and index 1 the right, matching the rest of
     * the presentation code.
     */
    struct Inputs
    {
        std::array<bool, 2> handAttached{};
        std::array<bool, 2> handPublicationReady{};
        std::array<bool, 2> handWeaponPairSuppressed{};
        bool nativeRenderable = false;
        bool handPoseHandoffComplete = false;
        bool pairFilterReady = false;
    };

    enum class BlockReason : std::uint8_t
    {
        None = 0,
        WeaponNotRenderable,
        HandPoseHandoffPending,
        PairFilterUnavailable,
        HandPublicationClosed,
        HandWeaponPairNotSuppressed,
    };

    [[nodiscard]] inline constexpr BlockReason blockReason(
        const Inputs& inputs) noexcept
    {
        if (!inputs.nativeRenderable) {
            return BlockReason::WeaponNotRenderable;
        }
        if (!inputs.handPoseHandoffComplete) {
            return BlockReason::HandPoseHandoffPending;
        }
        if (!inputs.pairFilterReady) {
            return BlockReason::PairFilterUnavailable;
        }
        for (std::size_t index = 0; index < inputs.handAttached.size();
             ++index) {
            if (!inputs.handAttached[index]) {
                continue;
            }
            if (!inputs.handPublicationReady[index]) {
                return BlockReason::HandPublicationClosed;
            }
            if (!inputs.handWeaponPairSuppressed[index]) {
                return BlockReason::HandWeaponPairNotSuppressed;
            }
        }
        return BlockReason::None;
    }

    [[nodiscard]] inline constexpr bool isWarmedUp(
        const Inputs& inputs) noexcept
    {
        return blockReason(inputs) == BlockReason::None;
    }

    [[nodiscard]] inline constexpr const char* blockReasonName(
        const BlockReason reason) noexcept
    {
        switch (reason) {
        case BlockReason::None:
            return "none";
        case BlockReason::WeaponNotRenderable:
            return "weapon-not-renderable";
        case BlockReason::HandPoseHandoffPending:
            return "hand-pose-handoff-pending";
        case BlockReason::PairFilterUnavailable:
            return "pair-filter-unavailable";
        case BlockReason::HandPublicationClosed:
            return "hand-publication-closed";
        case BlockReason::HandWeaponPairNotSuppressed:
            return "hand-weapon-pair-not-suppressed";
        default:
            return "unknown";
        }
    }
}
