#pragma once

#include <cstdint>

/*
 * When ROCK holds FRIK's primary weapon-node write block (FRIK API v2.3,
 * blockPrimaryWeaponNodeOwnership under ROCK's weapon-authority tag).
 *
 * FRIK's weapon pass runs after ROCK's AfterArmSolve callback, so any frame
 * in which ROCK wrote the node must end with the block held or FRIK re-applies
 * its own offset. The block is a state, not a per-frame flag: FRIK runs its
 * weapon reposition reset on every engage edge and re-applies its offset
 * local on every release edge, so a write keeps it held for a short tail.
 * The ownership predicates cover every legitimate hold, including the one-hand
 * recoil envelope, which writes the node without owning the weapon, and the
 * authored primary alignment, which writes the node on every frame it is
 * active. The short tail only bridges a missed write. Recoil writes keep a
 * longer tail so a semi-automatic cadence does not engage and release the
 * block on every round: FRIK logs each edge, and each engage exits its weapon
 * reposition mode. A tail is a backstop, not an owner: holdReason() names the
 * tail that holds the block so a missing predicate shows up in the log
 * instead of hiding behind the timer.
 */
namespace rock::weapon_node_write_block_policy
{
    inline constexpr std::uint32_t kHoldFramesAfterWrite = 4;
    // Bridges the gap between rounds of semi-automatic fire.
    inline constexpr std::uint32_t kHoldFramesAfterRecoilWrite = 30;
    inline constexpr std::uint32_t kNeverWritten = 0xFFFFFFFFu;

    [[nodiscard]] constexpr std::uint32_t advanceFramesSinceWrite(
        const std::uint32_t framesSinceWrite,
        const bool wroteThisFrame) noexcept
    {
        if (wroteThisFrame) {
            return 0;
        }
        return framesSinceWrite == kNeverWritten ? kNeverWritten : framesSinceWrite + 1;
    }

    struct OwnershipInput
    {
        std::uint32_t framesSinceWrite = kNeverWritten;
        // Frames since the last one-hand recoil write.
        std::uint32_t framesSinceRecoilWrite = kNeverWritten;
        // Full two-hand authority or part carry.
        bool ownsWeaponTransform = false;
        bool weaponReturnActive = false;
        // Left-firing carry in any manual-ownership state.
        bool leftCarryActive = false;
        // The right one-hand recoil pose is away from rest.
        bool oneHandRecoilActive = false;
        // The authored primary grip alignment writes the node every frame it is active.
        bool authoredPrimaryAlignmentActive = false;
    };

    enum class HoldReason : std::uint8_t
    {
        None,
        Predicate,
        WriteTail,
        RecoilTail,
    };

    [[nodiscard]] constexpr const char* holdReasonName(const HoldReason reason) noexcept
    {
        switch (reason) {
        case HoldReason::Predicate:
            return "predicate";
        case HoldReason::WriteTail:
            return "write";
        case HoldReason::RecoilTail:
            return "recoil";
        case HoldReason::None:
        default:
            return "none";
        }
    }

    // Predicates first: a tail is reported only when no predicate holds. The
    // recoil tail outlasts the write tail, so it is the one reported while both run.
    [[nodiscard]] constexpr HoldReason holdReason(const OwnershipInput& input) noexcept
    {
        if (input.ownsWeaponTransform || input.weaponReturnActive || input.leftCarryActive ||
            input.oneHandRecoilActive || input.authoredPrimaryAlignmentActive) {
            return HoldReason::Predicate;
        }
        if (input.framesSinceRecoilWrite < kHoldFramesAfterRecoilWrite) {
            return HoldReason::RecoilTail;
        }
        if (input.framesSinceWrite < kHoldFramesAfterWrite) {
            return HoldReason::WriteTail;
        }
        return HoldReason::None;
    }

    [[nodiscard]] constexpr bool shouldHoldWriteBlock(const OwnershipInput& input) noexcept
    {
        return holdReason(input) != HoldReason::None;
    }
}
