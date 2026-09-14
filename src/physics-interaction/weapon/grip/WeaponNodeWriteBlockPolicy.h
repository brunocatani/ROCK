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
 * recoil envelope, which writes the node without owning the weapon. The short
 * tail only bridges a missed write. Recoil writes keep a longer tail so a
 * semi-automatic cadence does not engage and release the block on every round:
 * FRIK logs each edge, and each engage exits its weapon reposition mode.
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
    };

    [[nodiscard]] constexpr bool shouldHoldWriteBlock(const OwnershipInput& input) noexcept
    {
        return input.framesSinceWrite < kHoldFramesAfterWrite ||
               input.framesSinceRecoilWrite < kHoldFramesAfterRecoilWrite ||
               input.ownsWeaponTransform ||
               input.weaponReturnActive ||
               input.leftCarryActive ||
               input.oneHandRecoilActive;
    }
}
