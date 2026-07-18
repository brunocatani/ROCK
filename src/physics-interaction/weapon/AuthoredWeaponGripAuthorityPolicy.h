#pragma once

#include <cstdint>

namespace rock::authored_weapon_grip_authority_policy
{
    inline constexpr std::uint16_t kCompleteFiringFingerMask = 0x7FFF;

    [[nodiscard]] constexpr bool completeFiringFingerPose(const std::uint16_t enabledMask) noexcept { return enabledMask == kCompleteFiringFingerMask; }

    /*
     * Live equipped capture is retained only as a compatibility fallback.
     * Once the matching off-screen idle asset has been sampled, later live
     * frames cannot replace that stable authored relation or erase its exact
     * finger pose.
     */
    [[nodiscard]] constexpr bool shouldAcceptPublication(const bool sameIdentity, const bool existingIsNativeIdlePreharvest, const bool incomingIsNativeIdlePreharvest) noexcept
    {
        return !sameIdentity || !existingIsNativeIdlePreharvest || incomingIsNativeIdlePreharvest;
    }
}
