#pragma once

#include <cstdint>

namespace rock::car_interaction_policy
{
    // Fallout4.esm KYWD 0x0021C00D. Vanilla vehicle MSTTs use this identity.
    inline constexpr std::uint32_t kExplodableCarKeywordFormId = 0x0021C00D;

    struct GrabPolicyInput
    {
        bool targetIsCar = false;
        bool playerInPowerArmor = false;
    };

    struct GrabPolicyDecision
    {
        bool allowed = true;
        const char* reason = "not-car";
    };

    inline constexpr GrabPolicyDecision evaluateGrab(const GrabPolicyInput& input)
    {
        if (!input.targetIsCar) {
            return GrabPolicyDecision{ .allowed = true, .reason = "not-car" };
        }
        if (!input.playerInPowerArmor) {
            return GrabPolicyDecision{ .allowed = false, .reason = "car-requires-power-armor" };
        }
        return GrabPolicyDecision{ .allowed = true, .reason = "power-armor-car-body" };
    }
}
