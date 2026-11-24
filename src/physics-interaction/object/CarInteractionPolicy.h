#pragma once

namespace rock::car_interaction_policy
{
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
