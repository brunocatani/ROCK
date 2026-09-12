#pragma once

namespace rock::equip_visual_bridge_policy
{
    // The detached loose model is visual-only. It may cover the native
    // inventory-to-first-person gap, but it must never become a persistent
    // substitute for the equipped weapon's render and collision authority.
    constexpr float kMaximumPresentationLeaseSeconds = 1.0f;

    [[nodiscard]] inline constexpr float effectivePresentationLeaseSeconds(
        const float requestedSeconds) noexcept
    {
        if (!(requestedSeconds > 0.0f)) {
            return kMaximumPresentationLeaseSeconds;
        }
        return requestedSeconds < kMaximumPresentationLeaseSeconds ?
            requestedSeconds :
            kMaximumPresentationLeaseSeconds;
    }

    [[nodiscard]] inline constexpr bool presentationLeaseExpired(
        const float elapsedSeconds,
        const float requestedSeconds) noexcept
    {
        return elapsedSeconds >=
               effectivePresentationLeaseSeconds(requestedSeconds);
    }
}
