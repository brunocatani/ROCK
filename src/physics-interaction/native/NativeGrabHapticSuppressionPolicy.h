#pragma once

#include <array>
#include <string_view>

namespace rock::native_grab_haptic_suppression
{
    inline constexpr char kRolloverRumbleEnabledSetting[] = "bRumbleOnRollover:VRInterface";
    inline constexpr char kHoverRumbleIntensitySetting[] = "fVRInputHoverRumbleIntensity:VRWand";
    inline constexpr char kHoverRumbleDurationSetting[] = "fVRInputHoverRumbleDuration:VRWand";

    inline constexpr bool kSuppressedRolloverRumbleEnabled = false;
    inline constexpr float kSuppressedHoverRumbleFloat = 0.0f;

    inline constexpr std::array<std::string_view, 3> kSuppressedIniSettings{
        kRolloverRumbleEnabledSetting,
        kHoverRumbleIntensitySetting,
        kHoverRumbleDurationSetting,
    };

    struct RuntimeInput
    {
        bool rockEnabled{ true };
        bool suppressionEnabled{ true };
    };

    [[nodiscard]] constexpr bool shouldSuppressNativeGrabHoverHaptics(RuntimeInput input) noexcept
    {
        return input.rockEnabled && input.suppressionEnabled;
    }

    [[nodiscard]] constexpr bool shouldRestoreNativeGrabHoverHaptics(bool previouslyApplied, RuntimeInput input) noexcept
    {
        return previouslyApplied && !shouldSuppressNativeGrabHoverHaptics(input);
    }
}
