#pragma once

// The FO4VR settings behind native grab-hover rumble, which ROCK always holds off
// (its own haptics replace it).
namespace rock::native_grab_haptic_suppression
{
    inline constexpr char kRolloverRumbleEnabledSetting[] = "bRumbleOnRollover:VRInterface";
    inline constexpr char kHoverRumbleIntensitySetting[] = "fVRInputHoverRumbleIntensity:VRWand";
    inline constexpr char kHoverRumbleDurationSetting[] = "fVRInputHoverRumbleDuration:VRWand";

    inline constexpr bool kSuppressedRolloverRumbleEnabled = false;
    inline constexpr float kSuppressedHoverRumbleFloat = 0.0f;
}
