#pragma once

namespace rock::dynamic_hand_collision_policy
{
    // These values define mandatory production behavior. They are not runtime
    // tuning controls and must not be loaded from ROCK.ini.
    inline constexpr float kMaximumLinearVelocityHavok = 15.0f;
    inline constexpr float kContactPressMaximumVelocityHavok = 1.0f;
    inline constexpr float kDivergenceTeleportDistanceGameUnits = 40.0f;
    inline constexpr float kDivergenceTeleportDwellSeconds = 0.3f;
    inline constexpr float kTeleportRecoverySeconds = 0.25f;
    inline constexpr float kRenderFollowMinimumDeviationGameUnits = 0.05f;
    inline constexpr float kRenderFollowSmoothingSpeed = 45.0f;
    inline constexpr int kVisualPriority = 80;

    inline constexpr float kHapticDurationSeconds = 0.035f;
    inline constexpr float kHapticBaseIntensity = 0.18f;
    inline constexpr float kHapticMaximumIntensity = 0.55f;
    inline constexpr float kHapticSpeedScale = 0.006f;
    inline constexpr float kHapticMinimumApproachSpeedGameUnitsPerSecond = 3.0f;
    inline constexpr float kHapticCooldownSeconds = 0.12f;

    inline constexpr float kSurfaceFingerProbeDeltaOpenUnits = 0.10f;
    inline constexpr float kSurfaceFingerResponseGain = 1.0f;
    inline constexpr float kSurfaceFingerMaximumDeflectionOpenUnits = 0.85f;
    inline constexpr float kSurfaceFingerMinimumHelpfulTravelGameUnits = 0.01f;
    inline constexpr float kSurfaceFingerDirectionSwitchHysteresisFraction = 0.10f;
    inline constexpr float kSurfaceFingerSmoothingSpeed = 30.0f;
    inline constexpr float kSurfaceFingerReleaseDelaySeconds = 0.12f;

    static_assert(kVisualPriority >= 0 && kVisualPriority <= 99);
    static_assert(kHapticBaseIntensity >= 0.0f);
    static_assert(kHapticMaximumIntensity >= kHapticBaseIntensity && kHapticMaximumIntensity <= 1.0f);
    static_assert(kSurfaceFingerProbeDeltaOpenUnits > 0.0f);
    static_assert(kSurfaceFingerMaximumDeflectionOpenUnits >= 0.0f && kSurfaceFingerMaximumDeflectionOpenUnits <= 1.0f);
}
