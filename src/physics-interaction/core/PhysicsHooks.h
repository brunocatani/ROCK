#pragma once

namespace rock
{
    void installBumpHook();
    bool installHavokTimingFixHook();
    void installNativeGrabHook();
    bool validateNativeMeleeSuppressionHookTargets();
    bool installNativeMeleeSuppressionHooks();

    // Installs the PlayerCharacter::ApplyMovementDelta vtable hook that captures the aligned-timing per-frame
    // room translation (held-object stick-locomotion stutter fix). Player-only, validate-before-swap.
    bool installLocomotionAuthorityHook();
    // Actual room translation speed (game units/sec) from the last aligned movement hook; -1 if unavailable.
    float getAlignedRoomSpeedGameUnits();
    void enforceNativeMeleeRuntimeSuppression(bool forceCheck = false);
    void enforceNativeGrabHapticRuntimeSuppression(bool forceCheck = false);
    void installRefreshManifoldHook();

    void advanceNativeMeleeFrameClock();
    void clearNativeMeleePhysicalSwingLeases();
    void setNativeMeleePhysicalSwingActive(bool isLeft, bool active);
    bool isNativeMeleePhysicalSwingActive(bool isLeft);
}
