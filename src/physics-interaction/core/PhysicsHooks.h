#pragma once

#include "RE/NetImmerse/NiPoint.h"

#include <cstdint>

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
    // World-space room translation VECTOR (game units) applied in the last aligned movement hook, with its
    // engine dt and offset-free speed. Returns false when no usable delta is available this frame.
    bool getAlignedRoomWorldDelta(RE::NiPoint3& outDeltaGameUnits, float& outTimeDelta, float& outSpeedGameUnits);
    // Advances the drift-free grab-proxy room-correction offset once per frame (idempotent within a frame).
    // renderRoomDeltaGameUnits is ROCK's render-sampled room translation this frame (the aliased delta).
    void updateAlignedRoomCorrectionForFrame(std::uint64_t frameIndex, const RE::NiPoint3& renderRoomDeltaGameUnits, bool renderValid);
    // Current bounded room-correction offset (game units) to add to the grab proxy target.
    RE::NiPoint3 getAlignedRoomCorrectionOffset();
    void enforceNativeMeleeRuntimeSuppression(bool forceCheck = false);
    void enforceNativeGrabHapticRuntimeSuppression(bool forceCheck = false);
    void installRefreshManifoldHook();

    void advanceNativeMeleeFrameClock();
    void clearNativeMeleePhysicalSwingLeases();
    void setNativeMeleePhysicalSwingActive(bool isLeft, bool active);
    bool isNativeMeleePhysicalSwingActive(bool isLeft);
}
