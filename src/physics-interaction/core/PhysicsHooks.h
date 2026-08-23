#pragma once

#include <cstdint>

#include "physics-interaction/timing/GameFrameTimingPolicy.h"

namespace rock
{
    void installBumpHook();
    bool installHavokTimingFixHook();
    void applyHavokTimingFixForGameFrame(const game_frame_timing_policy::GameFrameTiming& frameTiming);
    void installNativeGrabHook();
    bool validateNativeMeleeSuppressionHookTargets();
    bool installNativeMeleeSuppressionHooks();
    void enforceNativeMeleeRuntimeSuppression(bool forceCheck = false);
    void enforceNativeGrabHapticRuntimeSuppression(bool forceCheck = false);
    void installRefreshManifoldHook();

    void advanceNativeMeleeFrameClock();
    void clearNativeMeleePhysicalSwingLeases();
    void setNativeMeleePhysicalSwingActive(bool isLeft, bool active);
    bool isNativeMeleePhysicalSwingActive(bool isLeft);
}
