// Native hooks implemented in PhysicsHooks.cpp: the Havok timing fix
// (installed once by ROCKMain.cpp), the native grab and refresh-manifold hooks
// (installed by the PhysicsInteraction constructor), native melee suppression
// (installed by PhysicsInteraction::init) and native grab-haptic suppression.
#pragma once

#include <cstdint>

namespace rock
{
    bool installHavokTimingFixHook();
    void installNativeGrabHook();
    bool validateNativeMeleeSuppressionHookTargets();
    bool installNativeMeleeSuppressionHooks();
    void enforceNativeMeleeRuntimeSuppression(bool forceCheck = false);
    [[nodiscard]] bool isNativeMeleeSuppressionActive();
    void enforceNativeGrabHapticRuntimeSuppression(bool forceCheck = false);
    void installRefreshManifoldHook();

    void advanceNativeRuntimeSettingFrameClock();
}
