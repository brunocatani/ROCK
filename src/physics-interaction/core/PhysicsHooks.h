#pragma once

#include <cstdint>

namespace rock
{
    void installBumpHook();
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
