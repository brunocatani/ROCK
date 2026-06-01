#pragma once

namespace rock
{
    void installBumpHook();
    bool installHavokTimingFixHook();
    void installNativeGrabHook();
    bool validateNativeMeleeSuppressionHookTargets();
    bool installNativeMeleeSuppressionHooks();
    void enforceNativeMeleeRuntimeSuppression(bool forceCheck = false);
    void installRefreshManifoldHook();

    void advanceNativeMeleeFrameClock();
    void clearNativeMeleePhysicalSwingLeases();
    void setNativeMeleePhysicalSwingActive(bool isLeft, bool active);
    bool isNativeMeleePhysicalSwingActive(bool isLeft);
}
