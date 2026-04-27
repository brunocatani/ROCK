#pragma once

namespace frik::rock
{
    void installCCRadiusHook();
    void installBumpHook();
    void installNativeGrabHook();
    bool validateNativeMeleeSuppressionHookTargets();
    bool installNativeMeleeSuppressionHooks();
    void installRefreshManifoldHook();

    void setNativeMeleePhysicalSwingActive(bool isLeft, bool active);
    bool isNativeMeleePhysicalSwingActive(bool isLeft);
}
