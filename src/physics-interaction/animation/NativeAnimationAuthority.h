#pragma once

#include <cstdint>

namespace rock::native_animation_authority
{
    enum class RuntimeStatusFlag : std::uint32_t
    {
        None = 0,
        HookInstalled = 1u << 0,
        RuntimeEnabled = 1u << 1,
        CaptureValid = 1u << 2,
        LocalReloadTestLeaseActive = 1u << 3,
        HookInstallFailed = 1u << 4,
        ThreadMismatch = 1u << 5,
        CaptureFault = 1u << 6,
    };

    struct RuntimeStatus
    {
        std::uint32_t effectiveFlags{ 0 };
        std::uint32_t statusFlags{ 0 };
        std::uint32_t capturedTransformCount{ 0 };
        std::uint64_t captureSequence{ 0 };
    };

    // Install only after FRIK has emitted kSkeletonReady and has already
    // applied its verified PostUpdateAnimationGraphManager NOP patch.
    [[nodiscard]] bool installPostUpdateHook();

    void setRuntimeEnabled(bool enabled);
    void requestLocalReloadTestLease();
    void beginRockFrame();
    [[nodiscard]] bool applyCapturedPose();
    void completeRockFrame();
    void resetTransientState();

    [[nodiscard]] bool isHookInstalled();
    [[nodiscard]] RuntimeStatus queryRuntimeStatus();
}
