#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstdint>

namespace RE
{
    class NiNode;
}

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

    struct PrimaryFiringGripCaptureStatus
    {
        std::uint64_t captureSequence{ 0 };
        bool valid{ false };
    };

    enum class AuthoredSupportGripCaptureFailureReason : std::uint32_t
    {
        None = 0,
        SecondaryPassNotObserved,
        SourceTreeUnavailable,
        BoneCacheIncomplete,
        TopologyInvalid,
        PrimaryPoseUnavailable,
        AuthoredGraphPoseUnavailable,
        AuthoredHandHierarchyInvalid,
        SupportHandTransformInvalid,
        FingerTransformInvalid,
        ThreadMismatch,
        CaptureFault,
    };

    struct AuthoredSupportGripCaptureStatus
    {
        std::uint64_t captureSequence{ 0 };
        std::uint64_t secondaryPassSequence{ 0 };
        AuthoredSupportGripCaptureFailureReason failureReason{
            AuthoredSupportGripCaptureFailureReason::SecondaryPassNotObserved
        };
        std::uint16_t invalidOrMissingFingerMask{ 0 };
        bool valid{ false };
    };

    // Install only after FRIK has emitted kSkeletonReady and has already
    // applied its verified PostUpdateAnimationGraphManager NOP patch.
    [[nodiscard]] bool installPostUpdateHook();

    void setRuntimeEnabled(bool enabled);
    // The existing native-reload experiment also owns a ROCK-local manual
    // weapon-cycle window. It applies native arms/hands while preserving the
    // live ROCK weapon world transform.
    void setLocalManualCycleTestEnabled(bool enabled);
    // Independent ROCK-only experiment capture. A byte-validated native arm
    // hook records Bethesda's paired primary/support poses before hFRIK
    // replaces them. The runtime inverts the primary relation onto the live
    // controller and offers the support relation as a proximity-only grip
    // candidate; dynamic support grabbing remains the fallback.
    void setPrimaryFiringGripCaptureEnabled(bool enabled);
    [[nodiscard]] PrimaryFiringGripCaptureStatus queryPrimaryFiringGripCaptureStatus();
    [[nodiscard]] AuthoredSupportGripCaptureStatus queryAuthoredSupportGripCaptureStatus();
    [[nodiscard]] const char* authoredSupportGripCaptureFailureReasonName(
        AuthoredSupportGripCaptureFailureReason reason);
    [[nodiscard]] bool tryResolvePrimaryFiringGripAlignment(
        const RE::NiNode* expectedWeaponNode,
        const RE::NiTransform& liveWeaponWorld,
        const RE::NiTransform& trackedPrimaryHandWorld,
        RE::NiTransform& outWeaponWorld,
        RE::NiTransform& outCurrentAuthoredHandWorld,
        RE::NiTransform& outAuthoredPrimaryHandInWeapon,
        std::uint64_t& outCaptureSequence);
    [[nodiscard]] bool tryResolveAuthoredSupportGrip(
        const RE::NiNode* expectedWeaponNode,
        RE::NiTransform& outSupportHandInWeapon,
        std::array<RE::NiTransform, 15>& outFingerLocalTransforms,
        std::uint16_t& outFingerLocalTransformMask,
        std::uint64_t& outCaptureSequence);
    void requestLocalReloadTestLease();
    void beginRockFrame(float deltaSeconds);
    [[nodiscard]] bool applyCapturedPose();
    void completeRockFrame();
    void resetTransientState();

    [[nodiscard]] bool isHookInstalled();
    [[nodiscard]] RuntimeStatus queryRuntimeStatus();
}
