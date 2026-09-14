#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstdint>

namespace RE
{
    class NiNode;
}

namespace rock::authored_weapon_grip_capture
{
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

    // Installs ROCK's validated UpdateFirstPersonArm grip capture. The native
    // graph-output point is FRIK's NativeGraphOutput frame phase (API v2.3),
    // forwarded by ROCKMain to onNativeGraphOutput. Reload/bolt capture,
    // application, WeaponFire, and ReloadStateChange behavior remain
    // animation-addon responsibilities.
    [[nodiscard]] bool installHook();
    // FRIK's NativeGraphOutput phase: dispatch the ROCK V1 graph-output phase
    // to provider consumers, then capture the authored support-grip pose.
    void onNativeGraphOutput();
    [[nodiscard]] bool isHookInstalled();
    void setEnabled(bool enabled);
    void resetTransientState();

    [[nodiscard]] PrimaryFiringGripCaptureStatus
        queryPrimaryFiringGripCaptureStatus();
    [[nodiscard]] AuthoredSupportGripCaptureStatus
        queryAuthoredSupportGripCaptureStatus();
    [[nodiscard]] const char* authoredSupportGripCaptureFailureReasonName(
        AuthoredSupportGripCaptureFailureReason reason);
    [[nodiscard]] bool tryGetPrimaryFiringGripRelation(
        const RE::NiNode* expectedWeaponNode,
        RE::NiTransform& outAuthoredPrimaryHandInWeapon,
        std::uint64_t& outCaptureSequence);
    [[nodiscard]] bool tryResolveAuthoredSupportGrip(
        const RE::NiNode* expectedWeaponNode,
        RE::NiTransform& outSupportHandInWeapon,
        std::array<RE::NiTransform, 15>& outFingerLocalTransforms,
        std::uint16_t& outFingerLocalTransformMask,
        std::uint64_t& outCaptureSequence);
}
